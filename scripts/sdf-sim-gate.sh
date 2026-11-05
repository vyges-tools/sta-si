#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# SDF-annotated simulation gate: does the timer's verdict hold up in a simulation that uses the
# SDF the timer wrote?
#
# For each sample netlist in tests/sdf_sim/ and each corner (ss, tt, ff of sky130_fd_sc_hd):
#   1. run vyges-sta-si with that corner's Liberty and the sample's SDC, writing an SDF;
#   2. simulate the netlist with that SDF at the SDC's clock period (Icarus Verilog), comparing
#      every register every cycle against a behavioural model;
#   3. require that the timer and the simulation AGREE: slack >= 0 at setup and hold <=> the
#      simulation matches the model.
# Then it checks the samples do what they are for (the modified netlist fails setup at ss and
# meets it at ff, the skewed one fails hold), and that the agreement check has teeth: it must
# exit nonzero for an SDF with its delays zeroed, an SDF from the wrong corner, an SDF that
# under-states one flop's clock-to-Q, and an SDF that does not annotate at all.
#
# Not part of `cargo test`: it needs a sky130 PDK install and Icarus Verilog, neither of which CI
# carries. It is a box gate. See tests/sdf_sim/README.md for what the simulator does and does not
# support, which decides how this gate is built.
#
#   ./scripts/sdf-sim-gate.sh                       the gate
#   ./scripts/sdf-sim-gate.sh case DESIGN CORNER [MUTATION]
#        one case; exit 0 agree, 1 disagree, 2 inconclusive or not annotated
#        MUTATION: zero | corner=<corner> | zero-launch | rename
#
# Environment:
#   PDK_ROOT  sky130A install          (default ~/.ciel/sky130A)
#   STA_BIN   vyges-sta-si binary      (default: cargo build --release, then target/release)
#   WORK      scratch directory         (default: a new mktemp directory, kept and printed)
#
# Exit: 0 the gate passed; 1 it failed; 77 SKIPPED (a missing simulator or PDK). A skip is not a
# pass and never prints one.
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
T="$here/tests/sdf_sim"
PDK_ROOT="${PDK_ROOT:-$HOME/.ciel/sky130A}"
LIBDIR="$PDK_ROOT/libs.ref/sky130_fd_sc_hd/lib"
VDIR="$PDK_ROOT/libs.ref/sky130_fd_sc_hd/verilog"
CORNERS="ss_100C_1v60 tt_025C_1v80 ff_n40C_1v95"
# clock-to-Q measured in the simulation vs the SDF's IOPATH: the simulator rounds to its 1 ps
# precision, so allow a little over half of that each way
TOL=0.0015

skip() {
  echo "SKIP: $*" >&2
  echo "SKIP: sdf-sim gate NOT run (this is not a pass)" >&2
  exit 77
}

for tool in iverilog vvp jq awk; do
  command -v "$tool" >/dev/null 2>&1 || skip "$tool not found"
done
for f in "$VDIR/primitives.v" "$VDIR/sky130_fd_sc_hd.v"; do
  [ -f "$f" ] || skip "sky130 Verilog model missing: $f (set PDK_ROOT)"
done
for c in $CORNERS; do
  [ -f "$LIBDIR/sky130_fd_sc_hd__$c.lib" ] || skip "sky130 Liberty missing: $LIBDIR/sky130_fd_sc_hd__$c.lib (set PDK_ROOT)"
done

if [ -z "${STA_BIN:-}" ]; then
  if [ "${1:-}" != case ]; then
    (cd "$here" && cargo build --release --quiet --bin vyges-sta-si) || { echo "FAIL: build" >&2; exit 1; }
  fi
  STA_BIN="$here/target/release/vyges-sta-si"
fi
[ -x "$STA_BIN" ] || { echo "FAIL: no vyges-sta-si binary at $STA_BIN" >&2; exit 1; }

if [ -z "${WORK:-}" ]; then
  WORK="$(mktemp -d "${TMPDIR:-/tmp}/sdf-sim-gate.XXXXXX")"
fi
mkdir -p "$WORK"
export WORK STA_BIN

# ---- the cell models ---------------------------------------------------------------------------
# The sky130 timing models feed their flops through `D_delayed` / `CLK_delayed` nets that only a
# $setuphold with delayed-signal support drives. Icarus 12 does not support timing checks and
# leaves those nets undriven ("delayed signal ... will not be driven"), so every flop's Q stays X
# (the probe shows it). Drive each delayed net from its pin, which is what a simulator that does
# drive them does when no negative-limit adjustment applies. The copy lives in WORK only: the
# PDK is read, never vendored. SCE_gate_delayed is an internal net with its own driver.
models="$WORK/cells_icarus.v"
if [ ! -s "$models" ]; then
  sed -E '/SCE_gate_delayed/!s/^(\s*)wire\s+(\w+)_delayed\s*;/&\n\1assign \2_delayed = \2;/' \
    "$VDIR/sky130_fd_sc_hd.v" > "$models.tmp" && mv "$models.tmp" "$models"
fi

# compile + run one simulation; prints the simulator's output
simulate() { # out_vvp sdf_or_empty defines... ; extra sources come via SRC
  local vvp="$1" sdf="$2"; shift 2
  local sdfdef=()
  [ -n "$sdf" ] && sdfdef=("-DSDF_FILE=\"$sdf\"")
  iverilog -g2012 -gspecify -s tb "${sdfdef[@]}" "$@" -o "$vvp" \
    "$VDIR/primitives.v" "$models" "${SRC[@]}" 2> "$vvp.compile.log" \
    || { echo "SIM compile failed: see $vvp.compile.log"; return 1; }
  vvp -n "$vvp" -sdf-warn 2>&1
}

# IOPATH CLK Q rise/fall of instance $2 in SDF $1 (the typ value), as "rise fall"
iopath_clkq() {
  awk -v inst="$2" '
    /\(INSTANCE / { cur = $0; sub(/.*\(INSTANCE */, "", cur); sub(/\).*/, "", cur) }
    cur == inst && /\(IOPATH CLK Q / {
      n = split($0, f, /[():]+/); k = 0
      for (i = 1; i <= n; i++) if (f[i] ~ /^-?[0-9.]+$/) { v[++k] = f[i] }
      print v[2], v[5]; exit }' "$1"
}

# largest |SETUP| or |HOLD| in an SDF: the width of the band where the timer and a simulator that
# enforces no checks can honestly disagree (see the README)
check_band() {
  awk '/\((SETUP|HOLD) / { s = $0; sub(/.*\(/, "", s); split(s, f, ":"); v = f[2] + 0
         if (v < 0) v = -v; if (v > m) m = v } END { printf "%.6f\n", m + 0 }' "$1"
}

absdiff_ok() { awk -v a="$1" -v b="$2" -v t="$TOL" 'BEGIN { d = a - b; if (d < 0) d = -d; exit !(d <= t) }'; }

# ---- one case ----------------------------------------------------------------------------------
run_case() { # design corner [mutation]
  local d="$1" c="$2" mut="${3:-}" kind period sdc launch
  case "$d" in
    pipe|pipe_slow) kind=KIND_PIPE; sdc="$T/pipe.sdc"; launch=u_c0 ;;
    skew_ok|skew_bad) kind=KIND_SKEW; sdc="$T/skew.sdc"; launch=u_a ;;
    *) echo "unknown design $d" >&2; return 2 ;;
  esac
  period="$(awk '/^create_clock/ { for (i = 1; i <= NF; i++) if ($i == "-period") print $(i + 1) }' "$sdc")"
  local dir="$WORK/$d.$c${mut:+.$mut}"
  mkdir -p "$dir"
  printf 'design: %s\nnetlist: %s\nlib: %s\nsdc: %s\ninput_slew: 0.05\noutput_load: 0.005\n' \
    "$d" "$T/$d.v" "$LIBDIR/sky130_fd_sc_hd__$c.lib" "$sdc" > "$dir/job.sta"
  "$STA_BIN" run "$dir/job.sta" --json -o "$dir/sta.json" --sdf "$dir/sta.sdf" > "$dir/sta.log" 2>&1
  [ -s "$dir/sta.json" ] && [ -s "$dir/sta.sdf" ] || { echo "ERROR $d $c: STA wrote no report/SDF ($dir/sta.log)"; return 2; }
  local wns whs band
  wns="$(jq -r .wns_ns "$dir/sta.json")"
  whs="$(jq -r .whs_ns "$dir/sta.json")"
  band="$(check_band "$dir/sta.sdf")"

  # the SDF the simulation gets: the timer's, or a deliberately wrong one
  local simsdf="$dir/sim.sdf"
  case "$mut" in
    "") cp "$dir/sta.sdf" "$simsdf" ;;
    zero) sed -E 's/\(-?[0-9.]+:-?[0-9.]+:-?[0-9.]+\)/(0.000:0.000:0.000)/g' "$dir/sta.sdf" > "$simsdf" ;;
    corner=*)
      local other odir
      other="${mut#corner=}"
      odir="$WORK/$d.$other"
      [ -s "$odir/sta.sdf" ] || run_case "$d" "$other" > /dev/null
      cp "$odir/sta.sdf" "$simsdf" ;;
    zero-launch)
      awk -v inst="$launch" '
        /\(INSTANCE / { cur = $0; sub(/.*\(INSTANCE */, "", cur); sub(/\).*/, "", cur) }
        cur == inst && /\(IOPATH CLK Q / { gsub(/\([0-9.]+:[0-9.]+:[0-9.]+\)/, "(0.000:0.000:0.000)") }
        { print }' "$dir/sta.sdf" > "$simsdf" ;;
    rename) sed -E 's/\(INSTANCE (u_[A-Za-z0-9_]+)\)/(INSTANCE no_such_\1)/' "$dir/sta.sdf" > "$simsdf" ;;
    *) echo "unknown mutation $mut" >&2; return 2 ;;
  esac

  local out
  SRC=("$T/$d.v" "$T/tb.v")
  out="$(simulate "$dir/sim.vvp" "$simsdf" -DDUT="$d" -D"$kind" -DPERIOD="$period")"
  printf '%s\n' "$out" > "$dir/sim.log"
  local line cyc mis lr lf
  line="$(printf '%s\n' "$out" | grep '^SIM cycles=')" || { echo "ERROR $d $c: no SIM result ($dir/sim.log)"; return 2; }
  cyc="$(sed -E 's/.*cycles=([0-9]+).*/\1/' <<<"$line")"
  mis="$(sed -E 's/.*mismatches=([0-9]+).*/\1/' <<<"$line")"
  lr="$(sed -E 's/.*launch_rise=(-?[0-9.]+).*/\1/' <<<"$line")"
  lf="$(sed -E 's/.*launch_fall=(-?[0-9.]+).*/\1/' <<<"$line")"

  # A simulation that never applied the SDF would agree with a timer that passes and prove
  # nothing. The launch flop's clock-to-Q must be the one in the SDF the simulation was given.
  local want
  want="$(iopath_clkq "$simsdf" "$launch")"
  if [ -z "$want" ] || ! absdiff_ok "$lr" "${want% *}" || ! absdiff_ok "$lf" "${want#* }"; then
    echo "NOT-ANNOTATED $d $c${mut:+ [$mut]}: launch clock-to-Q in sim rise=$lr fall=$lf, SDF says ${want:-<no IOPATH for $launch>}"
    return 2
  fi
  [ "$cyc" -gt 0 ] || { echo "ERROR $d $c: zero cycles checked"; return 2; }

  local sta_ok sim_ok inconclusive
  sta_ok="$(awk -v s="$wns" -v h="$whs" 'BEGIN { print (s >= 0 && h >= 0) ? "met" : "VIOLATED" }')"
  [ "$mis" -eq 0 ] && sim_ok=match || sim_ok=MISMATCH
  inconclusive="$(awk -v s="$wns" -v h="$whs" -v b="$band" 'BEGIN { as = s < 0 ? -s : s; ah = h < 0 ? -h : h
                    print (as < b || ah < b) ? 1 : 0 }')"
  printf 'ROW %-9s %-13s wns=%9.4f whs=%9.4f band=%.4f sta=%-8s sim=%-8s (%s/%s cycles wrong)%s\n' \
    "$d" "$c" "$wns" "$whs" "$band" "$sta_ok" "$sim_ok" "$mis" "$cyc" "${mut:+ [$mut]}"
  if [ "$inconclusive" = 1 ]; then
    echo "INCONCLUSIVE $d $c: a slack is inside the +/-$band ns check band, where an STA check and a simulator that enforces no checks may honestly differ; retune the sample"
    return 2
  fi
  if { [ "$sta_ok" = met ] && [ "$sim_ok" = match ]; } || { [ "$sta_ok" = VIOLATED ] && [ "$sim_ok" = MISMATCH ]; }; then
    return 0
  fi
  echo "DISAGREE $d $c${mut:+ [$mut]}: STA $sta_ok (wns $wns, whs $whs) but simulation $sim_ok"
  return 1
}

if [ "${1:-}" = case ]; then
  shift
  run_case "$@"
  exit $?
fi

echo "work: $WORK"
echo "sta:  $STA_BIN"
echo "sim:  $(iverilog -V 2>&1 | head -1)"
fail=0
failmsg() { echo "FAIL: $*"; fail=1; }

# ---- stage 0: prove what the simulator does with an SDF, before relying on it ------------------
P="$WORK/probe"
mkdir -p "$P"
SRC=("$T/probe/probe.v" "$T/probe/tb_probe.v")
pr="$(simulate "$P/probe.vvp" "$T/probe/probe.sdf")"
printf '%s\n' "$pr" > "$P/probe.log"
pl="$(grep '^PROBE ' <<<"$pr")" || { failmsg "probe produced no result ($P/probe.log)"; pl=""; }
pv() { sed -E "s/.* $1=(-?[0-9.]+).*/\1/" <<<"$pl"; }
if [ -n "$pl" ]; then
  absdiff_ok "$(pv n1)" 11.000 && echo "probe: IOPATH on a combinational cell ......... applied (buf rise 1.000 -> 11.000)" \
    || failmsg "probe: combinational IOPATH not applied (n1=$(pv n1), want 11.000)"
  absdiff_ok "$(pv qf)" 33.300 && echo "probe: IOPATH CLK->Q on the flop's edge path ... applied (fall 3.300 -> 33.300)" \
    || failmsg "probe: flop CLK->Q IOPATH not applied (qf=$(pv qf), want 33.300)"
  if absdiff_ok "$(pv d)" 13.200; then
    echo "probe: INTERCONNECT .......................... NOT applied (D at 13.200, not 13.950)"
  elif absdiff_ok "$(pv d)" 13.950; then
    echo "probe: INTERCONNECT .......................... applied (D at 13.950)"
  else
    failmsg "probe: unexpected D arrival $(pv d) (13.200 = IOPATH only, 13.950 = with INTERCONNECT)"
  fi
  if absdiff_ok "$(pv qr)" 56.300 && [ "$(pv qx)" = 0 ] && ! grep -qi 'setup\|timing violation' <<<"$pr"; then
    echo "probe: SETUP/HOLD timing checks ............... NOT enforced (D 0.2 ns before the edge, inside a 0.5 ns setup: captured, no message, no X)"
  else
    echo "probe: SETUP/HOLD timing checks ............... enforced or changed (qr=$(pv qr) qx=$(pv qx)); this gate assumes they are not -- review it"
    fail=1
  fi
fi
# and the reason for the model patch: with the PDK model as shipped, the flop never leaves X
SRCRAW=("$T/probe/probe.v" "$T/probe/tb_probe.v")
if iverilog -g2012 -gspecify -s tb -DSDF_FILE="\"$T/probe/probe.sdf\"" -o "$P/raw.vvp" \
     "$VDIR/primitives.v" "$VDIR/sky130_fd_sc_hd.v" "${SRCRAW[@]}" 2> "$P/raw.compile.log"; then
  rl="$(vvp -n "$P/raw.vvp" 2>&1 | grep '^PROBE ')"
  if [ "$(sed -E 's/.* qf=(-?[0-9.]+).*/\1/' <<<"$rl")" = "-1.000" ]; then
    echo "probe: unpatched PDK flop model ............... Q never leaves X (delayed D/CLK undriven): patched copy used"
  else
    echo "probe: unpatched PDK flop model ............... works ($rl): the delayed-signal patch may no longer be needed"
  fi
fi

# ---- stage 1: the agreement table --------------------------------------------------------------
echo
echo "== timer vs SDF-annotated simulation, per corner =="
declare -A STA
for d in pipe pipe_slow skew_ok skew_bad; do
  for c in $CORNERS; do
    out="$(run_case "$d" "$c")"; rc=$?
    printf '%s\n' "$out"
    row="$(grep '^ROW ' <<<"$out")"
    STA[$d.$c]="$(sed -E 's/.* sta=([A-Za-z]+).*/\1/' <<<"$row")"
    STA[$d.$c.wns]="$(sed -E 's/.* wns= *(-?[0-9.]+).*/\1/' <<<"$row")"
    STA[$d.$c.whs]="$(sed -E 's/.* whs= *(-?[0-9.]+).*/\1/' <<<"$row")"
    [ "$rc" = 0 ] || failmsg "$d $c: timer and simulation do not agree (exit $rc)"
  done
done

# ---- stage 2: the samples do what they are for -------------------------------------------------
# Agreement alone could be met by samples that all pass. These pin the outcomes the samples were
# built to produce, so that the table above contains real failures at the right corners.
echo
echo "== sample intent =="
neg() { awk -v x="$1" 'BEGIN { exit !(x < 0) }'; }
pos() { awk -v x="$1" 'BEGIN { exit !(x >= 0) }'; }
for c in $CORNERS; do
  [ "${STA[pipe.$c]:-}" = met ] || failmsg "pipe $c should meet timing"
  [ "${STA[skew_ok.$c]:-}" = met ] || failmsg "skew_ok $c should meet timing"
  neg "${STA[skew_bad.$c.whs]:-0}" || failmsg "skew_bad $c should fail hold"
done
neg "${STA[pipe_slow.ss_100C_1v60.wns]:-0}" || failmsg "pipe_slow should fail setup at ss"
pos "${STA[pipe_slow.ff_n40C_1v95.wns]:--1}" || failmsg "pipe_slow should meet setup at ff"
[ "$fail" = 0 ] && echo "pipe all met; pipe_slow fails setup at ss, meets it at ff (tt: ${STA[pipe_slow.tt_025C_1v80]:-?}); skew_ok met; skew_bad fails hold at every corner"

# ---- stage 3: teeth -- the agreement check must FAIL on a wrong SDF ----------------------------
echo
echo "== teeth (each must exit nonzero) =="
teeth() { # label expected-verdict design corner mutation
  # Exit code AND reason: a case that crashed also exits nonzero, and would be a tooth that
  # bit for the wrong reason. The verdict line must be the one this mutation should produce.
  local label="$1" want="$2"; shift 2
  out="$("$0" case "$@" 2>&1)"; rc=$?
  if [ "$rc" = 0 ]; then
    failmsg "teeth '$label': the check PASSED on a wrong SDF"
    printf '%s\n' "$out" | sed 's/^/    /'
  elif ! grep -q "^$want " <<<"$out"; then
    failmsg "teeth '$label': exit $rc but not for the expected reason ($want)"
    printf '%s\n' "$out" | sed 's/^/    /'
  else
    echo "ok   '$label' -> exit $rc: $(grep "^$want " <<<"$out" | head -1)"
  fi
}
teeth "delays zeroed" DISAGREE pipe_slow ss_100C_1v60 zero
teeth "wrong corner (ff SDF in the ss check)" DISAGREE pipe_slow ss_100C_1v60 corner=ff_n40C_1v95
teeth "launch clock-to-Q zeroed (hold side)" DISAGREE skew_ok tt_025C_1v80 zero-launch
teeth "SDF does not annotate (instance names wrong)" NOT-ANNOTATED pipe ss_100C_1v60 rename

echo
if [ "$fail" = 0 ]; then
  echo "PASS: sdf-sim gate (timer and SDF-annotated simulation agree at every corner; teeth bite)"
  exit 0
fi
echo "FAIL: sdf-sim gate (see above; work dir $WORK)"
exit 1
