#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Correlate the engine against a signed-off routed sky130 block.
#
# The ordinary test suite compares the engine against itself: fixtures assert numbers the engine
# produced. That cannot catch a behaviour we never implemented, and every real defect found so
# far was found by comparing against an artifact from OUTSIDE — a sign-off timer's own reports on
# a real design with a real PDK and real parasitics. This script is that comparison, made
# repeatable.
#
# It is deliberately NOT part of `cargo test`: it needs a PDK and a design that are too large to
# vendor, so it is a separate, on-demand check. Run it locally before a release, and let CI run
# it on a schedule so drift surfaces even when nobody remembers to look.
#
#   ./scripts/correlate-sky130.sh
#
# Inputs are found automatically for a normal local checkout, and can be overridden:
#   PDK_LIB   path to sky130_fd_sc_hd__tt_025C_1v80.lib
#   DESIGN    path to a checkout of the vyges-edge-sensor-soc design repo
#   STA_BIN   path to the vyges-sta-si binary (default: build it)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- the reference numbers -------------------------------------------------------------------
# Read from the design's checked-in OpenLane/OpenSTA sign-off reports where they are available.
# A frozen artifact, not a re-run, which is what makes it a stable baseline — and reading it
# beats transcribing it, because a transcribed number cannot tell you when it went stale.
# `report_checks` sorts worst-first, so the first slack and the first endpoint are the ones.
#
# The literals below are the values those reports currently hold. They are the fallback for a
# checkout without the sign-off directory, and they double as documentation of what to expect.
SIGNOFF_WNS=5.6834
SIGNOFF_WHS=3.1359
SIGNOFF_WORST_SETUP_ENDPOINT=sram_clk_o
CLOCK_PERIOD=25.0

# Bounds, not equalities. An engine improvement moves our numbers CLOSER to sign-off and must
# pass; only divergence should fail. Set with headroom over the current agreement (0.22 % of
# period on setup, 0.40 ns on hold) so ordinary noise does not cause a false alarm, but a lost
# check category — which is what the endpoint assertion below really guards — cannot hide.
MAX_SETUP_DIFF_PCT_OF_PERIOD=0.5
MAX_HOLD_DIFF_NS=0.6

# ---- locate the inputs -----------------------------------------------------------------------
PDK_LIB="${PDK_LIB:-$HOME/.ciel/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib}"
DESIGN="${DESIGN:-$here/../vyges-edge-sensor-soc}"

fail=0
for f in "$PDK_LIB" \
         "$DESIGN/verilog/gl/fft_ctrl_tlul.v" \
         "$DESIGN/spef/fft_ctrl_tlul.nom.spef"; do
  [ -f "$f" ] || { echo "missing input: $f" >&2; fail=1; }
done
if [ "$fail" = 1 ]; then
  cat >&2 <<'EOM'

This check needs a PDK Liberty and the design, neither of which is vendored here.
  PDK_LIB=... DESIGN=... ./scripts/correlate-sky130.sh
The design is public: https://github.com/vyges/vyges-edge-sensor-soc
The PDK is mirrored at:  https://github.com/vyges-tools/pdk-releases  (sky130_fd_sc_hd.tar.zst)
EOM
  exit 2
fi

RPT="$DESIGN/signoff/fft_ctrl_tlul/openlane-signoff/timing-reports/nom_tt_025C_1v80"
if [ -f "$RPT/max.rpt" ] && [ -f "$RPT/min.rpt" ]; then
  SIGNOFF_WNS=$(awk '/slack/ { print $1; exit }' "$RPT/max.rpt")
  SIGNOFF_WHS=$(awk '/slack/ { print $1; exit }' "$RPT/min.rpt")
  SIGNOFF_WORST_SETUP_ENDPOINT=$(awk '/^Endpoint:/ { print $2; exit }' "$RPT/max.rpt")
  baseline="read from the sign-off reports"
else
  baseline="built-in fallback (sign-off reports not in this checkout)"
fi
echo "baseline: $baseline — WNS $SIGNOFF_WNS, WHS $SIGNOFF_WHS, worst $SIGNOFF_WORST_SETUP_ENDPOINT"

STA_BIN="${STA_BIN:-}"
if [ -z "$STA_BIN" ]; then
  ( cd "$here" && cargo build --release --quiet )
  STA_BIN="$here/target/release/vyges-sta-si"
fi

# ---- the job ---------------------------------------------------------------------------------
# Constraints reconstructed from the design's resolved LibreLane knobs; see the header comments
# in the internal correlation notes for why each value is what it is.
job="$(mktemp -t correlate-sky130-XXXXXX).sta"
cat > "$job" <<EOM
design:      fft_ctrl_tlul
netlist:     $DESIGN/verilog/gl/fft_ctrl_tlul.v
lib:         $PDK_LIB
spef:        $DESIGN/spef/fft_ctrl_tlul.nom.spef
clock:       clk_i 25.0
input_delay:  5.0
output_delay: 5.0
setup_uncertainty: 0.25
hold_uncertainty:  0.25
late_derate:  1.05
early_derate: 0.95
input_slew:  0.15
output_load: 0.033442
EOM

echo "== running the engine on a routed sky130 block =="
# stderr as well as stdout: the report goes to stdout, and the input-coverage events — which
# say whether the SPEF and the netlist agree about names — go to stderr. Discarding it is how
# the join defect below stayed invisible to this script.
errf="$(mktemp -t correlate-err-XXXXXX)"
trap 'rm -f "$job" "$errf"' EXIT
out="$("$STA_BIN" run "$job" 2>"$errf")"
err="$(cat "$errf")"
echo "$out" | grep -E "endpoints:|WNS:|WHS:|worst path to|worst hold path to"

# awk rather than sed: BSD and GNU sed disagree on BRE escapes (`\?`), and a parser that only
# works on the maintainer's laptop is how a CI-only failure gets written.
field_after() { awk -v k="$1" '{for (i = 1; i <= NF; i++) if ($i == k) { print $(i + 1); exit }}'; }
wns=$(echo "$out" | field_after "WNS:")
whs=$(echo "$out" | field_after "WHS:")
worst=$(echo "$out" | awk '/worst path to/ { print $4; exit }')

[ -n "$wns" ] && [ -n "$whs" ] && [ -n "$worst" ] || {
  echo "FAIL: could not parse the report — the output format changed" >&2; exit 1; }

# ---- compare ---------------------------------------------------------------------------------
echo
echo "== against sign-off =="
rc=0
report() { printf '  %-34s %-12s %-12s %s\n' "$1" "$2" "$3" "$4"; }
report "check" "ours" "sign-off" "verdict"

# The identity of the critical path matters more than the slack on it: reporting a DIFFERENT
# worst endpoint than sign-off means a whole category of path is missing, which is exactly the
# defect this block exposed once already (clock-source -> output-port paths were unreported, so
# WNS belonged to a path that was not the critical one).
if [ "$worst" = "$SIGNOFF_WORST_SETUP_ENDPOINT" ]; then
  report "worst setup endpoint" "$worst" "$SIGNOFF_WORST_SETUP_ENDPOINT" "OK"
else
  report "worst setup endpoint" "$worst" "$SIGNOFF_WORST_SETUP_ENDPOINT" "FAIL"
  echo "    -> a different critical path than sign-off means a missing check/path category" >&2
  rc=1
fi

verdict() { # value signoff bound label unit
  awk -v a="$1" -v b="$2" -v lim="$3" -v label="$4" -v u="$5" 'BEGIN{
    d = a - b; if (d < 0) d = -d;
    printf "  %-34s %-12.4f %-12.4f %s (|d| %.4f %s, bound %.4f)\n",
           label, a, b, (d <= lim ? "OK" : "FAIL"), d, u, lim;
    exit (d <= lim ? 0 : 1) }'
}

setup_bound=$(awk -v p="$CLOCK_PERIOD" -v pct="$MAX_SETUP_DIFF_PCT_OF_PERIOD" 'BEGIN{print p*pct/100}')
verdict "$wns" "$SIGNOFF_WNS" "$setup_bound" "setup WNS" "ns" || rc=1
verdict "$whs" "$SIGNOFF_WHS" "$MAX_HOLD_DIFF_NS" "hold WHS" "ns" || rc=1

# ---- did the inputs actually reach the engine? -----------------------------------------------
#
# Two slack numbers within bounds is a weaker statement than it looks. This check passed for
# weeks while **767 of the design's 14238 nets carried no parasitics at all** and 4527 coupling
# references were looked up, missed and dropped — because the netlist reader kept the leading
# backslash of a Verilog escaped identifier and the SPEF did not, so the two files never agreed
# on those names. The affected nets were timed as ideal wire; WNS moved by less than the bound
# and nothing said a word.
#
# So gate on the join as well as on the numbers. The engine reports it (`STA-SPEF`); a
# disagreement about NAMES is unambiguous — a parasitic described for a net nothing in the
# design is called means the two files are not talking about the same circuit. Coverage
# PERCENTAGE is deliberately not asserted here: a design legitimately has nets a SPEF omits, so
# any threshold on it is a guess, and 94.6 % is what the defect above looked like.
spef_note=$(echo "$err" | grep -o 'SPEF [^"]*' | head -1)
if echo "$spef_note" | grep -q "disagree about names"; then
  report "spef/netlist names" "disagree" "correspond" "FAIL"
  echo "    -> $spef_note" >&2
  echo "    -> those nets are timed as ideal wire; the slack numbers above are optimistic" >&2
  rc=1
else
  report "spef/netlist names" "correspond" "correspond" "OK"
fi

echo
if [ "$rc" = 0 ]; then
  echo "PASS — the engine still agrees with sign-off on this block."
else
  echo "FAIL — correlation with sign-off has regressed." >&2
  echo "This is not a flaky test. Something the engine reports about a real design changed," >&2
  echo "and the sign-off side is a frozen artifact, so the change is ours." >&2
fi
exit $rc
