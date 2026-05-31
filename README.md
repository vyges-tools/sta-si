# vyges-sta-si

Sign-off **static timing analysis with signal integrity**: a gate-level netlist
+ timing libraries + a clock in, a slack report out.

> **Vyges open EDA tools.** Commercial-grade silicon sign-off capability, built
> on open standards and plain file formats — and meant to be accessible to
> everyone, not only teams who can license a six-figure tool. `vyges-sta-si`
> opens up timing sign-off.

## Why this exists

A design is only correct if it meets timing: every path from launch to capture
must settle within the clock period, accounting for cell delays, wire delays,
and on-chip variation. Static timing analysis proves that across all paths at
once — and at 28 nm and below, crosstalk between neighbouring wires (signal
integrity) starts to move those numbers enough that ignoring it is not sign-off.

## How this is solved today

In production, timing sign-off is Synopsys **PrimeTime / PrimeTime-SI** or
Cadence **Tempus** — crosstalk delay/noise, statistical/AOCV-POCV derating,
multi-corner multi-mode — gated behind major licenses. The open baseline is
**OpenSTA** (used inside OpenROAD/LibreLane); solid for delay, but SI/crosstalk
and advanced OCV are where it stops short of advanced-node sign-off.
`vyges-sta-si` is an open engine in that space, behind the standard file formats
(Verilog, Liberty, SPEF, SDC), and correlated against OpenSTA as its baseline.

## The problem it solves

Given a **gate-level netlist** (`*.v`), one or more **Liberty** libraries
(`*.lib`), a **clock**, and *(optional)* **SPEF** parasitics (`*.spef`), it
builds a timing graph — cell arcs (delay from the NLDM tables, interpolated on
input slew × output load) and net arcs (the SPEF interconnect delay) — propagates
arrival and required times, and reports **WNS** (worst negative slack), **TNS**
(total negative slack), and the **worst path** with per-node arrival and slew.
With SPEF, the wire capacitance loads the driver and a **per-pin tree Elmore**
net delay is computed to each sink (delay = Σ over the driver→sink path of
`R · downstream-cap`), so different sinks see different interconnect delays;
without SPEF the interconnect is ideal (a lumped `R·C` is the fallback when the
SPEF has no usable tree).
**Coupling** capacitance in the SPEF adds a **crosstalk delta-delay** to victim
nets — the Miller-amplified coupling `R·(MCF−1)·Cc` — but **only from aggressors
whose switching window overlaps the victim's**. Windows are **slew-derived** (each
net's transition is an interval of width = its slew, so they overlap when
`|Δsw| ≤ (slew_v + slew_a)/2`), so sequentially-switching neighbours don't pile
on false pessimism. A late OCV derate is applied to cell delays.
It checks **setup** (max-delay) *and* **hold** (min-delay): the hold pass is a
second forward propagation using min-corner cell delays (and an early OCV derate),
and for each flop the earliest data arrival must clear that pin's hold constraint —
reported as **WHS** / **THS** alongside WNS / TNS.
On-chip variation has three modes. **Flat** (default) applies the scalar late/early
derates to every stage. **AOCV** takes a *depth-dependent* derate table — shallow
paths derate hard, deep paths relax toward 1.0 as variation averages out.
**POCV** is statistical: each cell stage carries a 1-sigma `pocv_sigma · delay`, the
variances sum along the path, and the reported delay carries an N-sigma band — so
pessimism grows as **√depth** (RSS), not linearly. POCV wins when `pocv_sigma > 0`,
else AOCV when a table is present, else flat.

## When & how to use it in your flow

```text
  RTL  ─[Yosys]─► netlist ─[OpenROAD: place+route]─► layout
                    │                                  │
                    │                                  └─[vyges-extract]─► *.spef
   *.lib (from the PDK, or vyges-char) ──┐                                   │
                                         ▼                                   ▼
                              ┌──────────────────────────────────────────────┐
                              │  vyges-sta-si  (netlist + .lib [+ .spef] +clk) │
                              └──────────────────────────────────────────────┘
                                         │
                                         ▼
                          WNS / TNS / worst path  ──►  meet timing? sign off :
                                                       fix critical path / retime / reconstrain
```

You run it **after synthesis and place-and-route** (you need a gate-level
netlist), with the **`.lib`** from your PDK or `vyges-char` and — for
accuracy — the **`.spef`** from `vyges-extract`. You run it **before
tape-out**, every time you change RTL, floorplan, or constraints. What it gives
you is the **answer to "does it meet timing, and if not, where?"** — the worst
path tells you the exact gates and arrival times, so you decide whether to sign
off, fix the critical path, or change the clock. In the open flow it occupies
the slot where OpenSTA runs inside LibreLane.

## Use it

```sh
# prebuilt binaries: dist/<triple>/vyges-sta-si  (or build it yourself:)
cargo build --release            # std-only, no external deps

vyges-sta-si run  top.sta -o top.rpt           # analyze -> timing report
vyges-sta-si run  top.sta --json               # machine-readable WNS/TNS/path
vyges-sta-si run  top.sta --fail-on-violation  # exit 3 if WNS < 0 (CI gate)
vyges-sta-si check top.sta                     # validate the job + inputs
vyges-sta-si demo                              # analyze a built-in 2-gate design
# common flags: -o FILE · --json · -q/--quiet · -v/--verbose · -h/--help · -V/--version
```

A job (`*.sta`) is a few `key: value` lines:

```text
design:      top
netlist:     top.v          # gate-level structural Verilog
lib:         cells.lib      # one or more, comma-separated
spef:        top.spef       # optional parasitics -> wire load + net delay
clock:       clk 1.0        # clock port + period (ns); repeat for multiple clocks:
#clock:      spiclk spi_clk 4.0       # name source period (source: port or inst/pin)
#clock:      divclk u_div/Q 2.0       # generated/divided clock off an internal pin
miller:      2.0            # crosstalk Miller factor (1.0 disables SI)
xtalk_window: 0.0           # ns — guard band added to the slew-derived window
input_slew:  0.02           # ns
output_load: 0.005          # pF at primary outputs
late_derate: 1.0            # flat OCV late derate on cell delays (setup / max path)
early_derate: 1.0           # flat OCV early derate on cell delays (hold / min path)
# advanced OCV — pick ONE refinement over the flat derates above:
aocv_late:  1:1.10, 8:1.02  # AOCV: late derate vs path depth (interpolated)
aocv_early: 1:0.90, 8:0.98  # AOCV: early derate vs path depth
pocv_sigma: 0.05            # POCV: per-stage 1-sigma as a fraction of stage delay
pocv_n:     3.0             # POCV: number of sigmas for the bound (default 3.0)
```

For **MCMM**, a job instead lists scenario files and the engine reports the worst
setup/hold across them:

```text
design:    top
scenarios: corner_ss.sta, corner_tt.sta, corner_ff.sta   # each a full single-corner .sta
```

A complete, runnable example is in [`examples/top/`](examples/top/);
`vyges-sta-si run examples/top/top.sta` reports the slack on a 3-inverter chain.
See [`examples/icsprout55/`](examples/icsprout55/) for a 55nm reg-to-reg path with
flat / POCV / multi-corner (`mcmm.sta`) runs.

## Open core, certified fab plugins

`vyges-sta-si` is open and contains **no foundry-confidential data**. The bulk of
the silicon correlation it relies on arrives *in the inputs* — the `.lib`
(from a `vyges-char` plugin) and `.spef` (from a `vyges-extract` plugin). What is
fab-specific to STA itself — the node's OCV/AOCV derate factors, sign-off margins,
and SI calibration — is delivered as a **separate, per-foundry plugin** under
that foundry's NDA, never in this repository.

```text
  vyges-sta-si — OPEN engine  (Apache-2.0, contains no fab data)
  ────────────────────────────────────────────────────────────────────
    netlist + .lib [+ .spef] + clock  ─►  timing graph  ─►  WNS / TNS / path
                                              ▲
                                              └─ published plugin contract
                                                 (derate · margins · SI calibration)
                                       │
        ┌──────────────────────────────┴──────────────────────────────┐
  OPEN reference plugin                          CERTIFIED per-fab plugins
  (in-repo · no NDA)                             (private · one per fab/node 🔒)
    • unit derates; worst-case SI                  • vyges-sta-si-tsmc28
      ✓ runs out of the box                        • vyges-sta-si-sec28
                                                   AOCV/POCV + SI margins, under NDA
```

## Current state (2026-05-31)

v1 does **setup *and* hold** timing. Setup is the max-delay path — combinational
(input → output) **and register-to-register** (flop Q launches via its CK→Q arc;
flop D pins are capture endpoints with required = period − setup). Hold is a second,
min-delay forward pass (min-corner cell delays + an early OCV derate) where the
earliest data arrival at each flop D must clear that pin's hold constraint, reported
as **WHS / THS**. On top of that: NLDM cell delays interpolated on slew × load, a
late OCV derate, **SPEF-driven interconnect** (wire-cap load + **per-pin tree
Elmore** net delay), and **crosstalk delta-delay with slew-derived switching
windows, iterated to convergence** (arrivals set the windows, the windows set the
coupling, repeat until the per-arc delays stabilise), **AOCV / POCV** on-chip
variation (depth-dependent derate table, or a statistical √depth N-sigma band),
**clock-network skew** (the clock is timed like any path; each capture flop's
insertion delay enters its required time, so common latency cancels and only skew
moves slack), **CRPR** (launch and capture take opposite clock corners — late/early
for setup, early/late for hold — and the OCV spread on the clock path they *share*
is credited back, removing reconvergence pessimism; `crpr: false` to disable), and
**MCMM** (a job can list per-corner scenario `.sta` files; the worst setup and worst
hold are reported across them), **rise/fall-split unate propagation**, and
**multi-clock / generated clocks** (cross-domain paths use the tightest launch→capture
edge relation, not a single period). Fully offline, no external deps, 34 tests green.
It **closes the loop with the other engines**: it reads the
Liberty `vyges-char` emits and the SPEF (incl. coupling + RC tree) `vyges-extract`
emits — the SI margin OpenSTA lacks.

Cell delays *and* setup/hold constraints are bilinear NLDM interpolations at the
operating slews (not table maxima) — the constraint methodology matches OpenSTA.

**Validated on real PDKs:** sky130, gf180, ihp-sg13g2, and **icsprout55 (55nm — our
first sub-100nm node)**, whose reg-to-reg setup/hold/POCV example is in
[`examples/icsprout55/`](examples/icsprout55/) and pinned in the test suite.

Propagation is **rise/fall-split by arc unateness** — an inverter chain alternates
edges rather than taking `max(rise,fall)` per stage, matching how real paths behave.

**Correlated against OpenSTA 2.7.0** on a sky130 design: single-arc paths match to
4 decimals (global WNS 9.3760 ns, DFF CK→Q 0.6240 ns), and a multi-stage reg→reg
path agrees within **~3%** (down from ~7% before unate-split), staying slightly
conservative — the residual is second-order slew propagation.

The road to sign-off grade builds on the same graph: clock phases / waveforms and
false-path & multicycle exceptions, plus slew-propagation refinement. The SI margin
it adds over OpenSTA stays.
