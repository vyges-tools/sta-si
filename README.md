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

**Describe the job, not the script.** Every incumbent here — PrimeTime, Tempus,
OpenSTA — is driven by hand-written **Tcl**, a recurring source of silent typos,
copy-paste drift across corners and blocks, and brittle maintenance. `vyges-sta-si`
takes a small **declarative job file** (`.sta`) instead: readable, diffable,
schema-checkable, with no control flow to get wrong. And the one constraint artifact
people *do* author — the **SDC** — is read directly (`sdc:`), not re-scripted. This
is a toolchain-wide property: char, extract, and em-ir are configured the same way.

**Validate fast, sign off with your tool.** `vyges-sta-si` reads the **standard
formats** (Verilog, Liberty, SPEF, SDC), so you iterate timing in the fast Vyges loop
and hand the *same files* to OpenSTA or PrimeTime for final sign-off — no flow change,
just a different timer on identical inputs. That interop is demonstrated, not promised:
on a real routed block, sta-si and OpenSTA agree on WNS within **0.5%** from the same
library/SPEF/SDC. Adopt it for the fast inner loop where licenses are scarce and runs
are slow; keep your sign-off tool for tape-out.

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
**POCV** is statistical: each cell stage carries a 1-sigma delay, the variances sum
along the path, and the reported delay carries an N-sigma band — so pessimism grows
as **√depth** (RSS), not linearly. The per-stage sigma comes from **LVF**
(`ocv_sigma_cell_rise/fall`, slew·load-dependent) when the library provides it —
which auto-enables POCV — otherwise from the global `pocv_sigma · delay` fraction.
POCV wins when LVF is present or `pocv_sigma > 0`, else AOCV when a table is present,
else flat.

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
accuracy — the **`.spef`** from `vyges-extract`. What it gives you is the
**answer to "does it meet timing, and if not, where?"** — the worst path tells
you the exact gates and arrival times.

### Where it sits vs OpenSTA / PrimeTime — run it *first*, not *instead*

`vyges-sta-si` is an **early-flow and complementary** engine, **not a tapeout
sign-off replacement** for OpenSTA or PrimeTime. It is *correlated to* OpenSTA
(within ~0.6 % of WNS on a routed sky130 block), i.e. one tier below it in
maturity — so it runs **upstream of**, and **alongside**, the signoff tool, never
in lieu of it for tape-out:

| Stage | Run | Why |
| --- | --- | --- |
| RTL / synth / P&R **iteration** | **`vyges-sta-si`** as the fast inner-loop + CI gate | std-only binary, no Tcl, `--fail-on-violation` exit 3 — catch timing breaks in seconds before spinning a full signoff run |
| **Pre-signoff** (open flow) | `vyges-sta-si` **then OpenSTA** | OpenSTA is the open signoff authority; sta-si adds the **SI / CRPR / AOCV-POCV(LVF) / multi-clock / exceptions** margins that base OpenSTA-in-LibreLane doesn't, as a second opinion |
| **Tape-out signoff** (if licensed) | `vyges-sta-si` early, **PrimeTime** for signoff | PrimeTime is the authority; sta-si stays the fast iteration loop + SI/crosstalk cross-check. Don't replace PT with it for the mask set |

So the rule of thumb: **run `vyges-sta-si` first and often** (iteration + regression
gate + SI/CRPR insight), then hand off to **OpenSTA** (open signoff) or **PrimeTime**
(licensed signoff) for the authoritative final numbers. Its unique value even when
you have PT is the fast, license-free loop and the bundled SI+CRPR+OCV view. (See
[`docs/primetime-comparison.md`](docs/primetime-comparison.md) for the honest gap.)

### What to capture, and how to use it downstream

Run with `--json` for machine-readable output. Capture:

- **`wns_ns` / `tns_ns`** (setup) and **`whs_ns` / `ths_ns`** (hold) + the **`met`**
  verdict — the slack numbers and pass/fail.
- **`worst_endpoint` + `worst_path`** (and the hold path) — the launch/capture pins
  and per-node arrival/slew: **where** the problem is.
- **`pba_wns_ns`** (if `pba: true`) — flags a non-greedy worst path the graph-based
  number can miss.
- For **MCMM**, the worst setup/hold **and the binding corner** per check.

How those feed the next step:

1. **Gate the loop** — `--fail-on-violation` (exit 3) in CI stops a broken design
   before it ever reaches OpenSTA/PT, saving the slow run.
2. **Fix, then re-run** — the worst path's gates are the ECO target: resize/buffer,
   re-place, retime, or reconstrain the clock; iterate on sta-si until it meets.
3. **Cross-check at signoff** — compare `wns_ns` to OpenSTA/PT. Agreement within the
   correlation band ⇒ confidence; a **gap ⇒ the SI/CRPR/OCV delta sta-si adds** —
   crosstalk or reconvergence risk to inspect in the signoff tool.
4. **Hand off the same inputs** — netlist + `.lib` + `.spef` + SDC are unchanged, so
   the signoff tool runs on identical data; sta-si's worst-path report tells the
   signoff engineer which paths to scrutinise first.

In the open flow it occupies the slot where OpenSTA runs inside LibreLane.

## Use it

```sh
# build it yourself (std-only, no deps) -- or grab a binary from GitHub Releases:
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
#false_path:  uart_rx  cfg_reg        # exclude a path (from to; * = any)
#multicycle:  mac_a    mac_acc  3     # N-cycle path (from to cycles)
miller:      2.0            # crosstalk Miller factor (1.0 disables SI)
xtalk_window: 0.0           # ns — guard band added to the slew-derived window
input_slew:  0.02           # ns
output_load: 0.005          # pF at primary outputs
late_derate: 1.0            # flat OCV late derate on cell delays (setup / max path)
early_derate: 1.0           # flat OCV early derate on cell delays (hold / min path)
# advanced OCV — pick ONE refinement over the flat derates above:
aocv_late:  1:1.10, 8:1.02  # AOCV: late derate vs path depth (interpolated)
aocv_early: 1:0.90, 8:0.98  # AOCV: early derate vs path depth
pocv_sigma: 0.05            # POCV: per-stage 1-sigma fraction (LVF lib tables, if any, override this)
pocv_n:     3.0             # POCV: number of sigmas for the bound (default 3.0)
#pba: true                  # path-based analysis: re-time critical paths (default false)
```

### Constraints from SDC

Real flows (synthesis, OpenROAD/LibreLane) emit their timing intent as **SDC**.
Point the job at one with `sdc:` and the constraints are read straight from it —
the netlist, libraries, and parasitics still come from the job (they are not in
SDC). The SDC is **authoritative** for what it sets; explicit `.sta` values fill
anything it leaves unspecified.

```text
design:  top
netlist: top.v
lib:     cells.lib
spef:    top.spef
sdc:     top.sdc        # clocks, I/O delays, uncertainty, derates, exceptions
```

Supported SDC (a Tcl-subset reader — `set var`, `$var`, `[get_ports …]`,
`[all_inputs]`, `{…}` lists, `set_units` scaling, `\`-continuations):

| command | effect |
|---|---|
| `create_clock` / `create_generated_clock` | clock(s); a generated clock's period is resolved from its master × `divide_by` / `multiply_by` |
| `set_input_delay` / `set_output_delay` | I/O timing budget — default (`all_inputs`/`all_outputs`) plus per-port overrides; seeds input arrival / eats the period at outputs |
| `set_clock_uncertainty [-setup|-hold]` | guard band — tightens setup required, relaxes hold required |
| `set_clock_latency` | source/network latency, applied to the I/O budget |
| `set_input_transition` / `set_load` | boundary slew / load |
| `set_timing_derate -late|-early` | flat OCV derate |
| `set_false_path` / `set_multicycle_path` | timing exceptions (`-from`/`-to`, pin → instance) |

Anything not modelled (`set_driving_cell`, `set_max_fanout`, …) is **never
silently dropped** — `run -v` lists every ignored command so you know exactly
what was and wasn't applied.

For **MCMM**, a job instead lists scenario files and the engine reports the worst
setup/hold across them:

```text
design:    top
scenarios: corner_ss.sta, corner_tt.sta, corner_ff.sta   # each a full single-corner .sta
```

A complete, runnable example is in [`examples/top/`](examples/top/);
`vyges-sta-si run examples/top/top.sta` reports the slack on a 3-inverter chain.
`run examples/top/top_sdc.sta -v` runs the same design with its clock, I/O delays,
uncertainty, and derate read from [`top.sdc`](examples/top/top.sdc) instead.
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
edge relation, not a single period), **timing exceptions** (false paths and
multicycle paths, matched on launch/capture instance or port), and **CCS-into-RC
delay** — a current-source model (`output_current` waveforms) plus an **effective
capacitance**: the driver behind a resistive net sees C1 + shielded-C2, not the
lumped total (Ceff iterated to convergence with the output slew), so cell delay
drops on resistive nets (this benefits NLDM too, not just CCS). The interconnect
delay to each sink is a **transient waveform-into-RC solve** (backward-Euler on the
RC tree driven by the driver edge) — the true response, e.g. 0.69·RC for a single
RC, not Elmore's pessimistic R·C — and the **degraded sink slew** it computes is
propagated downstream (a resistive net hands the next stage a slower edge, raising
its delay). With `pba: true` it adds **path-based analysis** — re-timing the
critical path and its fan-in alternatives with strictly path-local slews, catching
a non-greedy worst path that the graph-based max can miss. Fully offline, no external
deps, 50 tests green.
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
conservative — the residual is second-order slew propagation. On a real routed sky130 block (post-route netlist + OpenRCX SPEF) the reg→reg setup slack matches OpenSTA within ~0.6% of the clock period.

When a pin carries a CCS **receiver_capacitance** model (emitted by `vyges-char`),
the driver is loaded with the Miller-aware effective input cap (the C1/C2 segments)
rather than the static `capacitance` — a small, correct-direction increase in net
load and delay. v1 uses a representative scalar; full slew/load-resolved receiver
load is future.

The road to sign-off grade builds on the same graph: slew/load-resolved receiver
load and widening PBA from 1-exchange to k-worst enumeration. See
[`docs/primetime-comparison.md`](docs/primetime-comparison.md) for an honest
feature-by-feature comparison to Synopsys PrimeTime and where this engine can and
can't reach it. The SI margin it adds over OpenSTA stays.
