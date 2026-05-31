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
With SPEF, the wire capacitance loads the driver and a lumped Elmore (R·C) net
delay is added to each driver→sink arc; without it the interconnect is ideal. A
late OCV derate is applied to cell delays.

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
clock:       clk 1.0        # clock port + period (ns)
input_slew:  0.02           # ns
output_load: 0.005          # pF at primary outputs
late_derate: 1.0            # OCV late derate on cell delays
```

A complete, runnable example is in [`examples/top/`](examples/top/);
`vyges-sta-si run examples/top/top.sta` reports the slack on a 3-inverter chain.

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
    • unit derates (1.0), no SI                    • vyges-sta-si-tsmc28
      ✓ runs out of the box                        • vyges-sta-si-sec28
                                                   AOCV/POCV + SI margins, under NDA
```

## Current state (2026-05-31)

v1 does **combinational max-delay** timing (primary input → primary output) with
NLDM cell delays interpolated on slew × load, a late OCV derate, and **SPEF-driven
interconnect** — the wire cap loads the driver and a lumped Elmore (R·C) net delay
is added per arc. Fully offline, no external deps, 12 tests green. It **closes the
loop with the other engines**: it reads the Liberty `vyges-char` emits and the
SPEF `vyges-extract` emits.

The road to sign-off grade builds on the same graph: per-pin (path) Elmore from
the full SPEF RC tree (v1 lumps R·C), register setup/hold (sequential) timing,
AOCV/POCV statistical derating, and crosstalk delta-delay (the SI layer — the
engine reserves the `StaError::SiNotModeled` hook). Correlation target: match
OpenSTA on a routed block, then add the SI margin OpenSTA lacks.
