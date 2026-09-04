# vyges-sta-si

Sign-off **static timing analysis with signal integrity**: a gate-level netlist,
timing libraries, and a clock in — a slack report out.

> **Vyges open EDA tools.** Commercial-grade silicon sign-off capability, built
> on open standards and plain file formats — and meant to be accessible to
> everyone, not only teams who can license a six-figure tool. `vyges-sta-si`
> opens up timing sign-off.

**Docs:** [docs.vyges.com](https://docs.vyges.com) — this engine's chapter, the
[cross-engine integration guide](https://docs.vyges.com/engines/integration.html) (how the four
Vyges engines work together and where each plugs into an OpenROAD / LibreLane flow), and the
job-file formats. In-repo depth: [`docs/engines-integration.md`](docs/engines-integration.md),
[`docs/opensta-integration.md`](docs/opensta-integration.md),
[`integrations/librelane/`](integrations/librelane/). **Integrating at the binary level and need
help?** → <https://vyges.com/contact>.

## Why this exists

A design is only correct if it meets timing: every path from launch to capture
must settle within the clock period, accounting for cell delays, wire delays,
and on-chip variation. Static timing analysis proves that across all paths at
once — and at 28 nm and below, crosstalk between neighbouring wires (signal
integrity) starts to move those numbers enough that ignoring it is not sign-off.

## How this is solved today

In production, timing sign-off is **the commercial sign-off timer** — crosstalk
delay/noise, statistical/AOCV-POCV derating,
multi-corner multi-mode — gated behind major licenses. The open baseline is
**OpenSTA** (used inside OpenROAD/LibreLane); solid for delay, but SI/crosstalk
and advanced OCV are where it stops short of advanced-node sign-off.
`vyges-sta-si` is an open engine in that space, behind the standard file formats
(Verilog, Liberty, SPEF, SDC), and correlated against OpenSTA as its baseline.

**Describe the job, not the script.** Every incumbent here — the commercial sign-off timer,
OpenSTA — is driven by hand-written **Tcl**, a recurring source of silent typos,
copy-paste drift across corners and blocks, and brittle maintenance. `vyges-sta-si`
takes a small **declarative job file** (`.sta`) instead: readable, diffable,
schema-checkable, with no control flow to get wrong. And the one constraint artifact
people *do* author — the **SDC** — is read directly (`sdc:`), not re-scripted. This
is a toolchain-wide property: char, extract, and em-ir are configured the same way.

**Validate fast, sign off with your tool.** `vyges-sta-si` reads the **standard
formats** (Verilog, Liberty, SPEF, SDC), so you iterate timing in the fast Vyges loop
and hand the *same files* to OpenSTA or the commercial sign-off timer for final sign-off — no flow change,
just a different timer on identical inputs. That interop is demonstrated, not promised:
on a real routed block, sta-si and OpenSTA agree on WNS within **~1 %** — on the same critical
path — from the same library/SPEF/SDC. Adopt it for the fast inner loop where licenses are scarce and runs
are slow; keep your sign-off tool for tape-out.

**Built on a shared foundation, not a private front end.** The readers — Verilog and **Yosys
JSON** netlists, Liberty (NLDM + CCS), SDC, SPEF — are not part of this timer. They live in
[**`vyges-loom`**](https://github.com/vyges-tools/loom), the data plane every Vyges engine sits
on: parse once, query many. So the netlist you time is the same object `vyges-power`,
`vyges-extract` and `vyges-lvs` see, and a format the foundation learns is a format every engine
gains — Yosys JSON support reached this timer as a few lines, not a port. It is also why adding
a reader never drags a whole application in behind it (see
[`docs/opensta-integration.md`](docs/opensta-integration.md)).

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

### Capability at a glance

The axes usually cited when comparing an open timer against a commercial sign-off tool, and
where this engine actually stands on each. Two of them are the reason it exists; the others are
stated as gaps rather than papered over.

| Axis | `vyges-sta-si` | What that means concretely |
| --- | --- | --- |
| **Statistical STA / on-chip variation** | **yes** | Flat derate, **AOCV** (depth-dependent derate table), and **POCV** — a per-stage sigma with an N-sigma band growing as √depth (RSS), not linearly. Auto-enables from Liberty **LVF** `ocv_sigma_*` tables when the library carries them. **CRPR** credits back shared launch/capture clock pessimism. |
| **SI and crosstalk analysis** | **yes** | Coupling capacitance from SPEF with a Miller multiplier, filtered by **switching-window overlap** and iterated to convergence. **CCS** current-source delay when the Liberty has it, **effective capacitance** with resistive shielding (π-model reduction, Ceff iteration), and a **waveform-into-RC transient** for net delay and sink-slew degradation, measured at the library's own slew thresholds. |
| **Very large designs (>10 M gates)** | **not proven** | Runs just under **1 M instances** in ~33 s at ~11 GB, single-threaded. Nothing at 10 M has been run, so nothing is claimed there. Measured numbers, and the thing that actually governs them, are below. |
| **Fits an existing flow** | **partial** | Standard formats end to end: Liberty (NLDM/CCS/LVF), gate-level Verilog or Yosys JSON, SPEF, SDC in; SDF back-annotation and JSON reports out. Driven by a declarative job file, or an **experimental** OpenSTA-style Tcl subset. That subset is not a drop-in for a Tcl-driven sign-off flow, and is not meant to be — see below. |

The first two are the point: they are the margins the open baseline does not give you, and they
are why running this **alongside** an open sign-off timer tells you something new rather than
repeating it. The last two are honest limits, not roadmap promises.

#### Measured scale

Single-threaded, one core, no tuning. The second row is a public benchmark
([ISPD 2013 `netcard`](http://www.ispd.cc/contests/13/ispd2013_contest.html)), so it is
reproducible rather than a number you have to take on trust.

| design | instances | interconnect | wall | peak RSS |
| --- | --- | --- | --- | --- |
| routed sky130 block | 144 k | 20 MB SPEF | **10 s** | 0.5 GB |
| ISPD 2013 `netcard` | **982 k** | ideal | **33 s** | 10.6 GB |
| ISPD 2013 `netcard` | **982 k** | **1.3 GB SPEF** | **45 min** | 17.5 GB |

**With parasitics, what governs the runtime is extraction detail, not gate count.** The per-net
RC transient solve dominates, and it scales with how finely each net was extracted:

| design | instances | SPEF per instance | wall per instance |
| --- | --- | --- | --- |
| routed sky130 block | 144 k | 0.14 KB | **0.07 ms** |
| ISPD `des_perf` | 107 k | 0.87 KB | **2.38 ms** |
| ISPD `netcard` | 982 k | 1.39 KB | **2.73 ms** |
| ISPD `cordic` | 35 k | 1.07 KB | **2.85 ms** |

The three densely extracted designs cluster at **2.4–2.9 ms per instance across a 28× range in
size**, while the sky130 block — extracted six to ten times less finely — runs **34× faster per
instance**. So "how many gates" is the wrong question to ask about runtime here; "how detailed
is the SPEF" is the right one.

Practical reading: on a densely extracted netlist, run without parasitics for the fast iteration
loop, and with them when you want the SI and RC-accurate answer. The two are not close in cost,
and on this benchmark the parasitics move WNS by well under a picosecond on a 2 ns period — so
which one you want is a real choice, not a formality.

### Where it sits vs OpenSTA / the commercial signoff timer — run it *first*, not *instead*

`vyges-sta-si` is an **early-flow and complementary** engine, **not a tapeout
sign-off replacement** for OpenSTA or the commercial signoff timer. It is *correlated to*
OpenSTA — on a routed sky130 block, setup and hold agree to an RMS of **0.03 ns per
endpoint** over the endpoint set both report (see *Current state*) — and it is far narrower:
it covers a flop-based standard-cell abstraction and declines the rest (see *What it does
not model*). So it runs **upstream of**, and **alongside**, the signoff tool, never in lieu
of it for tape-out:

| Stage | Run | Why |
| --- | --- | --- |
| RTL / synth / P&R **iteration** | **`vyges-sta-si`** as the fast inner-loop + CI gate | std-only binary, no Tcl, `--fail-on-violation` exit 3 — catch timing breaks in seconds before spinning a full signoff run |
| **Pre-signoff** (open flow) | `vyges-sta-si` **then OpenSTA** | OpenSTA is the open signoff authority; sta-si adds the two margins OpenSTA genuinely lacks — **SI/crosstalk** and **statistical (AOCV/POCV-LVF) OCV** — as a second opinion. *(Multi-clock, timing exceptions, CRPR and flat-OCV derate OpenSTA already does, from the same SDC — so those agree, by design.)* |
| **Tape-out signoff** (if licensed) | `vyges-sta-si` early, **the commercial sign-off timer** for signoff | The commercial sign-off timer is the authority; sta-si stays the fast iteration loop + SI/crosstalk cross-check. Don't replace it with sta-si for the mask set |

So the rule of thumb: **run `vyges-sta-si` first and often** (iteration + regression
gate + SI/crosstalk + statistical-OCV insight), then hand off to **OpenSTA** (open
signoff) or **the commercial sign-off timer** (licensed signoff) for the authoritative final numbers. Its
unique value even when you have the commercial sign-off timer is the fast, license-free loop and the bundled
SI + statistical-OCV view.

### Run an OpenSTA-style script — `tcl` (experimental)

If you already have an OpenSTA TCL script, the **experimental** `tcl` subcommand runs its
portable subset through the Vyges engine — no `.sta` job needed:

```sh
vyges-sta-si tcl design.tcl            # read_liberty/verilog/spef/sdc + report_checks/wns/tns
vyges-sta-si tcl design.tcl --fail-on-violation   # same CI gate (exit 3)
```

It reuses the SDC parser for all constraints (`read_sdc` + inline `create_clock`/`set_*`) and
emits OpenSTA-flavoured reports. Commands outside the subset (`read_current_odb`,
`estimate_parasitics`, `report_power`, `check_setup`, …) are **listed and skipped**, never
silently dropped. **It is experimental, not a TCL interpreter, and not a drop-in for
LibreLane's `corner.tcl`** — see [`docs/opensta-integration.md`](docs/opensta-integration.md)
for the boundary and the production (LibreLane-step) path.

**Why this exists** — *can you use the Rust Vyges engines inside an OpenROAD stack?* **Yes.**
The engine doesn't *need* Tcl (it's declarative by design — no Tcl typos or copy-paste corner
drift), but your OpenROAD / OpenSTA flow speaks Tcl today. So we built this adapter to let you
try the Rust engine in your existing stack with **zero rewrite** — the on-ramp, not the
destination (the `.sta` job stays the recommended driver). **It's experimental and we want your
feedback**: which OpenSTA commands and report formats does your flow actually depend on? File a
feature request (`vyges-sta-si --feature-request`) — real usage should draw the subset boundary.

### Coming from Yosys — feed `write_json` straight in

If you synthesise with **Yosys**, you do not need to write Verilog back out for us to re-parse.
`vyges-sta-si` reads Yosys's JSON netlist directly: point `netlist:` at a `.json` and the reader
is chosen from the extension.

Whatever your Yosys script does, end it with `write_json` instead of `write_verilog`:

```sh
# ... your usual synthesis + tech mapping ...
yosys -p "read_verilog rtl/*.v; synth -top top; \
          dfflibmap -liberty cells.lib; abc -liberty cells.lib; \
          write_json top.json"
```

```text
design:  top
netlist: top.json        # Yosys write_json — .v works exactly the same way
lib:     cells.lib
clock:   clk 1.0
```

```sh
vyges-sta-si run top.sta --json
```

Skipping the Verilog round trip removes a step that costs time and invites dialect drift — the
netlist you time is the one Yosys actually produced.

**The format changes nothing about the answer.** Both readers build the same in-memory netlist,
so slack, the critical path and the SDF output are identical either way. That is asserted, not
assumed: [`examples/top/top_yosys.json`](examples/top/top_yosys.json) is genuine `write_json`
output for [`top.v`](examples/top/top.v), [`top_json.sta`](examples/top/top_json.sta) is the
matching job, and [`tests/yosys_json.rs`](tests/yosys_json.rs) checks the two jobs agree on WNS,
TNS and the worst endpoint.

Already have a gate-level netlist and just want the JSON? Yosys will convert it without
synthesising anything:

```sh
yosys -p "read_verilog top.v; write_json top.json"
```

Post-layout, add your `spef:` as usual — the netlist source is independent of parasitics.

**What about ABC's `-liberty` timing?** ABC reads Liberty and has its own static timer
(`read_lib`, `stime`), and it is *not* OpenSTA — it is UC Berkeley ABC's own, independent of any
sign-off timer. But it is **mapping-time estimation**: it exists to steer gate selection during
technology mapping, so it works from library delays alone — no parasitics, no SDC exceptions or
generated clocks, no crosstalk, no OCV/AOCV derating. That is the right trade for choosing
gates, and the wrong basis for believing a number.

So the two do not overlap and nothing needs reconciling: ABC picks the gates, then
`vyges-sta-si` times the netlist it produced, with SPEF, your real SDC, SI and derating applied.
Mapping estimate first, timing analysis second.

### Integrating into a flow — three ways (the leanest needs no adapter)

1. **Direct binary** *(recommended for new / upstream integration)* — `vyges-sta-si` is a plain
   binary: a declarative `.sta` job in, **JSON out**, CI-gating exit code. Any tool or flow —
   OpenROAD, LibreLane, a custom orchestrator, any language — can just `vyges-sta-si run job.sta
   --json`. **No Python, no Tcl, no linking.**
2. **`vyges-sta-si tcl`** (above) — for existing OpenSTA *scripts*.
3. **LibreLane Step** — for LibreLane-native *metrics* beside OpenSTA's
   ([`integrations/librelane/`](integrations/librelane/)).

The Tcl and Python paths are *conveniences*; the binary is the contract. **Integrating a Vyges
engine directly into your tool and have questions or challenges?** We're happy to help and to
shape the binary interface to your needs — reach us at **<https://vyges.com/contact>**. See
[`docs/opensta-integration.md`](docs/opensta-integration.md) for all three paths in detail, and
[`docs/engines-integration.md`](docs/engines-integration.md) for **how all four Vyges engines
(`char`/`extract`/`sta-si`/`em-ir`) work together** and where each plugs into a flow.

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
   correlation band ⇒ confidence; a **gap ⇒ the SI/crosstalk or statistical-OCV delta
   sta-si adds** — coupling or variation risk to inspect in the signoff tool.
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
vyges-sta-si run  top.sta --sdf top.sdf        # also write SDF back-annotation
vyges-sta-si sdc-lint top.sta                  # check the SDC for completeness/consistency
vyges-sta-si check top.sta                     # validate the job + inputs
vyges-sta-si demo                              # analyze a built-in 2-gate design
# common flags: -o FILE · --json · --sdf FILE · -q/--quiet · -v/--verbose · -h/--help · -V/--version
```

### SDC constraint lint (`sdc-lint`)

A correct slack report is worthless if the constraints are wrong — an unconstrained input
has no path to check, a missing clock leaves registers untimed, a clock on a port the design
doesn't have silently does nothing. `vyges-sta-si sdc-lint job.sta` checks the constraints
themselves, independent of the timing run, against the same netlist + Liberty:

```text
vyges-sta-si sdc-lint — 1 error(s), 2 warning(s)
  error   [no-clock]              design has registers but the SDC defines no clocks
  warning [unconstrained-input]   input `din` has no set_input_delay
  warning [unconstrained-output]  output `dout` has no set_output_delay
```

It flags only what is structurally certain — a non-positive clock period, a duplicate clock
name, a clock whose source isn't a port/net in the design, an I/O delay on a port that doesn't
exist, and uncovered primary inputs/outputs — so a clean lint means something. Exit 3 on any
**error** (or, with `--fail-on-violation`, on warnings too). Clock-tree tracing and exception
reachability are the depth passes (and partly the job of `vyges-cdc`).

### SDF back-annotation output (`--sdf`)

`vyges-sta-si run job.sta --sdf out.sdf` also writes a standard **SDF** (`DELAYFILE`): per-cell
**IOPATH** delays (rise/fall from the Liberty arcs at the net load), **TIMINGCHECK** SETUP/HOLD
on sequential cells, and top-level **INTERCONNECT** net delays from the SPEF parasitics (Elmore).
That is the standard hand-off a **gate-level / back-annotated simulator** consumes — produced
from the same Liberty + SPEF the rest of the flow already has, with no Tcl and no external STA.
Scope: single-corner today; IOPATH uses a nominal slew + the real net load (full
timer-propagated slew is the planned accuracy upgrade), and INTERCONNECT needs a SPEF (omitted
without one).

A job (`*.sta`) is a few `key: value` lines:

```text
design:      top
netlist:     top.v          # gate-level Verilog — or top.json from Yosys write_json
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
| --- | --- |
| `create_clock` / `create_generated_clock` | clock(s); a generated clock's period is resolved from its master × `divide_by` / `multiply_by` |
| `set_input_delay` / `set_output_delay` | I/O timing budget — default (`all_inputs`/`all_outputs`) plus per-port overrides; seeds input arrival / eats the period at outputs |
| `set_clock_uncertainty [-setup | -hold]` | guard band — tightens setup required, relaxes hold required |
| `set_clock_latency` | source/network latency, applied to the I/O budget |
| `set_input_transition` / `set_load` | boundary slew / load |
| `set_timing_derate -late | -early` | flat OCV derate |
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

## Design philosophy

Four rules shape this engine, and they explain most of what it does and does not do.

**1. It is a transcription, not an independent implementation.** Where `vyges-sta-si`
disagrees with OpenSTA, that is a defect here — not a difference of opinion, and not
something to argue about. The rules are read out of the reference's source, and each one is
pinned by a test that states the rule it came from. The corollary trips people up: being
*more accurate* than the reference is also a defect. When the reference computes an
increment in single precision, or uses a deliberately coarse fast `exp`, so does this
engine, because the goal is to land on the same number rather than a better one.

**2. Reading the algorithm is not enough — the call sequence carries as much of the
behaviour.** Several of the largest corrections in this engine were not arithmetic at all:
which of three implementations a virtual call resolves to, whether a flop is launched at
all, which of a pair of edges a check belongs to. Those are decided by dispatch and order,
not by a formula, and a faithful formula in the wrong place is still wrong.

**3. Correlation is judged per endpoint, never from WNS or WHS alone.** A worst-slack number
is one endpoint out of hundreds, and it hides everything that is not the minimum. This
engine has been measured with a setup WNS 1.9 % from the reference while a tenth of its
endpoints were out by more than 11 ns. Every correlation claim below is a distribution over
all shared endpoints; the headline number is quoted after it, never instead of it.

**4. Additions layer on top of the reference model, never in front of it.** SI/crosstalk,
AOCV/POCV and path-based analysis are extensions this engine has and the reference does not.
They are opt-in and they sit *beside* the transcribed model. The rule exists because it was
broken once: a home-grown interconnect model was tried first and silently displaced the
reference's for every net, and no output said so.

## What it does not model

Deliberately listed, because a timer that is quiet about its gaps is worse than one that
is narrow. `vyges-sta-si` covers a **flop-based standard-cell abstraction**, and outside
that it does not approximate — it simply has nothing to say:

- **Transparent latches.** There is no D→Q transparent path and no time borrowing. A
  latch-based design is not timed less accurately here; it is not timed.
- **Derived generated clocks.** A divided or generated clock is carried as its own clock
  with a stated period. It is not derived from a master's waveform, so `-divide_by`,
  `-edges` and edge-shift semantics are not honoured.
- **Checks other than setup, hold, recovery and removal** — no clock-gating checks, no
  minimum pulse width, no minimum period, no maximum skew.
- **Design-rule limit checks** — no maximum capacitance, transition or fanout reporting.
- **Power.** Use [`power`](https://github.com/vyges-tools/power).
- **SDF back-annotation as an input.** This engine *writes* SDF; it does not read one and
  time from it. Its delays are always its own.
- **Path enumeration.** It reports the worst setup path, the worst hold path and a
  per-endpoint slack list — not the N worst paths per group, and it has no path-group
  taxonomy, so recovery and removal checks are reported alongside hold rather than in a
  separate asynchronous group.

If a design needs any of the above, run the signoff timer for it. That is the same advice
as the section above, made specific.

## Domain coverage

`vyges-sta-si` operates on the **standard-cell digital abstraction** — it builds a timing
graph over **Liberty cell arcs** on a clocked gate-level netlist (NLDM/CCS delay tables,
setup/hold constraints, OCV derates). That makes it a **digital timing sign-off** engine: it
applies wherever a design reduces to characterized standard cells and a clock. It does **not**
apply to analog / mixed-signal blocks — their timing and behavior have no standard-cell or
Liberty-arc analogue, so there is nothing for the timing graph to traverse. For analog /
mixed-signal physical and integrity coverage, reach for the analog-capable Vyges engines —
[`lvs`](https://github.com/vyges-tools/lvs), [`layout`](https://github.com/vyges-tools/layout),
[`em-ir`](https://github.com/vyges-tools/em-ir), [`thermal`](https://github.com/vyges-tools/thermal),
and [`extract`](https://github.com/vyges-tools/extract).

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

## Current state (2026-09-04)

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
**multi-clock** (cross-domain paths use the tightest launch→capture
edge relation, not a single period; a divided or generated clock is carried as its own
clock with a stated period — it is not *derived* from a master's waveform), **timing exceptions** (false paths and
multicycle paths, matched on launch/capture instance or port), and **CCS-into-RC
delay** — a current-source model (`output_current` waveforms) plus an **effective
capacitance**: the driver behind a resistive net sees C1 + shielded-C2, not the
lumped total (Ceff iterated to convergence with the output slew), so cell delay
drops on resistive nets (this benefits NLDM too, not just CCS). The interconnect
delay to each sink is the reference delay calculator's own: the driver's Pi model
from an admittance-moment reduction, an **effective capacitance** from a
three-equation Newton solve, and a per-sink **wire delay and degraded sink slew**
taken from the resulting waveform's threshold crossings. A resistive net therefore
hands the next stage a slower edge, raising its delay. (A transient
waveform-into-RC solve is also available, `rc_model: transient`, as an explicit
opt-in beside that model rather than in front of it.) With `pba: true` it adds **path-based analysis** — re-timing the
critical path and its fan-in alternatives with strictly path-local slews, catching
a non-greedy worst path that the graph-based max can miss. It also writes **SDF
back-annotation** (`--sdf`: IOPATH + setup/hold TIMINGCHECK + SPEF INTERCONNECT, for
gate-level sim) and lints constraints with **`sdc-lint`** (completeness/consistency
of the SDC, independent of the timing run). Fully offline, no external deps, 216 tests
green.
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

**Correlated against OpenSTA per endpoint**, on a routed sky130 block (post-route netlist,
OpenRCX SPEF, the same Liberty and SDC given to both), with the reference re-run rather than
read from an archive:

- **setup** — RMS **0.032 ns** over the 790 endpoints both timers report, 7 of them outside
  0.1 ns; WNS 6.7987 against 6.8508.
- **hold** — RMS **0.034 ns** over 664 endpoints, 3 outside 0.1 ns; WHS 0.8824 against
  0.8821.
- Both timers report the **same endpoint set**, 790 and 664, with none extra on either side.
- The ECO planner built on this timer proposes **zero** delay cells on that block, which the
  reference and the sign-off run both call hold-clean.

The distributions are quoted before the headline slacks deliberately. Earlier in this
engine's life setup WNS sat 1.9 % from the reference while a tenth of its endpoints were out
by more than 11 ns — a worst-slack figure cannot see that, and neither can a reader who is
only given one.

On a **real routed sky130 block** (post-route netlist + OpenRCX SPEF), measured against the
sign-off timer's own checked-in results rather than a re-run:

| | agreement |
| --- | --- |
| **Per-arc delay** — median over ~58 000 back-annotated rise/fall arcs | **~2 %** |
| **Setup WNS**, on the same critical path the sign-off timer reports | **~1 %** |
| A reg→reg-class path both timers rank near the top | **~0.7 %** (~0.3 % of the clock period) |
| **Hold** — every check the sign-off timer reports, matched, and consistently on the pessimistic side | **sub-nanosecond** |

Per-arc delay is the sharpest of these, because it depends on neither path search nor
constraints — it isolates the delay calculator from everything else. The WNS figure is quoted
last on purpose: a slack number only means something once both timers are reporting the *same
path*, and reaching that took closing check-coverage gaps (recovery/removal checks, constant
propagation, clock-source→output-port paths) rather than touching the delay engine.

When a pin carries a CCS **receiver_capacitance** model (emitted by `vyges-char`),
the driver is loaded with the Miller-aware effective input cap (the C1/C2 segments)
rather than the static `capacitance` — a small, correct-direction increase in net
load and delay. v1 uses a representative scalar; full slew/load-resolved receiver
load is future.

The road to sign-off grade builds on the same graph: slew/load-resolved receiver
load and widening PBA from 1-exchange to k-worst enumeration. The SI margin it adds over OpenSTA stays.
