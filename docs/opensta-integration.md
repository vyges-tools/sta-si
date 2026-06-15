# OpenSTA Integration — design

*Status: design / intent. Created 2026-06-08.*

> Goal: make `vyges-sta-si` consumable by the flows that today drive **OpenSTA** —
> standalone OpenSTA scripts and, in production, **LibreLane / OpenROAD** — so a team
> can run the Vyges timer as a drop-in for the basic timing question *and* as the
> **SI + statistical-OCV** "second opinion" the README promises. This doc explains **what
> the OpenSTA "TCL files" actually contain**, why the obvious adapter is the wrong target,
> and the **two-layer adapter** that is right.

---

## Rationale — can the Rust Vyges engines plug into an OpenROAD stack?

**Short answer: yes — and that's what this adapter is for.** The Vyges EDA engines are
independent, std-only **Rust** binaries; they are *correlated against* OpenSTA, not built on
it. By design they take a small **declarative job file** (`.sta`), not a hand-written Tcl
script — deliberately, to kill the silent typos, copy-paste corner drift, and brittle control
flow that hand-authored Tcl invites.

So why build a Tcl adapter at all, if Vyges *doesn't need* Tcl? **To meet existing users where
they are.** The installed base — OpenROAD / LibreLane / OpenSTA flows — drives timing through
Tcl today. Asking those teams to rewrite their flow into `.sta` job files before they can even
*try* the Vyges timer is a barrier to adoption. The adapter removes it: **point your existing
OpenSTA script at the Vyges engine and get an answer, with zero rewrite.** It is the on-ramp,
not the destination — the declarative `.sta` job stays the recommended way to drive the engine;
the Tcl path is the low-friction way to **adopt it *alongside*** what you already run (the
README's "validate fast, sign off with your tool" posture).

The positioning in one line: **the Rust engine is the substance; the Tcl adapter is the
courtesy** — you get the SI + statistical-OCV second opinion without leaving the OpenROAD stack.

**This is experimental, and we solicit your feedback.** The OpenSTA command surface is large and
real flows lean on different corners of it. Tell us which commands, flags, and report formats
your flow actually depends on — that's what should decide where the next iteration of the subset
goes. File a feature request (`vyges-sta-si --feature-request`) or open a discussion; the
boundary of "supported subset" should be drawn by real usage, not guesswork.

---

## Three ways to integrate a Vyges engine — and the leanest is the binary itself

There are three ways to put `vyges-sta-si` (or any Vyges EDA engine) into a flow. **Pick the
lowest one that fits — they ascend in coupling, not capability.** *(For how all four engines —
`char`/`extract`/`sta-si`/`em-ir` — work together and where each plugs in, see
[`engines-integration.md`](engines-integration.md).)*

### 0. Direct binary — no Python, no TCL  *(recommended for new / upstream integration)*

The Vyges engines are **plain, std-only binaries**: a tiny declarative **job file in**, stable
**JSON out**. The leanest integration — for OpenROAD, LibreLane, a custom orchestrator, or a
script in *any* language — is to **call the binary directly**:

```sh
vyges-sta-si run timing.sta --json
# → {"design":"top","wns_ns":-0.05,"tns_ns":-0.2,"whs_ns":0.1,"met":false, ...}
#    exit 0 ok · 3 with --fail-on-violation if WNS<0  (a CI timing gate)
```

A `.sta` job is a few `key: value` lines (`netlist` / `lib` / `spef` / `sdc` / `clock`); the
JSON is documented and trivial to parse; the exit code gates CI. **No plugin, no interpreter,
no linking.** The same shape holds for the other engines — `vyges-char` (`.char` → `.lib`),
`vyges-extract` (`.ext` → SPEF), `vyges-em-ir` (`.emir` → IR/EM). The TCL adapter (path 1) and
the LibreLane Python step (path 2) below are **conveniences, not requirements** — for a clean
new integration, **the binary is the contract.**

> **Integrating a Vyges engine directly into your tool — OpenROAD, LibreLane, or anything else?**
> You do **not** need our Python or TCL; call the binary. And we want to make that binary
> interface whatever you need it to be. If you hit any question or challenge with direct
> binary-level integration — job-file fields, the JSON schema, exit codes, streaming, corners —
> **tell us at <https://vyges.com/contact>** (a bug report / feature request) and we'll be happy
> to work with you on it.

### 1. `vyges-sta-si tcl` (experimental) — for existing OpenSTA *scripts*

If you already have an OpenSTA Tcl script and don't want to rewrite it into a `.sta` job, the
**Layer 1 adapter** (§2 below) runs its portable subset through the engine. A courtesy on-ramp;
see below for the boundary.

### 2. LibreLane **Step** (experimental) — for LibreLane-native *metrics*

If you want the result as LibreLane metrics next to OpenSTA's (the SI/OCV second opinion in the
flow), use the [Layer 2 step](../integrations/librelane/) — it reads the flow's State and emits
`vyges__…` metrics. The integration walkthrough is in
[`integrations/librelane/README.md`](../integrations/librelane/README.md).

---

## 1. What the OpenSTA "TCL files" actually contain

OpenSTA is a TCL application: you run `sta -no_splash -exit <script.tcl>` and the script
issues commands into OpenSTA's embedded TCL interpreter. In a hand-written flow that
script is short and portable:

```tcl
read_liberty  sky130.lib          ;# cell timing models
read_verilog  top.v               ;# gate-level netlist
link_design   top
read_spef     top.spef            ;# parasitics
read_sdc      top.sdc             ;# clocks + I/O delays + exceptions  (or inline below)
create_clock  -name clk -period 5 [get_ports clk]
set_propagated_clock [all_clocks]
report_checks -path_delay max -format full_clock_expanded   ;# the worst paths
report_wns ; report_tns                                     ;# the scalars
```

That portable subset is the part `vyges-sta-si` can adapt to directly — and note **most
of it is already covered**: the engine reads Verilog/Liberty/SPEF, and `src/sdc.rs`
already parses `create_clock`, `set_input_delay`, `set_output_delay`,
`set_clock_uncertainty`, `set_clock_latency`, `set_input_transition`, `set_load`,
`set_false_path`, `set_multicycle_path`, `set_timing_derate`, `all_inputs/outputs`,
`get_ports/pins/clocks`, … (the `sdc:` job key already feeds it).

### But the *production* flow's TCL is not that

LibreLane invokes OpenSTA as `sta -no_splash -exit corner.tcl`, once per corner (MCMM:
ss/tt/ff). Its `corner.tcl` (verified on the box —
`librelane/scripts/openroad/sta/corner.tcl`) is a real TCL **program**, coupled to three
things a parser can't cheaply satisfy:

1. **OpenSTA's full command API** — not just `report_checks` (with `-sort_by_slack
   -path_delay min/max -fields {slew cap input net fanout} -format full_clock_expanded
   -group_path_count 1000 -corner ... -slack_max -unconstrained`), but `report_power`,
   `check_setup -verbose -unconstrained_endpoints -multiple_clock -no_clock …`,
   `report_check_types -max_slew -max_capacitance -max_fanout -violators`,
   `report_parasitic_annotation`, `worst_clock_skew`, and internal `sta::` procs
   (`sta::all_clocks`, `sta::max_slew_violation_count`, `sta::design_power`, …).
2. **LibreLane's `io.tcl`** helper library (`source $::env(SCRIPTS_DIR)/openroad/common/io.tcl`):
   the `lln::` corner namespace (`lln::get_corner_dict`, `lln::set_sta_cmd_corner`),
   `read_timing_info` (which itself calls `read_liberty -corner`, `read_verilog`,
   `read_sdc`, `link_design`), `read_spefs`, `write_metric_int/num`, and the
   **`%OL_CREATE_REPORT <name>.rpt` / `%OL_END_REPORT`** stdout markers LibreLane uses to
   slice stdout into named `.rpt` files + scrape metrics.
3. **OpenROAD's ODB** — `read_current_odb`, `estimate_parasitics -global_routing/-placement`
   (`grt::`, `est::`) when STA runs in‑process against the routed database.

**Conclusion:** "replace the `sta` binary so LibreLane's `corner.tcl` runs unchanged" means
re-implementing OpenSTA's TCL command API **and** LibreLane's `io.tcl` **and** OpenROAD's
ODB hooks. That is rebuilding OpenSTA's front-end — the wrong target, and it wouldn't even
let the Vyges engine's *extra* value (SI/CRPR/AOCV‑POCV) surface, because OpenSTA's report
format has nowhere to put it.

---

## 2. The right shape: a two-layer adapter

Split "TCL on one end, the engine on the other" into the two consumers that actually exist.

### Layer 1 — `vyges-sta-si tcl` : an OpenSTA-TCL-**subset** front-end  *(the literal adapter)*

> **Status: EXPERIMENTAL — implemented** (`src/tcl.rs`, the `tcl` subcommand). v1 covers the
> portable command subset below and is validated to match the equivalent `.sta` job's WNS
> exactly on the bundled `examples/top`. The command surface and report fidelity may change;
> it is **not** a TCL interpreter and **not** a drop-in for LibreLane's `corner.tcl`.

A subcommand that ingests an OpenSTA-style script restricted to the **portable
command subset** and drives the existing engine, emitting **OpenSTA-format text reports**.

```
vyges-sta-si tcl script.tcl [-o OUT] [--json] [--fail-on-violation]
```

**Command mapping (TCL verb → engine):**

| OpenSTA TCL | → `vyges-sta-si` |
| --- | --- |
| `read_liberty [-corner c] f.lib` | a `lib` input (per corner) |
| `read_verilog f.v` / `link_design top` | `netlist` + `design` |
| `read_spef f.spef` | `spef` input |
| `read_sdc f.sdc` / `source f.sdc` | feed to existing `src/sdc.rs` |
| inline SDC (`create_clock`, `set_input_delay`, `set_*`) | existing `src/sdc.rs` (already parsed) |
| `set_cmd_units` / `set_units` | record units (mostly no-op) |
| `set_propagated_clock [all_clocks]` | propagated-clock mode (engine already does latency/SPEF) |
| `report_checks [-path_delay min\|max\|min_max] [-fields …] [-format …] [-group_path_count N] [-slack_max X]` | run engine → emit worst-path report(s) in OpenSTA text format |
| `report_wns` / `report_tns` / `report_worst_slack -max/-min` | emit the scalar (engine already computes WNS/TNS/WHS/THS) |
| `report_clock_skew` | emit skew if computed |

**Explicitly out of scope (logged as "unsupported, ignored"), because they need OpenSTA
internals / OpenROAD / LibreLane:** `read_current_odb`, `estimate_parasitics`,
`report_power`, `check_setup`, `report_check_types` (max_slew/cap/fanout DRC checks),
`sta::*` procs, `lln::*`, `write_metric_*`, `%OL_*` markers.

This is a **focused TCL-subset lexer**, *not* a TCL interpreter — line-oriented, the verbs
above, brace/bracket-aware argument splitting. It reuses `sdc.rs` for the whole constraint
half. Deliverable entirely inside this repo. It makes `vyges-sta-si` a drop-in for
**hand-written / standalone OpenSTA scripts** and for a **Vyges-authored `corner.tcl`** —
not for LibreLane's OpenROAD-coupled one.

### Layer 2 — a LibreLane **Step** : the production integration  *(no TCL at all)*

> **Status: EXPERIMENTAL — implemented** in [`integrations/librelane/`](../integrations/librelane/)
> (`Vyges.StaSi` step + `ClassicWithVyges` flow). The pure State→`.sta`→metrics core is
> unit-tested standalone (`test_job_builder.py`); the Step is written against the real
> LibreLane `Step` API but **not yet run end-to-end in a full flow** — that's the remaining
> validation pass (run `ClassicWithVyges` on a real design, diff `vyges__…` vs `timing__…`).

For real LibreLane/OpenROAD flows, **don't parse TCL** — consume the structured **State**
LibreLane already has (`state_out.json` points at the netlist, per-corner `.lib`, `.spef`,
and the `.sdc`). A custom LibreLane `Step` (a thin Python plugin):

1. reads those paths from the incoming State + the corner list,
2. synthesizes a `.sta` (MCMM `scenarios:`) job — the same shape as `examples/icsprout55/mcmm.sta`,
3. runs `vyges-sta-si run … --json`,
4. writes the results back as LibreLane **metrics** (`timing__setup__ws`, `…__tns`, hold,
   plus the Vyges-only `…__si_delta`, `…__crpr`, OCV margins) and a `.rpt`.

This is where **the Vyges differentiation actually shows up**: run the Vyges step
*alongside* OpenSTA's (the README's "run first / second opinion" posture), and report the
**two margins OpenSTA genuinely lacks — SI/crosstalk and statistical (AOCV/POCV-LVF) OCV** —
as extra metrics next to OpenSTA's, not by impersonating its report format. *(Note: OpenSTA
already does CRPR — on by default in OCV mode — plus multi-clock, timing exceptions, and
flat-OCV derate, all from the same SDC; those are **not** the delta and must not be claimed
as one.)* Lands in the orchestrator layer (Sley / a LibreLane plugin), not in this repo.

### Layer 0 (declined) — embed at OpenSTA's C++ API (`StaApi`)

OpenSTA's [`StaApi.txt`](https://github.com/The-OpenROAD-Project/OpenSTA/blob/master/doc/StaApi.txt)
documents a **third** integration path we should name and **decline**: OpenSTA is built to be
*embedded and extended* — its TCL commands are a thin SWIG wrapper over the `Sta` C++ class, and
it exposes a **Network adapter** (~45 virtual functions to feed it an external netlist DB) and
`registerDelayCalc` / `ArcDelayCalc` to **plug in a custom delay calculator**. So one *could*
make Vyges a component *inside* OpenSTA (e.g. a Vyges delay calc registered into OpenSTA).
**We don't:** it would couple us to OpenSTA's C++ build and **invert our positioning** — we'd be
a plugin *enhancing OpenSTA*, not an **independent, std-only Rust engine** correlated against it.
The same StaApi fact *supports* Layer 1, though: since the TCL layer is the public API over
`Sta`, **targeting the portable TCL command subset is targeting OpenSTA's stable contract.**

---

## 3. Why this split is the honest, correct one

- **SDC is already the bridge.** The one artifact humans author — the SDC — is read directly
  today. Both OpenSTA and `vyges-sta-si` consume the *same* netlist/lib/SPEF/SDC, so the
  inputs already interoperate; the adapter is only about the **verbs** (read/link/report) and
  the **report shape**.
- **Layer 1** gives literal TCL↔engine interop for the portable subset (and is fully in this
  repo's control), satisfying "works with TCL on one end."
- **Layer 2** is the production path and the place the Vyges advantage is expressible — and it
  needs *no* TCL, just the State the flow already serialized.
- Neither layer tries to be OpenSTA's TCL runtime. We **adopt alongside**, exactly as the
  product page positions it ("validate fast, sign off with your tool") — we do **not** claim
  to run LibreLane's `corner.tcl` unmodified.

---

## 4. Phasing

1. **Layer 1 `vyges-sta-si tcl`** (this repo): the subset lexer + `report_checks/-wns/-tns`
   text emitter, reusing `sdc.rs`. Validate by running a hand-written OpenSTA script and an
   OpenSTA run on the *same* inputs and diffing WNS/TNS (the README already cites ~0.5%).
2. **Layer 2 LibreLane Step** (Sley / plugin): State→`.sta`→run→metrics, as a second-opinion
   step beside OpenSTA. This is the one that demonstrates the SI/CRPR/OCV delta in a real flow.
3. *(Optional, later)* a Vyges `corner.tcl`-equivalent that Layer 1 can run end-to-end for a
   standalone "Vyges timer, OpenSTA-style driver" experience.

## 5. Open questions

- **`report_checks` text fidelity** — how closely must the path report match OpenSTA's
  `full_clock_expanded` format? For Layer 1 standalone use, "close enough to read/diff" is
  fine; for any downstream *scraper*, match the WNS/TNS/worst-path lines exactly.
- **MCMM driving** — Layer 1 from repeated `read_liberty -corner` in one script vs. our
  `scenarios:` job model: map `-corner` tags to scenarios.
- **Where Layer 2 lives** — a standalone LibreLane plugin Step vs. inside the Sley
  orchestrator's step set (the engine adapter seam is the same one).

### Related
- `README.md` — the engine, formats, and the "run first / alongside OpenSTA" posture
- `src/sdc.rs` — the existing SDC parser (the constraint half of Layer 1, already done)
- `examples/icsprout55/mcmm.sta` — the MCMM job shape Layer 2 synthesizes
- The Sley orchestrator — the State⇄engine adapter seam Layer 2 rides
