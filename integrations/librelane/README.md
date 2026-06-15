# Integrating `vyges-sta-si` with LibreLane

> **EXPERIMENTAL** — Layer 2 of the OpenSTA integration
> (see [`../../docs/opensta-integration.md`](../../docs/opensta-integration.md)).

This directory is a LibreLane **plugin Step** that runs `vyges-sta-si` as a **second-opinion
timer** inside a LibreLane / OpenROAD flow. It does **no TCL parsing** — it consumes the
structured **State** the flow already has and writes `vyges__…` metrics **next to** OpenSTA's,
so the SI + statistical-OCV delta is just a metric comparison. It runs *alongside* OpenSTA; it
does not replace the signoff timer.

---

## Do you even need this plugin? — the direct-binary alternative

**If you only want the numbers, you don't need any of this Python.** `vyges-sta-si` is a plain
binary: a declarative job in, JSON out. Any flow step (or `Makefile`, or shell script) can call
it directly:

```sh
vyges-sta-si run corner.sta --json -o corner.json   # then parse corner.json
```

Use the plugin below **only** when you want the result wired into LibreLane as native
`vyges__…` **metrics** (so it shows up in the run's `metrics.json` beside OpenSTA's). For
anything else, the binary is the cleanest contract. *(Integrating directly and have questions?
File at <https://vyges.com/contact> — we'll help; see the design doc's "Direct binary" section.)*

---

## Integration walkthrough (the plugin)

### 1. Get the engine binary

Build it (`cargo build --release` in the repo root) and either put `target/release/vyges-sta-si`
on `$PATH`, or set the `VYGES_STA_SI_BIN` config var (below) to its absolute path.

### 2. Make the plugin importable

Put this directory on `PYTHONPATH` so LibreLane can import the modules (which registers the
step and the example flow):

```sh
export PYTHONPATH="/path/to/vyges-tools-sta-si/integrations/librelane:$PYTHONPATH"
```

### 3. Use the provided flow (or append the step to your own)

`example_flow.py` registers **`ClassicWithVyges`** = the stock `Classic` flow with `Vyges.StaSi`
appended as a trailing step (safe — by the end, State has the post-PNR netlist, per-corner SPEF,
and signoff SDC the step reads):

```sh
librelane --flow ClassicWithVyges <your-config.json>
```

To add it to a *custom* flow instead, append the step class to your flow's `Steps`:

```python
import vyges_sta_si_step                       # registers Vyges.StaSi
from librelane.flows import Flow, SequentialFlow

@Flow.factory.register()
class MyFlowWithVyges(SequentialFlow):
    Steps = list(Flow.factory.get("MyFlow").Steps) + [vyges_sta_si_step.VygesStaSi]
```

### 4. (Optional) configure it

In your design config (`config.json`):

| Variable | Default | Meaning |
| --- | --- | --- |
| `VYGES_STA_SI_BIN` | `vyges-sta-si` | path/name of the engine binary |
| `VYGES_STA_SI_MILLER` | `2.0` | crosstalk Miller factor (2.0 worst-case; 1.0 disables SI) |

### 5. Read the result

After the run, the step's `vyges__…` metrics are in the run's `metrics.json` (and the step dir):

```
vyges__timing__setup__ws__corner:<corner>     # per corner
vyges__timing__setup__tns__corner:<corner>
vyges__timing__hold__ws__corner:<corner>
vyges__timing__hold__tns__corner:<corner>
vyges__timing__setup__ws                       # worst (min) across corners
vyges__timing__hold__ws
```

**The second opinion = the comparison.** For each corner, diff Vyges (SI-aware) against the
OpenSTA value the flow already produced:

```
vyges__timing__setup__ws__corner:<c>   vs   timing__setup__ws__corner:<c>
```

A meaningful gap is the crosstalk/statistical-OCV margin OpenSTA-in-LibreLane doesn't model —
inspect it in your signoff tool. *(A timing violation is recorded as a metric here, **not** a
step failure; the flow's own checkers decide pass/fail.)*

### What it reads from the flow (per corner)

- **netlist** — `DesignFormat.NETLIST` (State)
- **liberty** — `toolbox.get_timing_files_categorized(config, corner)` (the same per-corner libs OpenSTA uses)
- **SPEF** — per-corner `DesignFormat.SPEF` (State), if extracted
- **SDC** — `DesignFormat.SDC` (State), else `SIGNOFF_SDC_FILE` / `PNR_SDC_FILE`

---

## Files

| File | Role |
| --- | --- |
| `job_builder.py` | Pure (librelane-free) core: State paths → `.sta` job, JSON → metrics. Unit-tested. |
| `vyges_sta_si_step.py` | The LibreLane `Step` (`Vyges.StaSi`). |
| `example_flow.py` | `ClassicWithVyges` — Classic + the step. |
| `test_job_builder.py` | `python3 test_job_builder.py` — 8 tests of the pure core (no LibreLane needed). |

## Status

The pure job-builder core is unit-tested. The Step is written against the real LibreLane `Step`
API but **not yet run end-to-end in a full flow** — that's the remaining validation pass (run
`ClassicWithVyges` on a real design and diff `vyges__…` vs `timing__…`). Experimental until
then; graduates into the Sley orchestrator's step set later.
