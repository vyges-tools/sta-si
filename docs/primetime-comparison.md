# vyges-sta-si vs Synopsys PrimeTime

An honest, engineer-to-engineer comparison. **PrimeTime (with PT-SI) is the
gold-standard sign-off tool** — decades of development, silicon-correlated, the
reference every other STA tool is measured against. `vyges-sta-si` is a from-scratch,
std-only Rust engine that has reached a meaningful *conceptual* subset and correlates
with OpenSTA on fundamentals. **It is not a PrimeTime replacement for tapeout sign-off
today.** This doc says exactly where we stand and what is — and isn't — reachable.

## Feature-by-feature

| Capability | PrimeTime / PT-SI | vyges-sta-si | Gap |
| --- | --- | --- | --- |
| Delay model | NLDM **+ CCS** (composite current source) | NLDM + **CCS-into-RC**: current-source `output_current`, **iterated Ceff↔slew** effective cap (driver shielding — for NLDM too), and a **transient waveform-into-RC** interconnect solve (backward-Euler RC tree, 0.69·RC not Elmore). Remaining: CCS **receiver models** (2-segment cap) + driver/receiver-model nuances | **small–medium** (accuracy) |
| Setup / hold / CK→Q | full | ✅ — OpenSTA-exact on single arcs | small |
| Unate (rise/fall) propagation | full | ✅ rise/fall lanes | small |
| SI / crosstalk | coupled-RC **delay + noise/glitch**, multi-aggressor, logical correlation | Miller-cap, slew-windowed **delay only** | medium–large |
| Parasitics reduction | moment matching / PI / Arnoldi | per-pin **Elmore** tree (+lumped) | medium |
| OCV | flat / AOCV / **POCV via LVF** (slew·load moments) | flat / AOCV(depth) / POCV(single σ) | medium |
| CRPR | rigorous, full clock network | nearest-common-node credit | small–medium |
| MMMC | distributed multi-scenario (DMSA) | serial scenarios, worst-merge | medium (scale) |
| Multi-clock / generated | waveforms, phase, async/exclusive groups, virtual clocks | periods + simple generated + edge relation | medium |
| Timing exceptions | full SDC, `-through`, path groups | false-path + multicycle (from/to) | medium |
| **Path-based analysis (PBA)** | yes — removes graph pessimism | **partial** (`pba:true`): our GBA already ties slew to the arrival-winning path (no worst-slew-merge pessimism); PBA re-times the critical path + 1-exchange fan-in alternatives, catching non-greedy worst paths. Not yet full k-worst enumeration | **small–medium** |
| Full SDC constraints | complete | tiny `.sta` subset | medium |
| ECO / what-if / hierarchical | full | none | large (scope) |
| Scale | multi-million-instance, incremental, distributed | single block, in-memory | large (scale) |
| Sign-off status | **silicon-correlated** | correlated to OpenSTA (a tier below) | the real bar |

## Correlation today

Against **OpenSTA 2.7.0** on a sky130 reg→reg: single-arc paths match to 4 decimals
(global WNS, DFF CK→Q); multi-stage paths agree within ~3% (conservative), the residual
being second-order slew propagation. OpenSTA itself is *not* PT-SI-grade, so "matches
OpenSTA" places us one tier below PrimeTime, not at it.

## Can we get into PrimeTime's ballpark over time?

Two different questions, two different answers.

**Block-level accuracy (within a few % on reported paths, for the nodes we target):
yes, reachable with focused engineering.** The remaining accuracy gaps are *known
algorithms*, not unknowns:
- **CCS delay** (current-source model) — the single biggest accuracy lever at advanced
  nodes. **Landed:** `ccs.rs` parses `output_current` waveforms and, crucially, the
  **CCS-into-RC** step — an **effective capacitance** (`ceff`) from the SPEF π-model
  (`pi_reduce`): the driver behind a resistive net drives C1 + shielded-C2 instead of
  the lumped total, so its cell delay drops (this corrects NLDM too); Ceff is
  **iterated to convergence**, and the interconnect uses a **transient waveform-into-RC**
  solve (backward-Euler RC tree → true sink response, 0.69·RC not Elmore's R·C).
  Remaining CCS depth: **receiver models** (2-segment receiver cap) and driver/receiver
  model nuances — diminishing returns from here.
- **Path-based analysis (PBA)** — recompute worst paths with path-specific slews to
  shed graph pessimism. Mostly graph plumbing on top of what we have.
- **LVF/POCV** (statistical moments), **higher-order RC reduction** (PI/Arnoldi),
  **clock waveforms/phases**, **fuller SDC** — all incremental, all known.

With CCS + PBA + LVF, block-level numbers in the single-digit-% range vs PT are a
realistic multi-quarter target.

**Full sign-off *trust* (betting a tapeout on it): a much higher bar, and not purely
an engineering one.** Matching PrimeTime as a sign-off authority means:
- **Silicon correlation** — calibration against measured silicon across nodes/corners.
  Needs foundry data, test chips, and time. This is the moat, and it's years + capital,
  not a sprint.
- **Scale + robustness** — full-chip, incremental, distributed, and a decade of
  corner-case hardening.
- **Ecosystem + certification** — ECO closure, methodology, and the institutional trust
  that makes a team stake a mask set on a number.

## So what *is* the goal

Not "replace PrimeTime for everyone." The goal is to be the **open, accessible,
good-enough-for-most STA+SI engine** — and the credible **trajectory** toward sign-off
grade — for the nodes and designs our users actually run. Concretely:
- A strong **early-flow / iterative / regression-gating** timing engine, today.
- A **second opinion** that carries SI + CRPR + OCV + MCMM + exceptions in *one open,
  license-free, embeddable binary* — a combination no open tool packages.
- On a multi-quarter path (CCS → PBA → LVF) into block-level accuracy parity for our
  target nodes, while honest that tapeout-sign-off *trust* is a longer, data-driven road.

The differentiator isn't beating PrimeTime at its own game — it's making commercial-grade
timing capability *open and accessible*, and owning it inside the Vyges flow.
