# How the Vyges EDA engines work together — and where to integrate them

> **Moved.** The cross-engine integration guide is now maintained once, canonically, at
> **<https://docs.vyges.com/engines/integration.html>** — where each Vyges engine plugs into an
> OpenROAD / LibreLane / OpenLane 2 flow, the "drop these in" pre-P&R vs post-layout split, and
> the three ways to integrate (direct binary, incumbent-script adapter, orchestrator step). This
> stub stays so existing links keep working; the cross-engine map no longer lives per-repo (to
> avoid copy-drift).

## Where `vyges-sta-si` sits

Static timing **with signal integrity** at sign-off — the SI/crosstalk + statistical-OCV answer
base OpenSTA can't give. It reads the `.lib` from `vyges-char` and the `.spef` from
`vyges-extract`; pre-P&R it runs on the synth netlist + SDC as a shift-left timing gate, and
post-route on the real parasitics. For most teams this is the highest-value single drop-in.

## sta-si-specific depth (code-coupled, stays in this repo)

- [`docs/opensta-integration.md`](opensta-integration.md) — the experimental OpenSTA-TCL-subset
  adapter and the LibreLane plugin Step.
- `integrations/librelane/` — the LibreLane Step that emits `vyges__…` metrics.
