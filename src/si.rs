//! Signal-integrity delta-delay — the SI layer (reserved in v0).
//!
//! Crosstalk-induced delay is the headline of a sign-off STA+SI engine: a
//! switching aggressor net couples into a victim and shifts its delay, but only
//! when their switching windows overlap. Doing it right needs coupling
//! capacitance (from SPEF) plus iterative timing-window overlap analysis.
//!
//! v0 models **no crosstalk** — it returns zero delta-delay so the base STA is
//! exact and honest. The interface is fixed here so the engine can grow into it
//! (coupling from SPEF → window overlap → per-arc delta) without a redesign; the
//! engine surfaces the gap via `StaError::SiNotModeled` when SI is requested.

/// Crosstalk delta-delay (ns) added to a victim net's stage delay.
/// v0: always 0.0 (no coupling model yet).
pub fn delta_delay() -> f64 {
    0.0
}

/// Whether a real crosstalk model is available. v0: false.
pub fn modeled() -> bool {
    false
}
