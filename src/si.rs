//! Signal-integrity crosstalk delta-delay.
//!
//! A coupling capacitance `Cc` between a victim net and a switching aggressor
//! shifts the victim's delay: when the aggressor switches *against* the victim,
//! the Miller effect makes `Cc` look like up to `2·Cc` of grounded load, slowing
//! the victim. v1 is a **worst-case, window-free** bound: the victim's
//! interconnect delay gains `R · (MCF − 1) · Cc`, where `MCF` (the Miller
//! coupling factor, ~2 for the worst late case) is set per run. The nominal net
//! delay already counts `Cc` once as grounded (`MCF = 1`), so only the extra
//! `(MCF − 1)·Cc` is added here.
//!
//! Timing-window-aware crosstalk (only aggressors whose switching overlaps the
//! victim's, with real alignment) is the refinement — it requires iterating
//! arrival windows. Pure std — unit-tested offline.

/// Crosstalk delta-delay (ns) for a victim net: `R[Ω] · (MCF−1) · Cc[fF]`.
pub fn xtalk_delta_ns(res_ohm: f64, coupling_ff: f64, miller: f64) -> f64 {
    res_ohm * (miller - 1.0).max(0.0) * coupling_ff * 1e-6
}

/// Whether a crosstalk model is applied. v1: true (reduced worst-case).
pub fn modeled() -> bool {
    true
}
