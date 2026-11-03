//! Signal-integrity crosstalk delta-delay: the closed-form bound.
//!
//! A coupling capacitance `Cc` between a victim net and a switching aggressor
//! shifts the victim's delay: when the aggressor switches *against* the victim,
//! the Miller effect makes `Cc` look like up to `2·Cc` of grounded load, slowing
//! the victim. [`xtalk_delta_ns`] is that bound for a lumped net: the victim's
//! interconnect delay gains `R · (MCF − 1) · Cc`, where `MCF` (the Miller
//! coupling factor, ~2 for the worst late case) is set per run. The nominal net
//! delay already counts `Cc` once as grounded (`MCF = 1`), so only the extra
//! `(MCF − 1)·Cc` is added.
//!
//! The timer itself is window-aware (see the per-arc interconnect loop in `sta.rs`):
//! every coupling capacitance is grounded on the victim once, and the extra
//! `(MCF − 1)·Cc` of an aggressor is added only when its driver's arrival falls within
//! the victim's switching window (the two nets' mean slew plus the run's `xtalk_window`
//! guard). Arrivals set the windows and the windows feed back into the arrivals, so the
//! late interconnect is iterated to a fixed point. The early (hold) interconnect takes no
//! aggressor term, at the converged operating point. Pure std — unit-tested offline.

/// Crosstalk delta-delay (ns) for a victim net: `R[Ω] · (MCF−1) · Cc[fF]`.
pub fn xtalk_delta_ns(res_ohm: f64, coupling_ff: f64, miller: f64) -> f64 {
    res_ohm * (miller - 1.0).max(0.0) * coupling_ff * 1e-6
}

/// Whether a crosstalk model is applied: always (window-aware in the timer).
pub fn modeled() -> bool {
    true
}
