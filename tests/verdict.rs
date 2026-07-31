// SPDX-License-Identifier: Apache-2.0
//! The accept policy for a speculative fix — G4 of the timing-driven ECO loop.
//!
//! The rollback *mechanism* lives in `vyges-opendb` (OpenDB's ECO journal). The *policy* — was
//! this fix worth keeping — needs timing, so it lives here. It is deliberately a pure function
//! over two reports: no database, no netlist, so it can be reasoned about and tested directly.
//!
//! The rule it encodes, and the reason it is not simply "nothing got worse":
//!
//! 1. the targeted metric must actually improve, because a fix costs area and a legalization
//!    disturbance even when it is harmless;
//! 2. the *other* check must not be **harmed** — pushed from met into violation, or an existing
//!    violation deepened. Trading setup margin for a hold fix is normal and must stay allowed.
use vyges_sta_si::sta::{judge, timing_delta, Check, RevertReason, TimingReport, Verdict};

const EPS: f64 = 1e-9;

/// A report with only the four headline numbers set — the rest is irrelevant to the policy.
fn report(wns: f64, tns: f64, whs: f64, ths: f64) -> TimingReport {
    TimingReport {
        wns,
        tns,
        endpoints: 0,
        worst_endpoint: String::new(),
        worst_path: Vec::new(),
        whs,
        ths,
        hold_endpoints: 0,
        worst_hold_endpoint: String::new(),
        worst_hold_path: Vec::new(),
        hold_slacks: Vec::new(),
        pba_wns: None,
    }
}

#[test]
fn delta_is_after_minus_before_so_positive_is_better() {
    let d = timing_delta(&report(-0.5, -2.0, 0.1, 0.0), &report(-0.2, -1.0, 0.05, 0.0));
    assert!((d.wns - 0.3).abs() < EPS, "WNS improved by 0.3");
    assert!((d.tns - 1.0).abs() < EPS, "TNS improved by 1.0");
    assert!((d.whs + 0.05).abs() < EPS, "hold margin shrank, so the delta is negative");
}

#[test]
fn a_setup_fix_that_improves_wns_is_kept() {
    let v = judge(&report(-0.5, -2.0, 0.2, 0.0), &report(-0.1, -0.4, 0.2, 0.0), Check::Setup, EPS);
    assert_eq!(v, Verdict::Keep);
    assert!(v.keep());
}

#[test]
fn a_fix_that_changes_nothing_is_rejected() {
    // Not free: it costs area and disturbs placement. "No worse" is not good enough.
    let same = report(-0.5, -2.0, 0.2, 0.0);
    assert_eq!(
        judge(&same, &same, Check::Setup, EPS),
        Verdict::Revert(RevertReason::NoImprovement)
    );
    assert_eq!(
        judge(&same, &same, Check::Hold, EPS),
        Verdict::Revert(RevertReason::NoImprovement)
    );
}

#[test]
fn floating_point_noise_is_not_progress() {
    let before = report(-0.5, -2.0, 0.2, 0.0);
    let after = report(-0.5 + 1e-15, -2.0, 0.2, 0.0);
    assert_eq!(
        judge(&before, &after, Check::Setup, EPS),
        Verdict::Revert(RevertReason::NoImprovement),
        "a sub-epsilon change must not count as an improvement"
    );
}

#[test]
fn a_hold_fix_may_spend_setup_margin() {
    // The case a naive "nothing got worse" rule would wrongly reject. Inserting delay to fix
    // hold costs setup margin almost by construction; as long as setup still MEETS, that is a
    // legitimate trade and the fix must be kept.
    let before = report(0.40, 0.0, -0.10, -0.30);
    let after = report(0.30, 0.0, 0.02, 0.0); // setup margin down, hold now met
    assert_eq!(judge(&before, &after, Check::Hold, EPS), Verdict::Keep);
}

#[test]
fn a_hold_fix_that_pushes_setup_into_violation_is_rejected() {
    // Same trade, one step too far: setup crosses from met to violating. That is harm, not a
    // trade, and the fix must be reverted with a reason that says so.
    let before = report(0.05, 0.0, -0.10, -0.30);
    let after = report(-0.02, -0.05, 0.02, 0.0);
    assert_eq!(
        judge(&before, &after, Check::Hold, EPS),
        Verdict::Revert(RevertReason::BrokeSetup)
    );
}

#[test]
fn a_hold_fix_that_deepens_an_existing_setup_violation_is_rejected() {
    // Setup was already failing; making it worse is still harm.
    let before = report(-0.10, -1.0, -0.10, -0.30);
    let after = report(-0.25, -2.0, 0.02, 0.0);
    assert_eq!(
        judge(&before, &after, Check::Hold, EPS),
        Verdict::Revert(RevertReason::BrokeSetup)
    );
}

#[test]
fn a_setup_fix_that_breaks_hold_is_rejected() {
    // The mirror case: upsizing shortens paths, which is exactly how hold gets broken.
    let before = report(-0.30, -1.0, 0.02, 0.0);
    let after = report(-0.05, -0.2, -0.04, -0.10);
    assert_eq!(
        judge(&before, &after, Check::Setup, EPS),
        Verdict::Revert(RevertReason::BrokeHold)
    );
}

#[test]
fn improving_a_still_violating_check_counts_as_progress() {
    // Repair is usually incremental — a fix that takes WNS from -0.5 to -0.3 has not closed
    // timing, but it moved the design forward and must be kept, or the loop can never converge.
    let v = judge(&report(-0.5, -3.0, 0.2, 0.0), &report(-0.3, -1.5, 0.2, 0.0), Check::Setup, EPS);
    assert_eq!(v, Verdict::Keep);
}

#[test]
fn hold_repair_that_merely_reduces_setup_margin_without_violating_is_fine() {
    // Guards the boundary from the other side: both before and after meet setup, so no amount
    // of margin erosion is "harm" by this rule. Documented as a deliberate choice — a policy
    // that also caps margin loss would go here.
    let before = report(1.00, 0.0, -0.05, -0.10);
    let after = report(0.01, 0.0, 0.01, 0.0);
    assert_eq!(judge(&before, &after, Check::Hold, EPS), Verdict::Keep);
}
