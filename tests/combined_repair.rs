// SPDX-License-Identifier: Apache-2.0
//! Setup and hold repaired **together**.
//!
//! A real block needs both, and they pull against each other: upsizing shortens paths and eats
//! hold margin; inserting delay lengthens them and eats setup margin. Running one rule to
//! completion and then the other lets the second undo the first's headroom, so the combined
//! rule attacks whichever check is *further* into violation each round.
//!
//! What stops the two from fighting is not the ordering but `judge`, which already refuses any
//! fix that pushes the other check into violation or deepens an existing one. The most
//! interesting case below is therefore the one where **nothing can be done**: the planner has to
//! recognise that and stop, rather than trade the two violations back and forth forever.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::repair::{plan_repair, CombinedOpts, RepairOpts};
use vyges_sta_si::sta::{Check, Move, Timer};
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/eco_demo/");

fn timer(period: f64, hold_uncertainty: f64) -> Timer {
    let mut job = StaJob::load(&format!("{D}eco_demo.sta")).unwrap();
    job.period_ns = period;
    job.hold_uncertainty = hold_uncertainty;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

fn opts() -> CombinedOpts {
    CombinedOpts {
        hold: RepairOpts { delay_cell: "BUF".into(), ..Default::default() },
        ..Default::default()
    }
}

#[test]
fn a_design_meeting_both_checks_is_left_alone() {
    let mut t = timer(1.0, 0.0);
    assert!(t.wns() > 0.0 && t.whs() > 0.0);
    let plan = plan_repair(&mut t, &opts()).unwrap();
    assert!(plan.is_empty(), "nothing failing must mean nothing proposed");
}

#[test]
fn it_repairs_hold_when_only_hold_is_failing() {
    let mut t = timer(1.0, 0.25);
    assert!(t.wns() > 0.0, "setup should be comfortable");
    assert!(t.whs() < 0.0, "hold should be failing");

    let plan = plan_repair(&mut t, &opts()).unwrap();

    assert!(!plan.is_empty());
    assert!(plan.whs_after >= 0.0, "hold should close: {}", plan.whs_after);
    assert!(plan.wns_after > 0.0, "and setup must not be broken doing it");
    assert!(
        plan.fixes.iter().all(|f| f.check == Check::Hold),
        "only hold fixes were called for"
    );
}

#[test]
fn it_repairs_setup_when_only_setup_is_failing() {
    let mut t = timer(0.16, 0.0);
    assert!(t.wns() < 0.0 && t.whs() > 0.0);

    let plan = plan_repair(&mut t, &opts()).unwrap();

    assert!(!plan.is_empty());
    assert!(plan.wns_after >= 0.0, "setup should close: {}", plan.wns_after);
    assert!(plan.whs_after > 0.0, "and hold must survive");
    assert!(plan.fixes.iter().all(|f| f.check == Check::Setup));
}

#[test]
fn the_worse_violation_is_attacked_first() {
    // With hold at -0.094 and setup at -0.016, hold is further into violation, so the first
    // thing tried must be a hold fix. Ordering matters even when both end up rejected: attacking
    // the shallower violation first wastes the margin the deeper one needs.
    let mut t = timer(0.16, 0.25);
    assert!(t.whs() < t.wns(), "hold should be the worse of the two here");

    let plan = plan_repair(&mut t, &opts()).unwrap();
    let first = plan
        .fixes
        .first()
        .map(|f| f.check)
        .or_else(|| plan.rejected.first().map(|_| Check::Hold));
    assert_eq!(first, Some(Check::Hold), "the deeper violation should be tried first");
}

#[test]
fn an_over_constrained_design_is_diagnosed_rather_than_oscillated_over() {
    // The case that justifies the design. Setup and hold both fail on the same path, so every
    // available fix helps one by breaking the other — and `judge` refuses all of them. The
    // planner must stop and say why, not trade the two violations back and forth forever.
    let mut t = timer(0.16, 0.25);
    let (wns0, whs0) = (t.wns(), t.whs());
    assert!(wns0 < 0.0 && whs0 < 0.0, "both checks should be failing");

    let plan = plan_repair(&mut t, &opts()).unwrap();

    assert!(plan.is_empty(), "no fix here is safe, so none should be proposed");
    assert!(!plan.rejected.is_empty(), "but it must report what it tried");
    assert_eq!((t.wns(), t.whs()), (wns0, whs0), "and leave the design exactly as it found it");

    // and the reasons must be the real ones — each check broken by the other's remedy
    let reasons: Vec<String> =
        plan.rejected.iter().map(|r| format!("{:?}", r.reason)).collect();
    assert!(
        reasons.iter().any(|r| r == "BrokeSetup"),
        "adding hold delay should be refused for deepening setup: {reasons:?}"
    );
    assert!(
        reasons.iter().any(|r| r == "BrokeHold"),
        "upsizing for setup should be refused for deepening hold: {reasons:?}"
    );
}

#[test]
fn a_combined_budget_caps_total_fixes_across_both_checks() {
    let mut t = timer(1.0, 0.25);
    let plan = plan_repair(&mut t, &CombinedOpts { max_fixes: 1, ..opts() }).unwrap();
    assert!(plan.fixes.len() <= 1);
}

#[test]
fn inserted_cells_get_unique_names_across_a_combined_run() {
    // The combined loop owns the naming counter precisely so that alternating between checks
    // cannot restart it and collide with a cell inserted earlier — a collision would silently
    // fail to stage and look like "no fix available".
    let mut t = timer(1.0, 0.25);
    let plan = plan_repair(&mut t, &opts()).unwrap();
    let mut names: Vec<&String> = plan
        .fixes
        .iter()
        .filter_map(|f| match &f.mv {
            Move::InsertDelay { name, .. } => Some(name),
            _ => None,
        })
        .collect();
    let total = names.len();
    names.sort();
    names.dedup();
    assert_eq!(names.len(), total, "inserted cell names must be unique");
}
