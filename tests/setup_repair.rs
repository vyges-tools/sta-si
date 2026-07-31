// SPDX-License-Identifier: Apache-2.0
//! Setup repair by upsizing — the second half of G3.
//!
//! Where hold repair works from a list of failing endpoints, setup repair works from the
//! critical *path*: the fix that helps is a bigger drive on whichever **arc** costs the most,
//! and a long path is rarely uniformly slow. So the rule takes the worst path, picks its most
//! expensive stage, and tries progressively larger interchangeable cells for the instance
//! driving it.
//!
//! The violation is created by squeezing the clock period rather than by mangling the design,
//! and `eco_demo.lib` carries real drive ladders (`cell_footprint` + `function` + `area`) so
//! candidates are *derived* rather than hard-coded.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::repair::{plan_setup_repair, SetupRepairOpts};
use vyges_sta_si::sta::{Move, Timer};
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/eco_demo/");

/// A timer for `eco_demo` at `period` ns, with hold uncertainty off so setup is the story.
fn timer(period: f64) -> Timer {
    let mut job = StaJob::load(&format!("{D}eco_demo.sta")).unwrap();
    job.hold_uncertainty = 0.0;
    job.period_ns = period;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

#[test]
fn a_met_design_yields_an_empty_plan() {
    let mut t = timer(1.0);
    assert!(t.wns() > 0.0);
    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();
    assert!(plan.is_empty(), "nothing to do must mean nothing proposed");
}

#[test]
fn a_real_setup_violation_is_closed_by_upsizing() {
    let mut t = timer(0.16);
    let before = t.wns();
    assert!(before < 0.0, "0.16 ns should not be met, got WNS {before}");

    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();

    assert!(!plan.is_empty(), "a fixable setup violation should produce fixes");
    assert!(plan.wns_after > before, "WNS must improve: {before} -> {}", plan.wns_after);
    assert!(plan.wns_after >= 0.0, "this one should close outright, got {}", plan.wns_after);
    assert!((t.wns() - plan.wns_after).abs() < 1e-12, "the timer holds the repaired design");
}

#[test]
fn the_fix_is_a_resize_to_a_derived_candidate() {
    // The candidate must come from the library's equivalence class, not from a hard-coded name:
    // that is the whole point of parsing `function`/`cell_footprint`.
    let mut t = timer(0.16);
    let original = t.netlist().insts.iter().find(|i| i.name == "r2").unwrap().cell.clone();
    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();

    let fix = plan.fixes.first().expect("expected at least one fix");
    let Move::Resize { inst, cell } = &fix.mv else {
        panic!("setup repair should resize, got {:?}", fix.mv)
    };
    assert_eq!(inst, "r2", "the dominant stage on the critical path is r2's clock-to-Q");

    let lib = Lib::load(&format!("{D}eco_demo.lib")).unwrap();
    let allowed: Vec<String> =
        lib.upsize_candidates(&original).into_iter().map(|c| c.name.clone()).collect();
    assert!(allowed.contains(cell), "{cell} must be an upsize candidate of {original}: {allowed:?}");
    assert!(fix.slack_after > fix.slack_before, "a kept fix must have improved WNS");
}

#[test]
fn a_setup_fix_reports_the_hold_margin_it_spends() {
    // Upsizing makes the launch path faster, which erodes hold margin — the mirror of a hold
    // fix spending setup margin. A plan that reported only the win would be misleading.
    let mut t = timer(0.16);
    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();
    assert!(!plan.is_empty());
    assert!(
        plan.whs_after < plan.whs_before,
        "a faster clock-to-Q should cost hold margin: {} -> {}",
        plan.whs_before,
        plan.whs_after
    );
    assert!(plan.whs_after > 0.0, "but it must not break hold — judge should forbid that");
}

#[test]
fn a_library_without_equivalence_data_declines_rather_than_guessing() {
    // seq.lib has neither `function` nor `cell_footprint`, so no cell can be shown
    // interchangeable with any other. A repair that cannot prove a replacement is safe must
    // make none — swapping on a guess changes what the design computes.
    let mut job = StaJob::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/seq.sta")).unwrap();
    job.period_ns = 0.16;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
    assert!(t.wns() < 0.0, "the squeezed seq design should violate setup");

    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();
    assert!(plan.is_empty(), "no derivable candidates must mean no fixes");
    assert!(!plan.rejected.is_empty(), "and it should say it tried and could not");
}

#[test]
fn a_fix_budget_is_respected() {
    let mut t = timer(0.13);
    let plan =
        plan_setup_repair(&mut t, &SetupRepairOpts { max_fixes: 1, ..Default::default() }).unwrap();
    assert!(plan.fixes.len() <= 1);
}

#[test]
fn an_unfixable_violation_terminates_instead_of_looping() {
    // A period nothing in the library can meet. Every site is tried, every candidate judged and
    // rejected, and the planner gives up — it must not keep re-proposing the same resize.
    let mut t = timer(0.02);
    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();
    assert!(plan.fixes.len() < 50, "planner must terminate, not resize unboundedly");
    assert!(t.wns() < 0.0, "and it should not claim to have fixed it");
}
