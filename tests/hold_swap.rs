// SPDX-License-Identifier: Apache-2.0
//! Hold repair by **swapping the driver for a slower cell**, rather than inserting delay.
//!
//! Inserting a delay cell always works but always costs: a new instance, area, and a cell that
//! lands on top of its neighbour until the placer sorts it out. Swapping the driver for a
//! slower interchangeable cell costs none of that — the cell keeps its own site.
//!
//! The name says "slower cell", not "smaller cell", and that distinction is the whole point:
//! a plain downsize also presents **less input capacitance**, which speeds up the stage
//! *before* it — sometimes by more than the weaker cell slows this one, making hold worse. The
//! reliable version is a Vt swap: same size, same load, slower. Both are cells in the same
//! equivalence class, so both get tried and `judge` decides.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::repair::{plan_hold_repair, RepairOpts};
use vyges_sta_si::sta::{judge, Check, Move, Timer, Verdict};
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/eco_demo/");

/// `eco_demo` with an over-strong driver on the hold-critical path — the case a swap exists for.
fn timer_with_strong_driver(hold_uncertainty: f64) -> Timer {
    let mut job = StaJob::load(&format!("{D}eco_demo.sta")).unwrap();
    job.hold_uncertainty = hold_uncertainty;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
    t.resize("g1", "INV_X4");
    t.update().unwrap();
    t
}

fn opts(prefer_swap: bool) -> RepairOpts {
    RepairOpts { delay_cell: "BUF".into(), prefer_swap, ..Default::default() }
}

#[test]
fn a_hold_fix_can_be_a_cell_swap_instead_of_an_insertion() {
    let mut t = timer_with_strong_driver(0.20);
    assert!(t.whs() < 0.0);

    let plan = plan_hold_repair(&mut t, &opts(true)).unwrap();

    assert!(plan.whs_after >= 0.0, "hold should close: {}", plan.whs_after);
    let swaps: Vec<&Move> =
        plan.fixes.iter().map(|f| &f.mv).filter(|m| matches!(m, Move::Resize { .. })).collect();
    assert!(!swaps.is_empty(), "at least one fix should be a swap, got {:?}", plan.fixes);
    // and the swap must target the DRIVER of the failing endpoint, not the endpoint itself
    let Move::Resize { inst, cell } = swaps[0] else { unreachable!() };
    assert_eq!(inst, "g1", "g1 drives the failing endpoint's net");
    assert_ne!(cell, "INV_X4", "the identity swap is not a fix");
}

#[test]
fn swapping_closes_hold_with_fewer_instances_than_inserting() {
    // The reason to prefer it: same result, less design disturbance.
    let mut swapped = timer_with_strong_driver(0.20);
    let plan_swap = plan_hold_repair(&mut swapped, &opts(true)).unwrap();

    let mut inserted = timer_with_strong_driver(0.20);
    let plan_insert = plan_hold_repair(&mut inserted, &opts(false)).unwrap();

    assert!(plan_swap.whs_after >= 0.0 && plan_insert.whs_after >= 0.0, "both must close hold");
    assert!(
        swapped.netlist().insts.len() < inserted.netlist().insts.len(),
        "swapping should add fewer cells: {} vs {}",
        swapped.netlist().insts.len(),
        inserted.netlist().insts.len()
    );
}

#[test]
fn disabling_the_swap_falls_back_to_insertion_only() {
    let mut t = timer_with_strong_driver(0.20);
    let plan = plan_hold_repair(&mut t, &opts(false)).unwrap();
    assert!(plan.whs_after >= 0.0, "insertion alone must still close hold");
    assert!(
        plan.fixes.iter().all(|f| matches!(f.mv, Move::InsertDelay { .. })),
        "no swaps should appear when the option is off"
    );
}

#[test]
fn insertion_is_still_the_fallback_when_no_slower_cell_exists() {
    // The first failing endpoint here is driven by a flop whose only sibling is FASTER, so no
    // swap can help and the rule must still fix it — by inserting.
    let mut t = timer_with_strong_driver(0.20);
    let plan = plan_hold_repair(&mut t, &opts(true)).unwrap();
    assert!(
        plan.fixes.iter().any(|f| matches!(f.mv, Move::InsertDelay { .. })),
        "the endpoint with no slower driver must still be repaired by insertion"
    );
}

#[test]
fn a_plain_downsize_can_make_hold_worse_which_is_why_candidates_are_judged() {
    // The finding that shaped this feature, pinned as a test. Downsizing g1 from INV_X4 lightens
    // the load on r1, speeding the launch path up by more than the weaker cell slows it down —
    // so hold gets WORSE. A rule that assumed "smaller means slower" would ship this as a fix.
    let mut t = timer_with_strong_driver(0.20);
    // reach the state where g1's endpoint is the worst one
    t.stage(Move::InsertDelay {
        inst: "r1".into(),
        pin: "D".into(),
        cell: "BUF".into(),
        name: "probe0".into(),
    });
    t.update().unwrap();

    let before = t.report().clone();
    t.resize("g1", "INV_X2"); // strictly smaller
    t.update().unwrap();
    let after = t.report().clone();

    assert!(
        after.whs < before.whs,
        "downsizing was expected to make hold worse here: {} -> {}",
        before.whs,
        after.whs
    );
    assert!(
        matches!(judge(&before, &after, Check::Hold, 1e-9), Verdict::Revert(_)),
        "and judge must reject it"
    );
}

#[test]
fn a_library_without_equivalence_data_just_inserts() {
    // seq.lib carries neither `function` nor `cell_footprint`, so nothing is interchangeable and
    // the swap path can never fire. It must degrade to insertion, not fail.
    let mut job =
        StaJob::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/seq.sta")).unwrap();
    job.hold_uncertainty = 0.25;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();

    let plan = plan_hold_repair(&mut t, &opts(true)).unwrap();
    assert!(!plan.is_empty(), "it should still repair");
    assert!(
        plan.fixes.iter().all(|f| matches!(f.mv, Move::InsertDelay { .. })),
        "with no derivable equivalents, every fix must be an insertion"
    );
}
