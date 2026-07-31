// SPDX-License-Identifier: Apache-2.0
//! Differential verification of the incremental timing path.
//!
//! The incremental path exists to be fast; its licence to exist is that it is **indistinguishable
//! from a full re-analysis**. That is checked here exhaustively rather than sampled — the
//! fixtures are small enough that every instance × every interchangeable cell is a tractable
//! matrix, so "we tested the cases we thought of" does not arise.
//!
//! Two things are asserted, and the second matters as much as the first:
//!
//! 1. Every edit the fast path **accepts** produces exactly the full analysis's answer.
//! 2. Edits it is supposed to **refuse** are still refused. Widening what the path claims to
//!    understand is precisely how its safety property erodes, so the refusals are pinned too.
mod common;

use common::{check_edit, load, resize_moves, Served};
use vyges_sta_si::sta::{Move, Timer};

const ECO: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/eco_demo/eco_demo.sta");
const TOP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
const SEQ: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/seq.sta");

#[test]
fn every_resize_the_fast_path_accepts_matches_a_full_re_analysis() {
    // The exhaustive matrix: each instance, each cell it could legally become.
    let (job, nl, lib) = load(ECO);
    let insts: Vec<String> = nl.insts.iter().map(|i| i.name.clone()).collect();

    let mut accepted = 0;
    let mut refused = 0;
    for inst in &insts {
        let probe = Timer::build(&nl, &lib, &job, None).unwrap();
        for mv in resize_moves(&probe, inst) {
            let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
            let checked = check_edit(&mut t, &job, &lib, mv.clone());
            assert!(!checked.not_applicable, "{mv:?} should apply");
            assert!(
                checked.agrees(),
                "incremental and full disagree for {mv:?}:\n  {}",
                checked.mismatches.join("\n  ")
            );
            match checked.served {
                Served::Incremental => accepted += 1,
                Served::Full => refused += 1,
            }
        }
    }
    assert!(accepted > 0, "the matrix should exercise the fast path at least once");
    // both outcomes are legitimate; what is not legitimate is disagreeing
    eprintln!("resize matrix: {accepted} served incrementally, {refused} fell back");
}

#[test]
fn a_delay_insertion_matches_a_full_re_analysis() {
    // Insertion changes topology, so it is expected to fall back — but the result still has to
    // be right, and the fallback still has to happen rather than the graph being reused stale.
    let (job, nl, lib) = load(ECO);
    for (inst, pin) in [("r1", "D"), ("r2", "D")] {
        let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
        let mv = Move::InsertDelay {
            inst: inst.into(),
            pin: pin.into(),
            cell: "BUF".into(),
            name: format!("diff_{inst}"),
        };
        let checked = check_edit(&mut t, &job, &lib, mv.clone());
        assert!(!checked.not_applicable);
        assert!(
            checked.agrees(),
            "insertion at {inst}/{pin} disagrees:\n  {}",
            checked.mismatches.join("\n  ")
        );
        assert_eq!(
            checked.served,
            Served::Full,
            "a topology change must not be served from the cone graph"
        );
    }
}

#[test]
fn a_sequential_resize_is_refused_by_the_fast_path() {
    // Pinned deliberately. Resizing a flop changes its CK-pin capacitance, which perturbs clock
    // arrival at every other flop on that net and shifts CRPR credits — not a cone-local edit.
    // If this test starts failing, the incremental path has widened what it claims to
    // understand, and that widening needs the argument in
    // docs/loom/incremental-timing-sequential-cells.md, not a silently updated assertion.
    let (job, nl, lib) = load(ECO);
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
    let checked =
        check_edit(&mut t, &job, &lib, Move::Resize { inst: "r2".into(), cell: "DFF_X2".into() });

    assert!(!checked.not_applicable);
    assert_eq!(checked.served, Served::Full, "a flop resize must fall back");
    assert!(
        checked.agrees(),
        "and the fallback must still be correct:\n  {}",
        checked.mismatches.join("\n  ")
    );
}

#[test]
fn repeated_edits_do_not_drift_from_a_full_re_analysis() {
    // A repair run applies many edits in sequence against one Timer. Each incremental update
    // starts from the previous one's state, so an error would compound rather than announce
    // itself — this walks a chain and re-checks against a full analysis at every step.
    let (job, nl, lib) = load(ECO);
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();

    let chain = [
        Move::Resize { inst: "g1".into(), cell: "INV_X2".into() },
        Move::Resize { inst: "g1".into(), cell: "INV_X4".into() },
        Move::InsertDelay {
            inst: "r2".into(),
            pin: "D".into(),
            cell: "BUF".into(),
            name: "chain0".into(),
        },
        Move::Resize { inst: "g1".into(), cell: "INV_X4_HVT".into() },
        Move::Resize { inst: "g1".into(), cell: "INV".into() },
    ];
    for (i, mv) in chain.into_iter().enumerate() {
        let checked = check_edit(&mut t, &job, &lib, mv.clone());
        assert!(!checked.not_applicable, "step {i}: {mv:?} should apply");
        assert!(
            checked.agrees(),
            "step {i} ({mv:?}) drifted from a full re-analysis:\n  {}",
            checked.mismatches.join("\n  ")
        );
    }
}

#[test]
fn a_rolled_back_edit_leaves_the_timer_agreeing_with_a_full_re_analysis() {
    // Speculation is the loop's core move: apply, judge, put back. A restore that left the
    // cached state subtly wrong would poison every later decision, and nothing downstream would
    // notice — the numbers would just be wrong.
    let (job, nl, lib) = load(ECO);
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();

    let snapshot = t.checkpoint();
    let checked =
        check_edit(&mut t, &job, &lib, Move::Resize { inst: "g1".into(), cell: "INV_X4".into() });
    assert!(checked.agrees());
    t.restore(snapshot);

    // after rolling back, the timer must agree with a full analysis of the ORIGINAL netlist.
    // A rollback restores cached state wholesale rather than recomputing, so unlike an
    // incremental update this really should be bit-identical — asserted as such.
    let reference = Timer::build(&nl, &lib, &job, None).unwrap();
    assert_eq!(t.wns().to_bits(), reference.wns().to_bits(), "WNS drifted across a rollback");
    assert_eq!(t.whs().to_bits(), reference.whs().to_bits(), "WHS drifted across a rollback");
    for p in 0..t.num_pins() {
        let label = t.pin_label(p).to_string();
        let q = reference.pin(&label).expect("pin should survive a rollback");
        assert_eq!(t.arrival(p).to_bits(), reference.arrival(q).to_bits(), "{label} arrival");
        assert_eq!(t.load(p).to_bits(), reference.load(q).to_bits(), "{label} load");
    }
}

#[test]
fn the_harness_also_covers_the_combinational_only_fixture() {
    // `top` has no flops at all, so every edit is squarely in the fast path's home territory.
    // Cheap breadth against the part of the engine the repair rules lean on hardest.
    let (job, nl, lib) = load(TOP);
    let insts: Vec<String> = nl.insts.iter().map(|i| i.name.clone()).collect();
    for inst in &insts {
        let probe = Timer::build(&nl, &lib, &job, None).unwrap();
        for mv in resize_moves(&probe, inst) {
            let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
            let checked = check_edit(&mut t, &job, &lib, mv.clone());
            assert!(
                checked.agrees(),
                "top: {mv:?} disagrees:\n  {}",
                checked.mismatches.join("\n  ")
            );
        }
    }
}

#[test]
fn a_library_without_equivalence_data_yields_no_resize_moves_to_check() {
    // Guards the harness itself: `seq.lib` has no `function` or `cell_footprint`, so the matrix
    // is legitimately empty there. Without this, a bug that silently emptied the matrix would
    // make the exhaustive tests above pass by testing nothing.
    let (job, nl, lib) = load(SEQ);
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    for inst in nl.insts.iter().map(|i| &i.name) {
        assert!(resize_moves(&t, inst).is_empty(), "seq.lib should offer no equivalents");
    }
}

// ---- Tier A: CK-load-preserving sequential resizes ------------------------------------------

#[test]
fn a_ck_load_preserving_flop_swap_is_served_incrementally_and_is_correct() {
    // Tier A. `DFF_HVT` has the same CK capacitance as `DFF`, so the clock driver sees an
    // identical load and every clock arrival, slew and CRPR credit is unchanged BY
    // CONSTRUCTION. That leaves only data-side effects, which are cone-local.
    //
    // The correctness half matters more than the speed half: this swap also changes the flop's
    // setup and hold CONSTRAINT tables, which live in the immutable topology. Without the
    // override that ships with this, the fast path would return a plausible, wrong slack — and
    // this assertion is the only thing standing between that and a released number.
    let (job, nl, lib) = load(ECO);
    for inst in ["r1", "r2"] {
        let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
        let checked = check_edit(
            &mut t,
            &job,
            &lib,
            Move::Resize { inst: inst.into(), cell: "DFF_HVT".into() },
        );
        assert!(!checked.not_applicable);
        assert_eq!(
            checked.served,
            Served::Incremental,
            "{inst} -> DFF_HVT preserves CK load and should take the fast path"
        );
        assert!(
            checked.agrees(),
            "{inst} -> DFF_HVT disagrees with a full re-analysis:\n  {}",
            checked.mismatches.join("\n  ")
        );
    }
}

#[test]
fn the_constraint_tables_really_do_change_so_the_override_is_load_bearing() {
    // Guards the guard. If DFF and DFF_HVT happened to share setup/hold constraints, the test
    // above would pass whether or not the override worked, and would quietly stop protecting
    // anything. Assert the fixture actually distinguishes them.
    let (_, _, lib) = load(ECO);
    let base = lib.cells.get("DFF").expect("DFF");
    let hvt = lib.cells.get("DFF_HVT").expect("DFF_HVT");
    assert!(!base.pins["D"].setup.is_empty(), "the fixture flop should carry setup constraints");
    // Constraint has no PartialEq, so compare what actually matters: the requirement each cell
    // evaluates to at a representative operating point.
    let ev = |cs: &[vyges_sta_si::liberty::Constraint]| -> f64 {
        cs.iter().map(|c| c.eval(0.01, 0.01)).fold(f64::NEG_INFINITY, f64::max)
    };
    let differs = ev(&base.pins["D"].setup) != ev(&hvt.pins["D"].setup)
        || ev(&base.pins["D"].hold) != ev(&hvt.pins["D"].hold);
    assert!(differs, "DFF_HVT must differ from DFF in its constraints, or the override is untested");
}

#[test]
fn a_swap_that_changes_ck_load_is_still_refused() {
    // The Tier A / Tier B boundary, pinned from the other side. `DFF_X2` differs from `DFF` in
    // CK capacitance, so it perturbs the clock network and must keep falling back — widening to
    // cover it needs incremental clock arrival and CRPR, which is a separate argument.
    let (_, _, lib) = load(ECO);
    let (a, b) = (lib.cells.get("DFF").unwrap(), lib.cells.get("DFF_X2").unwrap());
    assert_ne!(
        a.pins["CK"].capacitance, b.pins["CK"].capacitance,
        "the fixture's drive-strength flop pair must differ in CK load for this to mean anything"
    );

    let (job, nl, lib2) = load(ECO);
    let mut t = Timer::build(&nl, &lib2, &job, None).unwrap();
    let checked =
        check_edit(&mut t, &job, &lib2, Move::Resize { inst: "r2".into(), cell: "DFF_X2".into() });
    assert_eq!(checked.served, Served::Full, "a CK-load change must not take the fast path");
    assert!(checked.agrees());
}
