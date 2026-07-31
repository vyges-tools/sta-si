// SPDX-License-Identifier: Apache-2.0
//! Hold repair — G3 of the timing-driven ECO loop, end to end.
//!
//! The fixture (`examples/seq`) is a two-flop ring that *meets* hold, so the violation is
//! created the way a real design gets one: by tightening the hold requirement
//! (`hold_uncertainty`), not by mangling the netlist. That keeps the design honest and the
//! repair meaningful.
//!
//! What is being asserted is the loop's contract, not just that a function returns something:
//! a real violation goes in, a plan comes out, the plan's moves are addressable, the predicted
//! improvement is real, rejected candidates are never retried, and the whole thing terminates.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::repair::{plan_hold_repair, RepairOpts};
use vyges_sta_si::sta::{Check, Move, Timer};
use vyges_sta_si::{liberty::Lib, netlist};

const SEQ: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/");

/// Build a timer for `seq`, optionally with a hold requirement it cannot meet.
fn seq_timer(hold_uncertainty: f64) -> Timer {
    let mut job = StaJob::load(&format!("{SEQ}seq.sta")).unwrap();
    job.hold_uncertainty = hold_uncertainty;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

fn opts() -> RepairOpts {
    RepairOpts { delay_cell: "BUF".into(), ..Default::default() }
}

#[test]
fn a_met_design_yields_an_empty_plan() {
    // Nothing to do is a valid answer, and it must not invent work.
    let mut t = seq_timer(0.0);
    assert!(t.whs() > 0.0, "the fixture should meet hold before we tighten it");
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();
    assert!(plan.is_empty());
    assert!(plan.rejected.is_empty());
}

#[test]
fn a_real_hold_violation_is_repaired() {
    let mut t = seq_timer(0.25);
    let whs_before = t.whs();
    assert!(whs_before < 0.0, "tightening hold should create a violation, got {whs_before}");

    let plan = plan_hold_repair(&mut t, &opts()).unwrap();

    assert!(!plan.is_empty(), "a fixable hold violation should produce fixes");
    assert_eq!(plan.whs_before, whs_before);
    assert!(plan.whs_gain() > 0.0, "the plan must improve hold, gained {}", plan.whs_gain());
    // and the timer is left holding the repaired design
    assert!((t.whs() - plan.whs_after).abs() < 1e-12);
    assert!(t.whs() > whs_before);
}

#[test]
fn every_move_in_a_plan_is_addressable_and_uses_the_requested_cell() {
    // A plan is only useful if an applier can replay it without guessing. Each move must name a
    // real instance, a real pin on it, and the delay cell that was asked for.
    let mut t = seq_timer(0.25);
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();
    assert!(!plan.is_empty());

    let nl_before = seq_timer(0.25);
    for fix in &plan.fixes {
        let Move::InsertDelay { inst, pin, cell, name } = &fix.mv else {
            panic!("hold repair should only insert delay, got {:?}", fix.mv);
        };
        assert_eq!(cell, "BUF", "must use the cell the caller specified");
        assert!(name.starts_with("vy_hold"), "inserted cells should be identifiable: {name}");

        // the instance/pin must exist in the ORIGINAL design, so the plan is replayable
        let i = nl_before
            .netlist()
            .insts
            .iter()
            .find(|i| &i.name == inst)
            .unwrap_or_else(|| panic!("plan targets unknown instance {inst}"));
        assert!(
            i.conns.iter().any(|(p, _)| p == pin),
            "plan targets pin {pin} which {inst} does not have"
        );
        // the fix must record what it bought
        assert!(fix.whs_after > fix.whs_before, "a kept fix must have improved hold");
        assert!(fix.target.is_instance_pin());
    }
}

#[test]
fn the_repaired_netlist_actually_contains_the_inserted_cells() {
    // The plan and the timer's working netlist must agree — otherwise the predicted numbers
    // describe a design nobody has.
    let mut t = seq_timer(0.25);
    let before = t.netlist().insts.len();
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();

    assert_eq!(
        t.netlist().insts.len(),
        before + plan.fixes.len(),
        "one inserted instance per accepted fix"
    );
    for fix in &plan.fixes {
        let Move::InsertDelay { name, .. } = &fix.mv else { unreachable!() };
        let inserted = t.netlist().insts.iter().find(|i| &i.name == name).expect("cell inserted");
        assert_eq!(inserted.cell, "BUF");
        assert_eq!(inserted.conns.len(), 2, "a delay element has an input and an output");
    }
}

#[test]
fn the_price_in_setup_margin_is_reported() {
    // Adding delay to a data path costs setup margin. The plan must surface that rather than
    // report only the win — a reviewer needs both numbers to accept it.
    let mut t = seq_timer(0.25);
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();
    assert!(!plan.is_empty());
    assert!(plan.wns_cost() >= 0.0, "inserting delay should not gain setup margin");
    assert!(plan.wns_after > 0.0, "the repair must not have broken setup — judge should forbid it");
}

#[test]
fn a_fix_budget_is_respected() {
    let mut t = seq_timer(0.25);
    let plan = plan_hold_repair(&mut t, &RepairOpts { max_fixes: 1, ..opts() }).unwrap();
    assert!(plan.fixes.len() <= 1);
}

#[test]
fn an_impossible_repair_terminates_instead_of_looping() {
    // The "never loop forever" rule. With a hold requirement nothing can satisfy, every
    // candidate is either rejected or insufficient — the planner must give up, not spin
    // inserting cells until it runs out of memory.
    let mut t = seq_timer(50.0); // absurd hold requirement
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();
    // whatever it decided, it must have stopped and stayed sane
    assert!(plan.fixes.len() < 100, "planner should terminate, not insert unboundedly");
}

#[test]
fn an_unusable_delay_cell_produces_no_plan_rather_than_a_broken_one() {
    let mut t = seq_timer(0.25);
    // no cell configured
    let plan = plan_hold_repair(&mut t, &RepairOpts { delay_cell: String::new(), ..opts() }).unwrap();
    assert!(plan.is_empty());

    // a cell that does not exist in the library
    let mut t = seq_timer(0.25);
    let plan =
        plan_hold_repair(&mut t, &RepairOpts { delay_cell: "NO_SUCH_CELL".into(), ..opts() })
            .unwrap();
    assert!(plan.is_empty(), "an unknown cell must yield no fixes, not a plan that cannot apply");
}

#[test]
fn planning_leaves_no_violations_it_could_have_fixed() {
    // The loop's job is to exhaust what it can do. After planning, any remaining hold violation
    // must be one it tried and rejected — not one it simply never looked at.
    let mut t = seq_timer(0.25);
    let plan = plan_hold_repair(&mut t, &opts()).unwrap();
    let remaining = t.violations(Check::Hold, 0);
    for v in &remaining {
        assert!(
            plan.rejected.iter().any(|r| r.target.label == v.site.label) || !v.site.is_instance_pin(),
            "{} still violates but was never tried",
            v.site.label
        );
    }
}

#[test]
fn a_rejected_insertion_leaves_the_timer_completely_consistent() {
    // Regression. `restore` rolls the working netlist back, and the instance-name index MUST
    // travel with it — an index still naming a removed instance resolves to the wrong element
    // or past the end of the vector on the next lookup. The existing repair tests did not catch
    // this because they only inspected accepted fixes.
    let mut t = seq_timer(0.25);
    let before_insts: Vec<String> = t.netlist().insts.iter().map(|i| i.name.clone()).collect();

    // stage an insertion, then throw it away
    let snapshot = t.checkpoint();
    assert!(t.stage(Move::InsertDelay {
        inst: "r2".into(),
        pin: "D".into(),
        cell: "BUF".into(),
        name: "vy_rollback_probe".into(),
    }));
    t.update().unwrap();
    assert!(t.netlist().insts.iter().any(|i| i.name == "vy_rollback_probe"));
    t.restore(snapshot);

    // the netlist is back...
    let after_insts: Vec<String> = t.netlist().insts.iter().map(|i| i.name.clone()).collect();
    assert_eq!(after_insts, before_insts, "rollback must restore the netlist exactly");

    // ...and so is every lookup that depends on the index
    for p in 0..t.num_pins() {
        let s = t.pin_site(p);
        if let Some(inst) = &s.inst {
            assert!(
                before_insts.contains(inst),
                "pin {} resolved to '{inst}', which no longer exists — stale index",
                s.label
            );
        }
    }
    // and a move against a rolled-back instance must be refused, not silently applied elsewhere
    assert!(
        !t.resize("vy_rollback_probe", "BUF"),
        "a removed instance must not still be addressable"
    );
    // while a real one still is
    assert!(t.resize("g1", "INV"));
}
