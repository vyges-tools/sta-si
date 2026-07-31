// SPDX-License-Identifier: Apache-2.0
//! Recovery and removal — the asynchronous set/reset checks.
//!
//! An async reset pin carries neither `setup` nor `hold`, so before these were applied it
//! got **no check at all** and the design was signed off with that path unexamined. On a
//! real sky130 block the reset pins are roughly a fifth of the hold check set, so this is
//! not a rounding error.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/async_rst/");

fn timer() -> Timer {
    let job = StaJob::load(&format!("{D}async_rst.sta")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

#[test]
fn an_async_reset_pin_is_checked_at_all() {
    let t = timer();
    let rb = t.pin("r1/RESET_B").expect("the reset pin should be a timed node");
    // it must appear as a hold (removal) endpoint — before this it appeared nowhere
    let hold: Vec<_> = t.report().hold_slacks.iter().filter(|(i, _)| *i == rb).collect();
    assert_eq!(hold.len(), 1, "RESET_B should carry exactly one removal check");
    assert!(t.is_endpoint(rb), "and a recovery (setup-side) endpoint too");
}

#[test]
fn removal_uses_the_removal_table_not_the_data_hold_table() {
    // The whole point: these are different constraints with different numbers. If the
    // fixture gave them the same value this test would pass while checking nothing, so
    // assert the library really distinguishes them.
    let job = StaJob::load(&format!("{D}async_rst.sta")).unwrap();
    let lib = Lib::load(&job.resolve(&job.libs[0])).unwrap();
    let cell = &lib.cells["DFRTP"];
    let d_hold = cell.pins["D"].hold[0].eval(0.01, 0.01);
    let rb_removal = cell.pins["RESET_B"].removal[0].eval(0.01, 0.01);
    assert_ne!(d_hold, rb_removal, "fixture must distinguish hold from removal");

    let t = timer();
    let rb = t.pin("r1/RESET_B").unwrap();
    let d = t.pin("r1/D").unwrap();
    let slack_of = |i| t.report().hold_slacks.iter().find(|(p, _)| *p == i).map(|(_, s)| *s);
    let (sr, sd) = (slack_of(rb).unwrap(), slack_of(d).unwrap());
    // same clock, same launch: the slack difference is exactly the constraint difference
    assert!(
        ((sd - sr) - (rb_removal - d_hold)).abs() < 1e-9,
        "removal slack should differ from hold slack by exactly the table difference: \
         hold {d_hold}, removal {rb_removal}, slacks {sd} vs {sr}"
    );
}

#[test]
fn a_flop_without_async_pins_gains_no_extra_checks() {
    // Guards against the async path quietly adding endpoints to ordinary flops.
    let job = StaJob::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/seq.sta")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    let flops = nl.insts.iter().filter(|i| lib.cells.get(&i.cell).is_some_and(|c| c.is_seq)).count();
    assert_eq!(
        t.report().hold_endpoints,
        flops,
        "a plain DFF library should give exactly one hold check per flop"
    );
}
