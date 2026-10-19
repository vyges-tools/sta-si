// SPDX-License-Identifier: Apache-2.0
//! A check pairs the clock EDGE that launched the data with the edge that captures it.
//!
//! **The rule (the reference's cycle accounting, `CycleAccting::findDelays`).** For a launching
//! edge and a capturing edge, setup is the time from the launch to the first capture edge
//! strictly after it, and hold the time from the last capture edge at or before the launch. One
//! clock of period T, edges at 0 (rise) and T/2 (fall):
//!
//! | launch | capture | setup | hold edge |
//! |--------|---------|-------|-----------|
//! | rise   | rise    | T     | 0         |
//! | fall   | rise    | T/2   | −T/2      |
//! | rise   | fall    | T/2   | −T/2      |
//!
//! The capture edge is the check's own (`setup_rising` / `setup_falling`); the launch edge is
//! the register's CK->Q arc (`rising_edge` / `falling_edge`). A design whose registers launch on
//! both edges is timed once per launching edge, as the reference keeps one arrival per edge.
//!
//! **Witness.** `examples/fall_launch`, the reference at its pin on `ends.witness`
//! (`instruments/sta-si/tcl-witness.sh`): n1 (fall) → r2 (rise) setup 0.8000, hold 1.1300;
//! r3 (rise) → n4 (fall) setup 0.7500, hold 1.1800; r5, reached from both, setup 0.7400 (n1) and
//! hold 0.2400 (r3). Before this rule the setups read as a whole period (1.8000, 1.7500, 1.7000). Found on a dual-port SRAM macro whose read data is clocked out on
//! the falling edge: its paths were half a period optimistic.
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::sta::Timer;

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/fall_launch/");

fn timer() -> Timer {
    let job = vyges_sta_si::tcl::adapt(&format!("{D}fall_launch.tcl")).unwrap().job;
    let nl = vyges_sta_si::netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

fn setup(t: &Timer, pin: &str) -> f64 {
    t.slack(t.pin(pin).unwrap()).unwrap()
}

fn hold(t: &Timer, pin: &str) -> f64 {
    let p = t.pin(pin).unwrap();
    t.report().hold_slacks.iter().find(|&&(i, _)| i == p).unwrap().1
}

#[test]
fn a_falling_launch_is_captured_half_a_period_later_on_the_rise() {
    let t = timer();
    assert!((setup(&t, "r2/D") - 0.8).abs() < 1e-6, "setup at r2/D: {}", setup(&t, "r2/D"));
    assert!((hold(&t, "r2/D") - 1.13).abs() < 1e-6, "hold at r2/D: {}", hold(&t, "r2/D"));
}

#[test]
fn a_falling_capture_is_half_a_period_after_a_rising_launch() {
    let t = timer();
    assert!((setup(&t, "n4/D") - 0.75).abs() < 1e-6, "setup at n4/D: {}", setup(&t, "n4/D"));
    assert!((hold(&t, "n4/D") - 1.18).abs() < 1e-6, "hold at n4/D: {}", hold(&t, "n4/D"));
}

/// One register reached from BOTH edges: r5's setup is set by n1's falling launch (0.7400) and
/// its hold by r3's rising one (0.2400). Measured from its own launch edge the rising path is
/// the later (0.25 against 0.21), so one merged arrival keeps it, is checked a whole period out,
/// and reads setup 1.7000 — the falling launch is timed only where each edge is timed alone.
#[test]
fn each_launching_edge_is_checked_on_its_own() {
    let t = timer();
    assert!((setup(&t, "r5/D") - 0.74).abs() < 1e-6, "setup at r5/D: {}", setup(&t, "r5/D"));
    assert!((hold(&t, "r5/D") - 0.24).abs() < 1e-6, "hold at r5/D: {}", hold(&t, "r5/D"));
}

/// The design's worst setup and hold are the reference's (library values are read as `float`,
/// as the reference reads them: 0.05 is 0.0500000007).
#[test]
fn the_worst_checks_are_the_references() {
    let t = timer();
    let r = t.report();
    assert!((r.wns - 0.74).abs() < 1e-6, "wns {}", r.wns);
    assert_eq!(r.worst_endpoint, "r5/D");
    assert!((r.whs - 0.24).abs() < 1e-6, "whs {}", r.whs);
}

/// The fixture asks the question only if both registers really are opposite-edge.
#[test]
fn the_fixture_has_one_register_of_each_edge() {
    let lib = Lib::load(&format!("{D}fall_launch.lib")).unwrap();
    let edge = |c: &str| lib.cells[c].pins["Q"].arcs[0].clk_edge;
    assert_ne!(edge("DFF"), edge("DFFN"));
}
