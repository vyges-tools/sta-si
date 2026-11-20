// SPDX-License-Identifier: Apache-2.0
//! A clock path keeps the clock EDGE it started from while a non-unate arc flips its transition.
//!
//! **The rule (the reference's clock tags: `ClkInfo::clkEdge`, `Search::thruClkInfo`).** A clock
//! source's rising transition carries the clock's rise edge and its falling transition the fall
//! edge. Through a wire or a combinational arc the transition follows the arc's sense and the
//! edge does not change. A non-unate arc drives both output transitions from each input
//! transition, so past one both edges reach both transitions of every downstream pin, and the
//! reference keeps one arrival per (transition, edge): a separate tag. Each tag is its own launch
//! (a falling-edge register whose clock pin falls on the clock's RISE edge launches at time 0),
//! its own capture (the check runs against each tag on the check's transition, and the worst
//! counts), and its own CRPR clock path (`CheckCrpr::findCrpr` walks each tag's `prevPath`).
//! Merging the two edges into one arrival per transition, as we did, lost the other edge's
//! launches and captures: half a period on every check through such an arc.
//!
//! **Witness.** `examples/nonunate_clk`: a positive-unate clock buffer (rises slowly, falls fast)
//! then a non-unate one (slew-dependent delays), so each edge's tag also arrives at its own time.
//! The reference (standalone OpenSTA 4af3c130) on each end alone. Before this rule, 4 of 8 ideal
//! and 1 of 8 propagated slacks matched (the misses by a whole half period, ideal; by half a
//! period or by the other edge's clock arrival, propagated).
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/nonunate_clk/");

fn timer(job_file: &str) -> Timer {
    let job = StaJob::load(&format!("{D}{job_file}")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
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

/// (end, setup, hold) as the reference reports each end
fn check(job: &str, want: &[(&str, f64, f64)]) {
    let t = timer(job);
    for &(end, s, h) in want {
        assert!((setup(&t, end) - s).abs() < 1e-4, "{job} setup at {end}: {} want {s}", setup(&t, end));
        assert!((hold(&t, end) - h).abs() < 1e-4, "{job} hold at {end}: {} want {h}", hold(&t, end));
    }
}

/// Ideal clock: no network delay, so only the edges differ. n2 (falling-edge) launches on the rise
/// edge at 0 (y hold -0.9801: the merged model launched it at the fall, +0.0199) and is captured
/// by it (n2/D hold 0.1800, not 1.1800); r3 (rising-edge) launches on the fall edge at T/2
/// (y3 setup 0.5898, not 1.5898) and is captured by it (r3/D hold 0.1467, not 1.1467).
#[test]
fn each_clock_edge_through_a_non_unate_arc_launches_and_captures() {
    check(
        "nonunate_clk.sta",
        &[("n2/D", 0.7500, 0.1800), ("r3/D", 0.7833, 0.1467), ("y", 0.5656, -0.9801), ("y3", 0.5898, -1.0043)],
    );
}

/// Propagated clock: the rise edge's falling ck2 is cb0's slow rise into cb1 (0.30 before derate),
/// the fall edge's is cb0's fast fall into cb1 (0.16). y's setup is launched by the fall edge and
/// takes ITS arrival (1.4857), not the later of the two (1.3317); r3/D's setup and hold pair
/// tags whose CRPR spreads are each tag's own.
#[test]
fn each_clock_edge_takes_its_own_clock_path() {
    check(
        "nonunate_clk_prop.sta",
        &[("n2/D", 1.3940, -0.6500), ("r3/D", 0.7911, -0.0507), ("y", 1.4857, -0.7742), ("y3", 1.3546, -0.6662)],
    );
}
