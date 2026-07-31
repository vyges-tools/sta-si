// SPDX-License-Identifier: Apache-2.0
//! Constant (tie-cell) propagation.
//!
//! A net driven by a tie cell (`function : "1"` / `"0"`) can never switch, so it carries
//! no timing: nothing to launch, nothing to check. Treating the tie output as an ordinary
//! undriven node instead makes it a path source at t=0, which **manufactures hold
//! violations on wires that cannot toggle** — that was the sole cause of the only hold
//! violation sta-si reported on a block sign-off says is clean.
//!
//! The second test matters as much as the first: making constants untimed must not make
//! a gate with *one* constant input untimed as well.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/tie/");

fn timer() -> Timer {
    let job = StaJob::load(&format!("{D}tie.sta")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

#[test]
fn a_flop_fed_by_a_tie_cell_is_not_hold_checked() {
    let t = timer();
    let tied = t.pin("r_tied/D").expect("the pin still exists as a node");
    assert!(
        !t.report().hold_slacks.iter().any(|(i, _)| *i == tied),
        "a pin that can never switch must carry no hold check"
    );
    assert!(t.arrival(tied).is_infinite(), "and must be unreached, not seeded at t=0");
}

#[test]
fn a_gate_with_one_constant_input_still_times_through_the_other() {
    // The regression this could easily have introduced. g0.B is tied high; g0.A is real.
    // If constants made the whole gate untimed, the flop behind it would silently lose
    // its check — trading a false violation for a missing one, which is worse.
    let t = timer();
    let a = t.pin("g0/A").expect("g0/A");
    let x = t.pin("g0/X").expect("g0/X");
    let b = t.pin("g0/B").expect("g0/B");
    assert!(t.arrival(a).is_finite(), "the real input must be timed");
    assert!(t.arrival(b).is_infinite(), "the tied input must not be");
    assert!(t.arrival(x).is_finite(), "and the gate output must still be timed");
    assert!(t.arrival(x) > t.arrival(a), "through the real input's arc");

    let real = t.pin("r1/D").expect("r1/D");
    assert!(
        t.report().hold_slacks.iter().any(|(i, _)| *i == real),
        "the flop behind a partially-constant gate must KEEP its hold check"
    );
}

#[test]
fn the_tie_cell_does_not_become_the_worst_hold_endpoint() {
    // The end-to-end shape of the real-block bug: a tie net seeded at t=0 launched
    // nothing yet dominated WHS. This fixture does carry a genuine, marginal hold
    // violation on the REAL path — which is the useful control: it shows ordinary hold
    // checks still fire, so a passing test here cannot mean "hold checking is off".
    let t = timer();
    let worst = t.report().worst_hold_endpoint.clone();
    assert!(
        !worst.starts_with("r_tied/"),
        "worst hold endpoint should not be the tied flop: {worst:?}"
    );
    assert!(t.whs() < 0.0, "the real path here is genuinely marginal — control for the above");
    // and the tied flop contributes nothing to the totals
    let tied = t.pin("r_tied/D").unwrap();
    assert!(!t.report().hold_slacks.iter().any(|(i, _)| *i == tied));
}
