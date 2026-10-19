// SPDX-License-Identifier: Apache-2.0
//! Diagnostic: dump every pin's propagated slew, load and arrival as CSV.
//!
//! Read-only — it changes no engine behaviour. It exists for **correlation against another
//! timer**: sign-off reports (OpenSTA `report_checks`) print per-node `Cap` and `Slew` alongside
//! delay, so having ours in the same shape turns "our delays disagree" into "our *slews* disagree"
//! or "our *loads* disagree", which are different bugs with different fixes.
//!
//! `arrival` is the LATE (setup) propagation and `arrival_min` the EARLY (hold) one. Both are
//! dumped because their SPREAD is itself a finding: with a single corner and flat derate a
//! reference timer carries one edge through the clock network, so a large early/late spread on
//! a clock pin points at our per-lane collapse rather than at any delay number.
//!
//! Usage: `cargo run --release --example dump_pin_state -- JOB.sta|SCRIPT.tcl > pins.csv`
//! (a `.tcl` script is adapted as `vyges-sta-si tcl` adapts it, so the dump is that command's job)

use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;

fn main() {
    let job_path = std::env::args().nth(1).expect("usage: dump_pin_state JOB.sta|SCRIPT.tcl");
    let job = if job_path.ends_with(".tcl") {
        vyges_sta_si::tcl::adapt(&job_path).expect("adapt script").job
    } else {
        StaJob::load(&job_path).expect("load job")
    };
    // the timer's own reader, so a dump shows what the timer saw
    let (nl, lib, spef) =
        vyges_sta_si::engine::load_inputs(&job, Default::default()).expect("load inputs");
    let t = Timer::build(&nl, &lib, &job, spef.as_ref()).expect("build timer");

    // hold slack is per-endpoint rather than per-pin, so index it up front
    let hold: std::collections::HashMap<usize, f64> =
        t.report().hold_slacks.iter().map(|&(i, s)| (i, s)).collect();

    println!("pin,slew,slew_min,load,arrival,arrival_min,setup_slack,hold_slack");
    for p in 0..t.num_pins() {
        let f = |v: Option<f64>| v.map(|x| format!("{x:.6}")).unwrap_or_default();
        println!(
            "{},{:.6},{:.6},{:.6},{:.6},{:.6},{},{}",
            t.pin_label(p),
            t.slew(p),
            t.slew_min(p),
            t.load(p),
            t.arrival(p),
            t.arrival_min(p),
            f(t.slack(p)),
            f(hold.get(&p).copied())
        );
    }
}
