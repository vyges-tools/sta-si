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
//! Usage: `cargo run --release --example dump_pin_state -- JOB.sta > pins.csv`

use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::sta::Timer;

fn main() {
    let job_path = std::env::args().nth(1).expect("usage: dump_pin_state JOB.sta");
    let job = StaJob::load(&job_path).expect("load job");
    let nl = vyges_sta_si::netlist::load(&job.resolve(&job.netlist)).expect("load netlist");
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).expect("load lib").cells);
    }
    let spef = job
        .spef
        .as_ref()
        .map(|p| vyges_sta_si::spef::Spef::load(&job.resolve(p)).expect("load spef"));
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
