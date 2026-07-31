// SPDX-License-Identifier: Apache-2.0
//! Clock-source → output-port paths.
//!
//! A block that forwards its clock off-chip — the shape of every SRAM or DDR interface —
//! has a timed path whose **startpoint is the clock source itself**. Its launch is a clock
//! *edge*, not t=0: the rising edge leaves at 0 and the falling edge half a period later,
//! while every other path in the graph carries a delay measured *from* whichever edge
//! launched it.
//!
//! Missing this does not make a number wrong; it makes an endpoint set incomplete, which is
//! worse, because the reported WNS then belongs to a path that is not the critical one. On a
//! design where the forwarded clock is the critical path, the half-period of launch time is
//! the entire difference.
//!
//! The asymmetric fixture is the discriminating one: there the falling edge is the *faster*
//! edge through the cell, so it can only be critical because of **when it leaves**. A model
//! that just added half a period to whichever edge was already worst would fail it.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/clkout/");

fn timer(job_file: &str) -> (Timer, StaJob) {
    let job = StaJob::load(&format!("{D}{job_file}")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    (Timer::build(&nl, &lib, &job, None).unwrap(), job)
}

#[test]
fn a_forwarded_clock_launches_on_a_clock_edge_not_at_zero() {
    let (t, job) = timer("clkout.sta");
    let p = t.pin("clk_o").expect("the forwarded clock port");

    // `arrival` keeps its meaning — delay from the launching edge — so the two differ by
    // exactly the fall edge's launch time. With symmetric rise/fall cells that is period/2.
    let half = job.period_ns / 2.0;
    let delta = t.setup_arrival(p) - t.arrival(p);
    assert!(
        (delta - half).abs() < 1e-9,
        "the fall edge launches at half a period: setup_arrival - arrival = {delta}, want {half}"
    );

    // and the slack that is reported has to be the one that offset produces
    let slack = t.slack(p).expect("clk_o is a setup endpoint");
    let req = t.required(p).unwrap();
    assert!(
        (slack - (req - t.setup_arrival(p))).abs() < 1e-9,
        "reported slack must come from the launch-aware arrival"
    );
}

#[test]
fn the_forwarded_clock_is_the_critical_path() {
    let (t, _) = timer("clkout.sta");
    assert_eq!(
        t.report().worst_endpoint,
        "clk_o",
        "half a period of launch time makes the forwarded clock worse than any flop path here"
    );
    // WNS must agree with the endpoint ranking — the two are computed separately.
    let (worst_pin, worst_slack) = t.endpoint_slacks()[0];
    assert_eq!(t.pin_label(worst_pin), "clk_o");
    assert!(
        (worst_slack - t.report().wns).abs() < 1e-9,
        "endpoint ranking and WNS disagree: {worst_slack} vs {}",
        t.report().wns
    );
}

#[test]
fn an_ordinary_flop_launched_output_is_untouched() {
    let (t, _) = timer("clkout.sta");
    let p = t.pin("q_o").expect("the data output port");
    assert!(
        (t.setup_arrival(p) - t.arrival(p)).abs() < 1e-9,
        "a flop already launches on the clock edge — its arrival must not be offset again"
    );
    assert!(
        t.slack(p).unwrap() > t.slack(t.pin("clk_o").unwrap()).unwrap(),
        "the data path should be the slacker of the two in this fixture"
    );
}

#[test]
fn the_launch_time_is_per_edge_not_a_blanket_offset() {
    // BUFA's rising edge is much slower than its falling one, so the main (max-lane)
    // arrival at clk_o is the RISE chain. The clock-launched worst is nonetheless the
    // fall chain, because it leaves half a period later — the offset therefore cannot
    // equal period/2 here, and must be smaller by the rise/fall delay difference.
    let (t, job) = timer("clkout_asym.sta");
    let p = t.pin("clk_o").expect("the forwarded clock port");
    let half = job.period_ns / 2.0;
    let delta = t.setup_arrival(p) - t.arrival(p);
    assert!(
        delta > 0.0 && delta < half - 1e-6,
        "expected a per-edge launch (0 < delta < {half}), got {delta} — a blanket \
         period/2 offset would give exactly {half}"
    );
    // It still launched on the fall edge: the arrival is past the half-period mark.
    assert!(
        t.setup_arrival(p) > half,
        "the falling edge leaves at {half}; arrival {} cannot precede it",
        t.setup_arrival(p)
    );
}
