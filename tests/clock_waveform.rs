// SPDX-License-Identifier: Apache-2.0
//! Clock edge times, rule by rule, on the two-flop ring `r1.Q -[g1]-> r2.D`, `r2.Q` the output
//! `y`. The rules are the reference's (`ClockEdge::time`, `Clock::generate`, `CycleAccting`); they
//! were also witnessed against it on gcd (sky130hd), 13 of 14 variants WNS/TNS/worst slack both
//! ways (the 14th launches one endpoint from two clocks, which this timer merges).
use vyges_sta_si::job::{merge_sdc_into, StaJob};
use vyges_sta_si::sdc::Sdc;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/");

fn wns(sdc: &str) -> f64 {
    let mut job = StaJob::load(&format!("{D}seq.sta")).unwrap();
    merge_sdc_into(&Sdc::parse(sdc).unwrap(), &mut job);
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let lib = Lib::load(&job.resolve(&job.libs[0])).unwrap();
    Timer::build(&nl, &lib, &job, None).unwrap().report().wns
}

/// Rule (`ClockEdge::time`, `CycleAccting`): an output delay relative to a clock is required at
/// THAT clock's edge, its waveform's time — a virtual clock rising 0.3 into the period is the first
/// capture edge after the launch at 0, so the setup relation is 0.3 where a clock rising at 0
/// gives a whole period.
#[test]
fn an_output_delay_is_required_at_its_clock_edge_time() {
    let base = "create_clock -name clk -period 1 [get_ports clk]\n";
    let at0 = wns(&format!("{base}create_clock -name v -period 1\nset_output_delay 0.1 -clock v [get_ports y]\n"));
    let at3 = wns(&format!("{base}create_clock -name v -period 1 -waveform {{0.3 0.8}}\nset_output_delay 0.1 -clock v [get_ports y]\n"));
    assert!((at3 - at0 + 0.7).abs() < 1e-9, "{at0} {at3}");
}

/// Rule (`Clock::generate`, `generateEdgesClk`): a generated clock carries the edges derived from
/// its master — `-edges {2 3 4}` rises on the master's fall (0.5) and falls on its next rise.
#[test]
fn a_generated_clock_carries_its_derived_edges() {
    let mut job = StaJob::load(&format!("{D}seq.sta")).unwrap();
    let sdc = "create_clock -name clk -period 1 [get_ports clk]\ncreate_generated_clock -name g -source [get_ports clk] -master_clock clk -edges {2 3 4} -add [get_ports clk]\n";
    merge_sdc_into(&Sdc::parse(sdc).unwrap(), &mut job);
    assert_eq!(job.clock_waveforms.get("g"), Some(&[0.5, 1.0]));
    assert_eq!(job.clock_waveforms.get("clk"), None, "no -waveform: the default edges");
}
