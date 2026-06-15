//! SDC reader: parsing the Tcl subset + merging onto a job + the engine effect
//! of the I/O budget and clock uncertainty.

use vyges_sta_si::job::{ExcKind, StaJob};
use vyges_sta_si::sdc::Sdc;

const SDC: &str = r#"
# a representative sign-off SDC
set clk_period 10.0
create_clock -name core_clk -period $clk_period [get_ports clk]
set_input_delay  2.0 -clock core_clk [all_inputs]
set_output_delay 3.0 -clock core_clk [all_outputs]
set_input_delay  1.5 -clock core_clk [get_ports data_in]
set_clock_uncertainty 0.25 -setup
set_clock_uncertainty 0.10 -hold
set_input_transition 0.08 [all_inputs]
set_load 0.012 [all_outputs]
set_timing_derate -late 1.06
set_timing_derate -early 0.94
set_false_path -from [get_ports test_mode] -to [get_ports done]
set_multicycle_path 2 -from [get_pins a_reg/Q] -to [get_pins b_reg/D] -setup
set_propagated_clock [all_clocks]   ;# ignored, we propagate insertion anyway
"#;

#[test]
fn parses_core_commands() {
    let s = Sdc::parse(SDC).expect("parse");
    assert_eq!(s.clocks.len(), 1);
    assert_eq!(s.clocks[0].name, "core_clk");
    assert_eq!(s.clocks[0].source, "clk");
    assert!((s.clocks[0].period - 10.0).abs() < 1e-9, "var-subst period");

    // one default + one per-port input delay
    assert!(s.input_delays.iter().any(|d| d.default && (d.value - 2.0).abs() < 1e-9));
    assert!(s
        .input_delays
        .iter()
        .any(|d| d.ports.iter().any(|p| p == "data_in") && (d.value - 1.5).abs() < 1e-9));
    assert!(s.output_delays.iter().any(|d| d.default && (d.value - 3.0).abs() < 1e-9));

    assert!((s.setup_uncertainty - 0.25).abs() < 1e-9);
    assert!((s.hold_uncertainty - 0.10).abs() < 1e-9);
    assert_eq!(s.input_transition, Some(0.08));
    assert_eq!(s.load, Some(0.012));
    assert_eq!(s.late_derate, Some(1.06));
    assert_eq!(s.early_derate, Some(0.94));

    assert_eq!(s.exceptions.len(), 2);
    assert!(matches!(s.exceptions[0].kind, ExcKind::FalsePath));
    assert!(matches!(s.exceptions[1].kind, ExcKind::Multicycle(2)));
    assert_eq!(s.exceptions[0].from, "test_mode");
    assert_eq!(s.exceptions[1].from, "a_reg");
    assert!(s.ignored.iter().any(|c| c == "set_propagated_clock"));
}

#[test]
fn generated_clock_period_resolves() {
    let sdc = r#"
        create_clock -name clk -period 4.0 [get_ports clk]
        create_generated_clock -name clk_div2 -source [get_ports clk] -divide_by 2 [get_pins div/Q]
    "#;
    let s = Sdc::parse(sdc).unwrap();
    let g = s.clocks.iter().find(|c| c.name == "clk_div2").expect("gen clock");
    assert!((g.period - 8.0).abs() < 1e-9, "divide_by 2 -> 2x period");
    assert_eq!(g.source, "div/Q");
}

#[test]
fn units_scale_ps_to_ns() {
    let sdc = r#"
        set_units -time 1ps -capacitance 1fF
        create_clock -name clk -period 5000 [get_ports clk]
        set_input_delay 1000 -clock clk [all_inputs]
        set_load 12 [all_outputs]
    "#;
    let s = Sdc::parse(sdc).unwrap();
    assert!((s.clocks[0].period - 5.0).abs() < 1e-9, "5000 ps -> 5 ns");
    assert!(s.input_delays[0].default && (s.input_delays[0].value - 1.0).abs() < 1e-9);
    assert!((s.load.unwrap() - 0.012).abs() < 1e-9, "12 fF -> 0.012 pF");
}

#[test]
fn merge_overrides_job_clock_and_io() {
    // a job with NO clock — the SDC supplies it.
    let job_text = "design: top\nnetlist: top.v\nlib: x.lib\nsdc: top.sdc\n";
    let mut job = StaJob::parse(job_text, ".").expect("clockless job ok with sdc:");
    assert!(job.clock_port.is_empty(), "no clock until merge");

    let s = Sdc::parse(SDC).unwrap();
    vyges_sta_si::job::merge_sdc_into(&s, &mut job);
    assert_eq!(job.clock_port, "clk");
    assert!((job.period_ns - 10.0).abs() < 1e-9);
    // default input delay + clock_latency(0) ; per-port override for data_in
    assert!((job.input_delay - 2.0).abs() < 1e-9);
    assert!((job.input_delay_for("data_in") - 1.5).abs() < 1e-9);
    assert!((job.output_delay_for("any_out") - 3.0).abs() < 1e-9);
    assert!((job.setup_uncertainty - 0.25).abs() < 1e-9);
    assert!((job.input_slew - 0.08).abs() < 1e-9);
    assert!((job.late_derate - 1.06).abs() < 1e-9);
    assert_eq!(job.exceptions.len(), 2);
}

// ---- engine effect: I/O budget + uncertainty must reduce setup slack --------

const LIB: &str = r#"
library (d) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
}
"#;

const NL: &str = "module top ( a, y ); input a; output y; wire n1;\n\
                  INV u1 ( .A(a), .Y(n1) ); INV u2 ( .A(n1), .Y(y) ); endmodule";

fn run(input_delay: f64, output_delay: f64, unc: f64) -> f64 {
    use vyges_sta_si::engine::analyze_inputs;
    let mut job = base_job();
    job.input_delay = input_delay;
    job.output_delay = output_delay;
    job.setup_uncertainty = unc;
    analyze_inputs(NL, LIB, &job).unwrap().wns
}

fn base_job() -> StaJob {
    let mut j = StaJob::parse("design: top\nnetlist: x\nlib: x\nclock: clk 10.0\n", ".").unwrap();
    j.input_slew = 0.02;
    j.output_load = 0.005;
    j.miller = 1.0; // disable SI for a clean, deterministic delta
    j
}

#[test]
fn io_budget_and_uncertainty_eat_slack() {
    let base = run(0.0, 0.0, 0.0);
    // input delay pushes the launch later -> less slack, 1:1
    let with_in = run(1.0, 0.0, 0.0);
    assert!((base - with_in - 1.0).abs() < 1e-6, "input_delay 1ns -> 1ns less slack");
    // output delay eats the period at the endpoint -> less slack, 1:1
    let with_out = run(0.0, 2.0, 0.0);
    assert!((base - with_out - 2.0).abs() < 1e-6, "output_delay 2ns -> 2ns less slack");
    // setup uncertainty tightens required time -> less slack, 1:1
    let with_unc = run(0.0, 0.0, 0.3);
    assert!((base - with_unc - 0.3).abs() < 1e-6, "uncertainty 0.3ns -> 0.3ns less slack");
}
