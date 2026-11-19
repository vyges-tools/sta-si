//! The clock slew a timing check's margin is looked up at, as the reference (OpenSTA) has it: the
//! clock pin's slew of the OTHER analysis (`Scene::checkClkSlewMinMax` under its default
//! on-chip-variation analysis) — the early slew for a setup check, the late one for hold.

use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::{StaError, Timer};

/// A clock buffer whose output slew follows its load, a register whose clock pin loads it with a
/// capacitance RANGE (0.001 early, 0.01 late: slew 0.02 early, 0.2 late), and setup / hold tables
/// that follow the clock slew: setup 0.05 at 0.02 and 0.5 at 0.2, hold 0.01 and 0.1.
const LIB: &str = r#"
library (ckslew) {
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (CKBUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output; function : "A";
      timing () {
        related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        cell_fall (t)       { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.02, 0.2", "0.02, 0.2" ); }
        fall_transition (t) { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.02, 0.2", "0.02, 0.2" ); }
      }
    }
  }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.005;
      rise_capacitance_range (0.001, 0.01); fall_capacitance_range (0.001, 0.01); }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.02, 0.2"); index_2 ("0.0, 1.0"); values ( "0.05, 0.05", "0.5, 0.5" ); }
        fall_constraint (s) { index_1 ("0.02, 0.2"); index_2 ("0.0, 1.0"); values ( "0.05, 0.05", "0.5, 0.5" ); } }
      timing () { related_pin : "CK"; timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.02, 0.2"); index_2 ("0.0, 1.0"); values ( "0.01, 0.01", "0.1, 0.1" ); }
        fall_constraint (s) { index_1 ("0.02, 0.2"); index_2 ("0.0, 1.0"); values ( "0.01, 0.01", "0.1, 0.1" ); } }
    }
    pin (Q) {
      direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        cell_fall (t)       { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        rise_transition (t) { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        fall_transition (t) { index_1 ("0.0, 1.0"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); } }
    }
  }
}
"#;

fn job(period: f64) -> StaJob {
    StaJob {
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "t".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: period,
        clocks: vec![],
        input_slew: 0.0,
        input_slews: Vec::new(),
        output_load: 0.0,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 1.0,
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions: vec![],
        async_groups: vec![],
        clock_groups: vec![],
        ideal_clocks: Default::default(),
        clock_waveforms: Default::default(),
        driving_cells: Default::default(),
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
        output_delay_declared: false,
        unconstrained_outputs: false,
        io_input_delays: vec![],
        io_output_delays: vec![],
        io_input_all_but: vec![],
        io_output_all_but: vec![],
        case_values: Vec::new(),
        disabled_timing: Vec::new(),
        data_check_cmds: Vec::new(),
        max_time_borrow: Vec::new(),
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        clock_uncertainty: Vec::new(),
        sdc: None,
        metadata: None,
        base_dir: String::new(),
    }
}

fn build(nl: &str, j: &StaJob) -> Result<Timer, StaError> {
    let nl = vyges_sta_si::netlist::parse(nl).map_err(|e| StaError::Parse(e.to_string()))?;
    let lib = vyges_sta_si::liberty::Lib::parse(LIB).map_err(|e| StaError::Parse(e.to_string()))?;
    Timer::build(&nl, &lib, j, None)
}

const NL: &str = "module t (clk, d, y); input clk, d; output y; wire ck;\n\
     CKBUF b (.A(clk), .Y(ck));\n\
     DFF r (.CK(ck), .D(d), .Q(y));\n\
     endmodule";

/// Setup: the early clock slew (0.02) sets the setup time (0.05), not the late one (0.2 -> 0.5).
/// d arrives at 0; required is the period plus the clock latency (0.1) less 0.05.
#[test]
fn a_setup_check_reads_the_early_clock_slew() {
    let t = build(NL, &job(10.0)).unwrap();
    let r_d = t.pin("r/D").unwrap();
    let s = t.slack(r_d).unwrap();
    assert!((s - 10.05).abs() < 1e-6, "setup slack {s}");
}

/// Hold: the late clock slew (0.2) sets the hold time (0.1), not the early one (0.02 -> 0.01).
/// d arrives at 0 against the clock latency (0.1) plus 0.1.
#[test]
fn a_hold_check_reads_the_late_clock_slew() {
    let t = build(NL, &job(10.0)).unwrap();
    let r_d = t.pin("r/D").unwrap();
    let h = t.report().hold_slacks.iter().find(|(p, _)| *p == r_d).map(|(_, s)| *s).unwrap();
    assert!((h + 0.2).abs() < 1e-6, "hold slack {h}");
}
