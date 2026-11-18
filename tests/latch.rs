//! Level-sensitive latches and timing loops, each rule as the reference timer (OpenSTA) has it,
//! stated at its test: how the loop check treats a latch's D->Q arc, what reaches a latch output,
//! and how a latch data pin is checked (time borrowing).

use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::{StaError, Timer};

/// INV, a rising-edge DFF and two level-sensitive latches: DLH open while G is high, DLL open
/// while G is low. Constant tables, so every delay below can be read off them.
const LIB: &str = r#"
library (latch) {
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output; function : "!A";
      timing () {
        related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        cell_fall (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        fall_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
      }
    }
  }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); } }
      timing () { related_pin : "CK"; timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); } }
    }
    pin (Q) {
      direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        cell_fall (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        rise_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        fall_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); } }
    }
  }
  cell (DLH) {
    latch (IQ, IQN) { enable : "G"; data_in : "D"; }
    pin (G) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "G"; timing_type : setup_falling;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); } }
      timing () { related_pin : "G"; timing_type : hold_falling;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); } }
    }
    pin (Q) {
      direction : output; function : "IQ";
      timing () { related_pin : "G"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        cell_fall (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        rise_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        fall_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); } }
      timing () { related_pin : "D"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.25, 0.25", "0.25, 0.25" ); }
        cell_fall (t)       { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.25, 0.25", "0.25, 0.25" ); }
        rise_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        fall_transition (t) { index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); } }
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
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        metadata: None,
        base_dir: String::new(),
    }
}

fn build(nl: &str, j: &StaJob) -> Result<(), StaError> {
    let nl = vyges_sta_si::netlist::parse(nl).map_err(|e| StaError::Parse(e.to_string()))?;
    let lib = vyges_sta_si::liberty::Lib::parse(LIB).map_err(|e| StaError::Parse(e.to_string()))?;
    Timer::build(&nl, &lib, j, None).map(|_| ())
}

/// A loop the order cannot pass is refused BY NAME: the message lists the pins of one cycle, in
/// order, so it can be found and cut. (The reference breaks such a loop at a back edge and warns;
/// this timer refuses, and the refusal has to say where.)
#[test]
fn a_combinational_loop_is_refused_naming_its_pins() {
    let nl = "module t (clk, y); input clk; output y; wire a, b;\n\
              INV g1 (.A(b), .Y(a));\n\
              INV g2 (.A(a), .Y(b));\n\
              INV g3 (.A(a), .Y(y));\n\
              endmodule";
    let err = build(nl, &job(10.0)).expect_err("a ring of inverters is a loop");
    let StaError::CombinationalLoop(pins) = &err else { panic!("not a loop refusal: {err}") };
    let mut sorted = pins.clone();
    sorted.sort();
    assert_eq!(sorted, ["g1/A", "g1/Y", "g2/A", "g2/Y"], "the cycle's pins: {pins:?}");
    let msg = err.to_string();
    for p in ["g1/A", "g1/Y", "g2/A", "g2/Y"] {
        assert!(msg.contains(p), "the message names {p}: {msg}");
    }
    assert!(!msg.contains("g3"), "a pin off the cycle is not named: {msg}");
}
