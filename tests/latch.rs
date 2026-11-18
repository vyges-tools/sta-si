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
  cell (DLL) {
    latch (IQ, IQN) { enable : "!G"; data_in : "D"; }
    pin (G) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "G"; timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); } }
      timing () { related_pin : "G"; timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); } }
    }
    pin (Q) {
      direction : output; function : "IQ";
      timing () { related_pin : "G"; timing_type : falling_edge;
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

fn build(nl: &str, j: &StaJob) -> Result<Timer, StaError> {
    let nl = vyges_sta_si::netlist::parse(nl).map_err(|e| StaError::Parse(e.to_string()))?;
    let lib = vyges_sta_si::liberty::Lib::parse(LIB).map_err(|e| StaError::Parse(e.to_string()))?;
    Timer::build(&nl, &lib, j, None)
}

fn pin(t: &Timer, name: &str) -> usize {
    t.pin(name).unwrap_or_else(|| panic!("no pin {name}"))
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6 // delays are stored in float, as the reference stores them
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
    let Err(err) = build(nl, &job(10.0)) else { panic!("a ring of inverters is a loop") };
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

/// The reference's levelization does not search through a latch D->Q arc
/// (`Levelize::searchThru`): a latch output is ordered by its enable, so a loop closed only through
/// a latch — the latch feeding its own data, as a latch-based clock gate's enable feeds back from
/// the flops it clocks — is no loop.
#[test]
fn a_loop_closed_only_through_a_latch_is_no_loop() {
    let nl = "module t (clk, y); input clk; output y; wire q, nq;\n\
              DLH l1 (.G(clk), .D(nq), .Q(q));\n\
              INV i1 (.A(q), .Y(nq));\n\
              DFF r (.CK(clk), .D(q), .Q(y));\n\
              endmodule";
    let t = build(nl, &job(10.0)).expect("a loop through a latch D->Q arc is not refused");
    // the latch output is timed from its enable: G->Q, 0.3 after the clock edge
    assert!(close(t.arrival(pin(&t, "l1/Q")), 0.3), "l1/Q late {}", t.arrival(pin(&t, "l1/Q")));
}

/// On the min analysis nothing crosses a latch D->Q arc: the reference propagates it only for max
/// (`Search` latch D->Q branch: `min_max == MinMax::max()`), so the earliest latch output is its
/// enable path even when data through D would be earlier.
#[test]
fn the_earliest_latch_output_is_its_enable_path() {
    // d arrives at 0, so D->Q (0.25) would beat G->Q (0.3) at l1/Q if it were searched
    let nl = "module t (clk, d, y); input clk, d; output y; wire q;\n\
              DLH l1 (.G(clk), .D(d), .Q(q));\n\
              DFF r (.CK(clk), .D(q), .Q(y));\n\
              endmodule";
    let t = build(nl, &job(10.0)).unwrap();
    let a = t.arrival_min(pin(&t, "r/D"));
    assert!(close(a, 0.3), "r/D early arrival {a}, the G->Q path's 0.3");
}

/// The latch output a test's data reaches is timed from its enable; the checks below look at the
/// latch data pin `l1/D`, fed from a rising-edge register through two inverters: 0.2 + 2 x 0.1.
const BORROW_NL: &str = "module t (clk, y); input clk; output y; wire a, b, c, q;\n\
     DFF r0 (.CK(clk), .D(y), .Q(a));\n\
     INV i1 (.A(a), .Y(b));\n\
     INV i2 (.A(b), .Y(c));\n\
     LATCH l1 (.G(clk), .D(c), .Q(q));\n\
     DFF r1 (.CK(clk), .D(q), .Q(y));\n\
     endmodule";

fn borrow_of(t: &Timer, p: usize) -> Option<f64> {
    t.report().latch_borrow.iter().find(|(q, _)| *q == p).map(|(_, b)| *b)
}

/// Data reaching a latch while it is open borrows the time it is late and meets its check exactly
/// (`Latches::latchRequired`: required is the data's own arrival). A positive latch opens on the
/// rise: the falling edge its setup check is against is the first after the launch (5), and it
/// opens a pulse width before that, at 0. The data at 0.4 borrows 0.4; slack 0, not the closing
/// edge's 5 - 0.05 - 0.4.
#[test]
fn data_arriving_while_the_latch_is_open_borrows() {
    let t = build(&BORROW_NL.replace("LATCH", "DLH"), &job(10.0)).unwrap();
    let d = pin(&t, "l1/D");
    let s = t.slack(d).expect("a latch data pin is a setup end");
    assert!(close(s, 0.0), "slack {s}");
    let b = borrow_of(&t, d).expect("a latch end reports its borrow");
    assert!(close(b, 0.4), "borrow {b}");
}

/// Past the maximum borrow the check fails by what is left: borrow is the maximum (pulse width
/// less the setup time, ideal clocks) and required is the opening plus it — the closing edge's
/// ordinary setup requirement. Period 0.8: open 0, maximum 0.4 - 0.05, data at 0.4.
#[test]
fn data_past_the_maximum_borrow_fails_by_the_rest() {
    let t = build(&BORROW_NL.replace("LATCH", "DLH"), &job(0.8)).unwrap();
    let d = pin(&t, "l1/D");
    let s = t.slack(d).unwrap();
    assert!(close(s, -0.05), "slack {s}");
    let b = borrow_of(&t, d).unwrap();
    assert!(close(b, 0.35), "borrow {b}");
}

/// Data arriving before the latch opens has to meet the OPENING, not the closing edge, and borrows
/// nothing. A negative latch opens on the fall: its setup check is against the rise after the
/// launch (10), and it opens a pulse width before that, at 5. Slack 5 - 0.4.
#[test]
fn data_before_the_latch_opens_is_checked_against_the_opening() {
    let t = build(&BORROW_NL.replace("LATCH", "DLL"), &job(10.0)).unwrap();
    let d = pin(&t, "l1/D");
    let s = t.slack(d).unwrap();
    assert!(close(s, 4.6), "slack {s}");
    let b = borrow_of(&t, d).unwrap();
    assert!(close(b, 0.0), "borrow {b}");
}
