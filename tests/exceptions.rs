// Timing exceptions: false_path drops a path from analysis; multicycle moves the
// capture edge out. A reg→reg path that violates at a tight 1-cycle period either
// disappears (false path) or meets (multicycle 2).
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::{ExcKind, Exception, StaJob};

const LIB: &str = r#"
library (ex) {
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
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input;
      capacitance : 0.001;
      timing () {
        related_pin : "CK";
        timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
      }
    }
    pin (Q) {
      direction : output;
      timing () {
        related_pin : "CK";
        timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
      }
    }
  }
}
"#;

// din -> r1 -> g1 -> r2 -> dout ; the reg→reg r1->r2 is the tight path
const NL: &str = "module ex ( clk, din, dout ); input clk, din; output dout; wire q1, n1;\n\
                  DFF r1 ( .CK(clk), .D(din), .Q(q1) );\n\
                  INV g1 ( .A(q1),   .Y(n1) );\n\
                  DFF r2 ( .CK(clk), .D(n1),  .Q(dout) );\n\
                  endmodule";

fn job(exceptions: Vec<Exception>) -> StaJob {
    StaJob {
        design: "ex".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 0.25, // tight: the reg→reg path violates at 1 cycle
        clocks: vec![],
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions,
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
        io_input_delays: vec![],
        io_output_delays: vec![],
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        base_dir: String::new(),
    }
}

fn exc(kind: ExcKind, from: &str, to: &str) -> Exception {
    Exception { kind, from: from.into(), to: to.into() }
}

#[test]
fn false_path_drops_the_violating_endpoint() {
    let base = analyze_inputs(NL, LIB, &job(vec![])).unwrap();
    // at 0.25 ns the reg→reg path violates and is the worst endpoint
    assert_eq!(base.worst_endpoint, "r2/D", "base worst {}", base.worst_endpoint);
    assert!(base.wns < 0.0, "base should violate, wns={}", base.wns);

    let fp = analyze_inputs(NL, LIB, &job(vec![exc(ExcKind::FalsePath, "r1", "r2")])).unwrap();
    // r2/D is excluded -> it's no longer the worst, and timing now meets
    assert_ne!(fp.worst_endpoint, "r2/D", "false path should drop r2/D");
    assert!(fp.wns > 0.0, "with r1->r2 false, design meets: wns={}", fp.wns);
    assert_eq!(fp.endpoints, base.endpoints - 1, "one fewer setup endpoint");
}

#[test]
fn multicycle_relaxes_the_path() {
    let base = analyze_inputs(NL, LIB, &job(vec![])).unwrap();
    assert!(base.wns < 0.0 && base.worst_endpoint == "r2/D", "base violates at r2/D");
    let mc = analyze_inputs(NL, LIB, &job(vec![exc(ExcKind::Multicycle(2), "r1", "r2")])).unwrap();
    // a 2-cycle path gets one extra period of capture window -> r2/D meets and is
    // no longer the binding endpoint, so the design meets.
    assert!(mc.wns > 0.0, "2-cycle path meets: wns={}", mc.wns);
    assert_ne!(mc.worst_endpoint, "r2/D", "multicycle should relax r2/D off the critical path");
}
