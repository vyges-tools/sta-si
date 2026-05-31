use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (seq) {
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
      timing () {
        related_pin : "CK";
        timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
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

// two-flop ring: r1.Q -[g1]-> r2.D ; r2.Q (= output y) -> r1.D.
// both flop D pins are launched by a flop Q, so setup + hold are reg-to-reg.
const NL: &str = "module seq ( clk, y ); input clk; output y; wire q1, n1;\n\
                  DFF r1 ( .CK(clk), .D(y),  .Q(q1) );\n\
                  INV g1 ( .A(q1),   .Y(n1) );\n\
                  DFF r2 ( .CK(clk), .D(n1), .Q(y)  );\n\
                  endmodule";

fn job(period: f64) -> StaJob {
    StaJob {
        design: "seq".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: period,
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
        base_dir: String::new(),
    }
}

#[test]
fn reg_to_reg_setup_path() {
    let rep = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    // endpoints: r1/D, r2/D (flop data pins) + y (primary output)
    assert!(rep.endpoints >= 3, "endpoints={}", rep.endpoints);
    // the tight path is the reg->reg one, captured at r2/D
    assert_eq!(rep.worst_endpoint, "r2/D");
    // launched by r1/Q (CK->Q), through g1
    assert!(rep.worst_path.iter().any(|p| p.label == "r1/Q"), "{:?}", rep.worst_path);
    // required = period - setup(0.05); 1 ns easily met
    assert!(rep.wns > 0.0 && rep.wns < 0.95, "wns={}", rep.wns);
}

#[test]
fn tighter_setup_eats_slack() {
    // larger setup -> required earlier -> less slack
    let base = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    let tight = analyze_inputs(NL, &LIB.replace("\"0.05\"", "\"0.20\""), &job(1.0)).unwrap();
    assert!(tight.wns < base.wns, "tight {} !< base {}", tight.wns, base.wns);
}

#[test]
fn reg_to_reg_hold_path() {
    let rep = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    // both flop D pins are hold endpoints, launched by the min CK->Q path
    assert_eq!(rep.hold_endpoints, 2, "hold_endpoints={}", rep.hold_endpoints);
    // earliest data (>= ~min CK->Q) easily clears the 0.02 ns hold here
    assert!(rep.whs > 0.0, "whs={}", rep.whs);
    // worst hold path starts at the clock and reaches a flop D via a flop Q
    let labels: Vec<&str> = rep.worst_hold_path.iter().map(|p| p.label.as_str()).collect();
    assert!(rep.worst_hold_endpoint.ends_with("/D"), "{}", rep.worst_hold_endpoint);
    assert!(labels.iter().any(|l| l.ends_with("/Q")), "{:?}", labels);
}

#[test]
fn bigger_hold_eats_hold_slack() {
    // larger hold requirement -> the same early arrival clears less of it
    let base = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    let tight = analyze_inputs(NL, &LIB.replace("\"0.02\"", "\"0.09\""), &job(1.0)).unwrap();
    assert!(tight.whs < base.whs, "tight {} !< base {}", tight.whs, base.whs);
    // hold slack is period-independent (same-edge check) — unchanged at 2 ns
    let wide = analyze_inputs(NL, LIB, &job(2.0)).unwrap();
    assert!((wide.whs - base.whs).abs() < 1e-9, "hold moved with period");
}
