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

// reg -> comb -> reg : r1.Q -> g1 -> r2.D ; r2.Q -> y
const NL: &str = "module seq ( clk, a, y ); input clk, a; output y; wire q1, n1;\n\
                  DFF r1 ( .CK(clk), .D(a),  .Q(q1) );\n\
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
