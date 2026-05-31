// Clock-network insertion delay + skew. A clock buffer (two inverters) delays one
// flop's clock; the capture flop's insertion delay enters its required time, the
// launch flop's enters the data arrival, so common latency cancels and only the
// difference (skew) moves slack.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (sk) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
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
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.10, 0.22" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.10, 0.22" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.03, 0.09" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.03, 0.09" ); }
      }
    }
  }
}
"#;

// Internal two-flop ring (r2.Q -> q2 -> r1.D ; r1.Q -[g1]-> r2.D) so neither flop
// Q is a primary output — primary I/O would need set_in/out_delay we don't model.
// A separate din -> dout feed-through gives the module its port without touching
// the reg-to-reg paths under test. Both flops on the same (undelayed) clock.
const NL_FLAT: &str = "module sk ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       DFF r1 ( .CK(clk), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(clk), .D(n1), .Q(q2) );\n\
                       endmodule";

// a 2-inverter clock buffer (clk -> ckd) delays r2's clock only -> skew
const NL_SKEW: &str = "module sk ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1, ck1, ckd;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       INV cb1 ( .A(clk), .Y(ck1) );\n\
                       INV cb2 ( .A(ck1), .Y(ckd) );\n\
                       DFF r1 ( .CK(clk), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(ckd), .D(n1), .Q(q2) );\n\
                       endmodule";

// both flops on the SAME delayed clock -> common latency, no skew
const NL_BOTH_DELAYED: &str = "module sk ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1, ck1, ckd;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       INV cb1 ( .A(clk), .Y(ck1) );\n\
                       INV cb2 ( .A(ck1), .Y(ckd) );\n\
                       DFF r1 ( .CK(ckd), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(ckd), .D(n1), .Q(q2) );\n\
                       endmodule";

fn job() -> StaJob {
    StaJob {
        design: "sk".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
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
        crpr: true,
        base_dir: String::new(),
    }
}

#[test]
fn skew_shifts_worst_endpoint_and_tightens_slack() {
    let flat = analyze_inputs(NL_FLAT, LIB, &job()).unwrap();
    let skew = analyze_inputs(NL_SKEW, LIB, &job()).unwrap();

    // undelayed: the reg->reg path (r1 -> g1 -> r2) is worst, captured at r2/D
    assert_eq!(flat.worst_endpoint, "r2/D", "flat worst {}", flat.worst_endpoint);

    // delaying r2's clock credits r2/D more required time (skew helps it) but
    // delays r2/Q's launch, so the path captured at r1/D becomes the tight one.
    assert_eq!(skew.worst_endpoint, "r1/D", "skew worst {}", skew.worst_endpoint);
    assert!(skew.wns < flat.wns, "skew {} !< flat {}", skew.wns, flat.wns);
}

#[test]
fn common_clock_latency_cancels() {
    // both flops behind the same buffer: launch latency (in arrival) and capture
    // latency (in required) are equal, so they cancel — slack must match the
    // zero-latency design to numerical precision.
    let flat = analyze_inputs(NL_FLAT, LIB, &job()).unwrap();
    let both = analyze_inputs(NL_BOTH_DELAYED, LIB, &job()).unwrap();
    assert!(
        (both.wns - flat.wns).abs() < 1e-9,
        "common latency did not cancel: both {} vs flat {}",
        both.wns,
        flat.wns
    );
}
