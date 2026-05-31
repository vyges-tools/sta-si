// Multi-clock: a path launched by clk1 (10 ns) and captured by clk2 (4 ns) is
// constrained not by either period but by the tightest launch→capture edge
// separation over the beat. For 10 vs 4 ns that worst separation is 2 ns
// (launch @10 → capture @12), so the cross-domain path is far tighter than the
// same path analysed in a single 10 ns (or even 4 ns) domain.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (mc) {
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

// r1 on clk1, r2 on clk2; cross-domain path r1.Q -> g1 -> r2.D
const NL: &str = "module mc ( clk1, clk2, din, dout ); input clk1, clk2, din; output dout; wire q1, n1;\n\
                  DFF r1 ( .CK(clk1), .D(din), .Q(q1) );\n\
                  INV g1 ( .A(q1),    .Y(n1) );\n\
                  DFF r2 ( .CK(clk2), .D(n1),  .Q(dout) );\n\
                  endmodule";

fn job(clocks: Vec<(String, String, f64)>) -> StaJob {
    StaJob {
        design: "mc".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk1".into(),
        period_ns: 10.0,
        clocks,
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
        exceptions: vec![],
        crpr: true,
        pba: false,
        base_dir: String::new(),
    }
}

fn ck(name: &str, period: f64) -> (String, String, f64) {
    (name.into(), name.into(), period)
}

#[test]
fn cross_domain_uses_tightest_edge_relation() {
    // clk1=10, clk2=4 -> r2/D capture window = 2 ns (the worst launch->capture beat)
    let cross = analyze_inputs(NL, LIB, &job(vec![ck("clk1", 10.0), ck("clk2", 4.0)])).unwrap();
    assert_eq!(cross.worst_endpoint, "r2/D", "cross worst {}", cross.worst_endpoint);
    // required = 0 + 2.0 - setup(0.05); arrival ~0.21 -> slack ~1.74, well under 2.0
    assert!(cross.wns > 1.3 && cross.wns < 1.95, "cross-domain wns={} (expect ~1.74)", cross.wns);
}

#[test]
fn cross_domain_is_tighter_than_either_single_domain() {
    let cross = analyze_inputs(NL, LIB, &job(vec![ck("clk1", 10.0), ck("clk2", 4.0)])).unwrap();
    // same netlist as one 10 ns domain (clk2 unknown -> falls back to primary 10 ns)
    let one10 = analyze_inputs(NL, LIB, &job(vec![])).unwrap();
    // ... and as a single 4 ns domain on both
    let one4 = analyze_inputs(NL, LIB, &job(vec![ck("clk1", 4.0), ck("clk2", 4.0)])).unwrap();
    // 2 ns relation is tighter than 10 ns and even tighter than 4 ns
    assert!(cross.wns < one10.wns - 5.0, "cross {} should be << 10ns-domain {}", cross.wns, one10.wns);
    assert!(cross.wns < one4.wns, "cross {} should be < 4ns-domain {}", cross.wns, one4.wns);
}
