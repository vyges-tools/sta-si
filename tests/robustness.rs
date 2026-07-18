// Robustness fixes surfaced by the routed-block (sky130 counter) correlation:
//  (1) SPEF *C_UNIT/*R_UNIT scaling (OpenRCX emits pF, not fF),
//  (2) physical-only cells (fill/decap/tap, no connections) must be skipped,
//  (3) async clear/preset arcs (e.g. dfrtp RESET_B->Q) are not max-delay data paths.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;
use vyges_sta_si::spef::Spef;

#[test]
fn spef_pf_units_scale_to_ff() {
    // *C_UNIT 1 PF: a "0.010" grounded cap is 0.010 pF = 10 fF internally.
    let spef = r#"
*SPEF "ieee 1481-1999"
*C_UNIT 1 PF
*R_UNIT 1 KOHM
*NAME_MAP
*1 n1
*D_NET *1 0.020000
*CAP
1 *1 0.010000
*RES
1 *1 *1 1.000000
*END
"#;
    let s = Spef::parse(spef);
    let rc = s.nets.get("n1").expect("n1");
    assert!(
        (rc.cap_ff - 20.0).abs() < 1e-6,
        "D_NET 0.02 pF -> 20 fF, got {}",
        rc.cap_ff
    );
    assert!(
        (rc.ground[0].1 - 10.0).abs() < 1e-6,
        "0.01 pF -> 10 fF, got {}",
        rc.ground[0].1
    );
    // 1 KOHM -> 1000 Ω
    assert!(
        (rc.res[0].2 - 1000.0).abs() < 1e-6,
        "1 KOHM -> 1000 Ω, got {}",
        rc.res[0].2
    );
}

const LIB: &str = r#"
library (r) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
  cell (DFFR) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; clear : "RESET_B'"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (RESET_B) { direction : input; capacitance : 0.001; }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); } }
    }
    pin (Q) {
      direction : output;
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        cell_fall (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); } }
      timing () { related_pin : "RESET_B"; timing_type : clear;
        cell_fall (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "5.00, 5.00", "5.00, 5.00" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); } }
    }
  }
}
"#;

// fill cell (not in lib, no connections) + a reg->reg ring with async reset
const NL: &str = "module top ( clk, rst_n, q ); input clk, rst_n; output q; wire q1, n1;\n\
                  sky130_fill FILLER_0 ();\n\
                  DFFR r1 ( .CK(clk), .RESET_B(rst_n), .D(q),  .Q(q1) );\n\
                  INV  g1 ( .A(q1), .Y(n1) );\n\
                  DFFR r2 ( .CK(clk), .RESET_B(rst_n), .D(n1), .Q(q) );\n\
                  endmodule";

fn job() -> StaJob {
    StaJob {
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 10.0,
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
        input_delay: 0.0,
        output_delay: 0.0,
        io_input_delays: vec![],
        io_output_delays: vec![],
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        scenarios: vec![],
        exceptions: vec![],
        async_groups: vec![],
        crpr: true,
        pba: false,
        base_dir: String::new(),
    }
}

#[test]
fn fillers_skipped_and_async_reset_not_data() {
    // must not error on the unknown, no-connection fill cell
    let rep = analyze_inputs(NL, LIB, &job()).expect("analyze (fill skipped)");
    // the RESET_B->Q 'clear' arc (delay 5.0) must NOT be on the worst data path —
    // the launch is the real CLK->Q reg-to-reg, not the async reset.
    assert!(
        !rep.worst_path.iter().any(|p| p.label.ends_with("/RESET_B")),
        "async reset must not be a data launch: {:?}",
        rep.worst_path
    );
    assert!(rep.wns > 0.0, "reg->reg meets at 10 ns: {}", rep.wns);
}
