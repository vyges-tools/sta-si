// Path-based analysis removes graph-based pessimism. Gate G's output O is fed by two
// arcs: I2->O arrives later (0.20) with a FAST output slew (0.05), I1->O arrives
// earlier (0.10) with a SLOW output slew (0.50). GBA, as the reference times it, merges
// O's arrival AND its slew to the worst over the arcs, each on its own
// (`annotateDelaySlew`): arrival 0.20 with slew 0.50, so H takes 2.0 — a path no input
// actually has (arrival 2.20, WNS 0.80). PBA re-times each path with its own slews: the
// real worst is I1 (0.10 + 2.0 = 2.10, WNS 0.90).
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (p) {
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (G) {
    pin (I1) { direction : input; capacitance : 0.001; }
    pin (I2) { direction : input; capacitance : 0.001; }
    pin (O) {
      direction : output;
      timing () {
        related_pin : "I1";
        timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.50, 0.50", "0.50, 0.50" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.50, 0.50", "0.50, 0.50" ); }
      }
      timing () {
        related_pin : "I2";
        timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.20, 0.20", "0.20, 0.20" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.20, 0.20", "0.20, 0.20" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
      }
    }
  }
  cell (H) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.05, 0.50"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "2.00, 2.00" ); }
        cell_fall (t)       { index_1 ("0.05, 0.50"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "2.00, 2.00" ); }
        rise_transition (t) { index_1 ("0.05, 0.50"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.05, 0.50"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
      }
    }
  }
}
"#;

const NL: &str = "module top ( a1, a2, z ); input a1, a2; output z; wire o;\n\
                  G g1 ( .I1(a1), .I2(a2), .O(o) );\n\
                  H h1 ( .A(o), .Y(z) );\n\
                  endmodule";

fn job(pba: bool) -> StaJob {
    StaJob {
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "p".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 3.0,
        clocks: vec![],
        input_slew: 0.02,
        input_slews: Vec::new(),
        output_load: 0.005,
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
        driving_cells: Default::default(),
        crpr: true,
        pba,
        input_delay: 0.0,
        output_delay: 0.0,
        output_delay_declared: false,
        unconstrained_outputs: false,
        io_input_delays: vec![],
        io_output_delays: vec![],
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        metadata: None,
        base_dir: String::new(),
    }
}

#[test]
fn pba_removes_the_merged_slew_pessimism() {
    let rep = analyze_inputs(NL, LIB, &job(true)).unwrap();
    assert!((rep.wns - 0.80).abs() < 1e-6, "GBA: latest arrival with the worst slew, got {}", rep.wns);
    let pba = rep.pba_wns.expect("PBA enabled");
    assert!((pba - 0.90).abs() < 0.05, "PBA WNS should be ~0.90 (arrival 2.10), got {pba}");
    assert!(pba >= rep.wns, "PBA is never more pessimistic than GBA: pba {pba} vs gba {}", rep.wns);
}

#[test]
fn pba_off_by_default() {
    let rep = analyze_inputs(NL, LIB, &job(false)).unwrap();
    assert!(rep.pba_wns.is_none(), "PBA should be off");
}
