// Path-based analysis catches a non-greedy worst path. Gate G's output O is fed by
// two arcs: I2->O arrives later (0.20) with a FAST output slew (0.05), I1->O
// arrives earlier (0.10) with a SLOW output slew (0.50). GBA keeps the later
// arrival (I2) and its fast slew, so the slew-sensitive next stage H looks cheap.
// But the I1 route, with its slow slew, blows up H's delay — the true worst path.
// GBA misses it (optimistic); PBA re-times the I1 alternative and finds it.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (p) {
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
        design: "p".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 3.0,
        clocks: vec![],
        input_slew: 0.02,
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
        crpr: true,
        pba,
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

#[test]
fn pba_catches_non_greedy_worst_path() {
    let rep = analyze_inputs(NL, LIB, &job(true)).unwrap();
    // GBA picks I2 at O (later arrival, fast slew) -> H looks cheap -> arrival ~0.30
    // -> WNS ~2.70. The real worst (via I1, slow slew -> H=2.0) is arrival ~2.10.
    assert!(rep.wns > 2.5, "GBA (optimistic) WNS should be ~2.70, got {}", rep.wns);
    let pba = rep.pba_wns.expect("PBA enabled");
    assert!((pba - 0.90).abs() < 0.05, "PBA WNS should be ~0.90 (arrival 2.10), got {pba}");
    assert!(pba < rep.wns - 1.5, "PBA must catch the optimism: pba {pba} vs gba {}", rep.wns);
}

#[test]
fn pba_off_by_default() {
    let rep = analyze_inputs(NL, LIB, &job(false)).unwrap();
    assert!(rep.pba_wns.is_none(), "PBA should be off");
}
