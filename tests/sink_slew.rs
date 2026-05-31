// Sink-slew degradation: a sink behind a resistive net gets a much slower edge
// than the driver produced (RC degrades the slew). The transient computes it; the
// engine must thread it onto the sink node, not pass the driver slew through.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::spef::Spef;
use vyges_sta_si::sta::analyze;

// net n1 (u1 -> u2): 10 kΩ to a 100 fF sink -> τ = 1.0 ns, so the 30-70% slew at
// u2/A is ~0.85 ns — far slower than the driver's ~0.04 ns edge.
const SPEF: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*3 u1
*4 u2
*D_NET *1 100.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *4:A 100.000000
*RES
1 *3:Y *4:A 10000.000000
*END
"#;

const LIB: &str = r#"
library (d) {
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
}
"#;

const NL: &str = "module top ( a, y ); input a; output y; wire n1;\n\
                  INV u1 ( .A(a), .Y(n1) ); INV u2 ( .A(n1), .Y(y) ); endmodule";

fn job() -> StaJob {
    StaJob {
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 5.0,
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
        pba: false,
        base_dir: String::new(),
    }
}

fn sink_slew(rep: &vyges_sta_si::sta::TimingReport) -> f64 {
    rep.worst_path.iter().find(|p| p.label == "u2/A").map(|p| p.slew).expect("u2/A on path")
}

#[test]
fn rc_degrades_sink_slew() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let with_rc = analyze(&nl, &lib, &job(), Some(&Spef::parse(SPEF))).unwrap();
    let ideal = analyze(&nl, &lib, &job(), None).unwrap();

    let s_rc = sink_slew(&with_rc);
    let s_ideal = sink_slew(&ideal);
    // RC degrades the sink edge to ~0.85 ns (vs the driver's ~0.04 ns passed through
    // in the ideal case)
    assert!(s_rc > 0.3, "RC sink slew should be heavily degraded, got {s_rc}");
    assert!(s_rc > s_ideal + 0.2, "RC slew {s_rc} should exceed ideal {s_ideal}");
}
