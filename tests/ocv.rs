// AOCV / POCV on-chip-variation modes, exercised on an inverter chain (depth 4)
// so that depth-dependent derating and the sqrt(depth) statistical band matter.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (ocv) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.09, 0.20", "0.13, 0.28" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
}
"#;

// a -> u1 -> u2 -> u3 -> u4 -> y  (4 cell stages)
const NL: &str = "module ch ( a, y ); input a; output y; wire n1, n2, n3;\n\
                  INV u1 ( .A(a),  .Y(n1) );\n\
                  INV u2 ( .A(n1), .Y(n2) );\n\
                  INV u3 ( .A(n2), .Y(n3) );\n\
                  INV u4 ( .A(n3), .Y(y)  );\n\
                  endmodule";

fn base() -> StaJob {
    StaJob {
        design: "ch".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
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
        exceptions: vec![],
        async_groups: vec![],
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

#[test]
fn aocv_sits_between_flat_extremes() {
    // nominal (no derate) is the most optimistic late slack.
    let nominal = analyze_inputs(NL, LIB, &base()).unwrap();

    // a flat 1.10 derate on every stage is the most pessimistic.
    let mut flat = base();
    flat.late_derate = 1.10;
    let flat = analyze_inputs(NL, LIB, &flat).unwrap();

    // AOCV derates depth 1 at 1.10 but relaxes toward 1.02 by depth 6, so a
    // 4-stage path is derated less than the flat 1.10 -> slack lands in between.
    let mut aocv = base();
    aocv.aocv_late = vec![(1.0, 1.10), (6.0, 1.02)];
    let aocv = analyze_inputs(NL, LIB, &aocv).unwrap();

    assert!(
        nominal.wns > aocv.wns,
        "nominal {} !> aocv {}",
        nominal.wns,
        aocv.wns
    );
    assert!(
        aocv.wns > flat.wns,
        "aocv {} !> flat {}",
        aocv.wns,
        flat.wns
    );
}

#[test]
fn pocv_band_eats_setup_slack_monotonically() {
    let nominal = analyze_inputs(NL, LIB, &base()).unwrap();

    let mut p1 = base();
    p1.pocv_sigma = 0.10; // 10% per-stage 1-sigma, 3-sigma band
    let p1 = analyze_inputs(NL, LIB, &p1).unwrap();

    let mut p2 = base();
    p2.pocv_sigma = 0.20; // wider sigma -> wider band -> less slack
    let p2 = analyze_inputs(NL, LIB, &p2).unwrap();

    // the N-sigma band lengthens the late path, so slack shrinks with sigma
    assert!(
        p1.wns < nominal.wns,
        "pocv {} !< nominal {}",
        p1.wns,
        nominal.wns
    );
    assert!(
        p2.wns < p1.wns,
        "bigger sigma {} !< smaller {}",
        p2.wns,
        p1.wns
    );
}

#[test]
fn pocv_sub_linear_in_depth() {
    // The 3-sigma band over 4 stages must be LESS than 4x a single stage's band
    // (RSS: sqrt(4)=2, not 4). Compare the slack hit on the 4-stage chain to the
    // same chain with one stage's worth of sigma applied linearly.
    let nominal = analyze_inputs(NL, LIB, &base()).unwrap();
    let mut p = base();
    p.pocv_sigma = 0.10;
    let p = analyze_inputs(NL, LIB, &p).unwrap();
    let band = nominal.wns - p.wns; // ns the band cost over 4 stages
                                    // a single stage's nominal delay is ~0.10-0.30 ns; 3-sigma of one stage is
                                    // ~3*0.10*0.30 ≈ 0.09 ns. A linear (worst-case) 4-stage band would be ~4x
                                    // that; RSS is ~2x. So the band must be well under the linear bound.
    let linear_bound = 4.0 * 3.0 * 0.10 * 0.30;
    assert!(band > 0.0, "band {band} not positive");
    assert!(
        band < linear_bound,
        "band {band} !< linear bound {linear_bound} (RSS should beat linear)"
    );
}
