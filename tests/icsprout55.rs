// Regression: the committed icsprout55 (open 55nm) example must keep analyzing.
// This pins our first sub-100nm node into the suite — real foundry NLDM cells
// (DFFQX1H7R, INVX1H7R), register-to-register setup + hold.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = include_str!("../examples/icsprout55/ics55_LLSC_H7CR_inv_dff.lib");
const NL: &str = include_str!("../examples/icsprout55/regreg.v");

fn job() -> StaJob {
    StaJob {
        design: "ics55_regreg".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
        input_slew: 0.05,
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
fn icsprout55_reg_to_reg_setup_and_hold() {
    let rep = analyze_inputs(NL, LIB, &job()).unwrap();
    // 2 flop D pins + 1 primary output
    assert_eq!(rep.endpoints, 3, "endpoints={}", rep.endpoints);
    assert_eq!(rep.hold_endpoints, 2, "hold_endpoints={}", rep.hold_endpoints);
    // values measured against the foundry NLDM at the typical corner, 2 ns clock,
    // with slew-interpolated setup/hold constraints (OpenSTA-correlated)
    assert!((rep.wns - 1.8273).abs() < 0.01, "setup WNS drifted: {}", rep.wns);
    assert!((rep.whs - 0.1113).abs() < 0.005, "hold WHS drifted: {}", rep.whs);
    // both must meet
    assert!(rep.wns > 0.0 && rep.whs > 0.0, "55nm path should meet: wns={} whs={}", rep.wns, rep.whs);
    // setup launched by a real flop Q (CK->Q arc)
    assert!(rep.worst_path.iter().any(|p| p.label.ends_with("/Q")), "{:?}", rep.worst_path);
}

#[test]
fn icsprout55_pocv_shrinks_hold_margin() {
    // at 55nm the hold margin is tiny; a 3-sigma POCV band must shrink it further.
    let flat = analyze_inputs(NL, LIB, &job()).unwrap();
    let mut p = job();
    p.pocv_sigma = 0.06;
    let pocv = analyze_inputs(NL, LIB, &p).unwrap();
    assert!(pocv.whs < flat.whs, "POCV hold {} !< flat {}", pocv.whs, flat.whs);
    assert!(pocv.whs > 0.0, "still meets: {}", pocv.whs);
}
