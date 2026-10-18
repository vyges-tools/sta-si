// Regression: the committed icsprout55 (open 55nm) example must keep analyzing.
// This pins our first sub-100nm node into the suite — real foundry NLDM cells
// (DFFQX1H7R, INVX1H7R), register-to-register setup + hold.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = include_str!("../examples/icsprout55/ics55_LLSC_H7CR_inv_dff.lib");
const NL: &str = include_str!("../examples/icsprout55/regreg.v");

fn job() -> StaJob {
    StaJob {
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "ics55_regreg".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
        clocks: vec![],
        input_slew: 0.05,
        input_slews: Vec::new(),
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
        clock_groups: vec![],
        ideal_clocks: Default::default(),
        driving_cells: Default::default(),
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
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
fn icsprout55_reg_to_reg_setup_and_hold() {
    let rep = analyze_inputs(NL, LIB, &job()).unwrap();
    // 2 flop D pins + 1 primary output
    assert_eq!(rep.endpoints, 3, "endpoints={}", rep.endpoints);
    assert_eq!(
        rep.hold_endpoints, 2,
        "hold_endpoints={}",
        rep.hold_endpoints
    );
    // values measured against the foundry NLDM at the typical corner, 2 ns clock,
    // with slew-interpolated setup/hold constraints (OpenSTA-correlated)
    assert!(
        (rep.wns - 1.8273).abs() < 0.01,
        "setup WNS drifted: {}",
        rep.wns
    );
    assert!(
        (rep.whs - 0.1113).abs() < 0.005,
        "hold WHS drifted: {}",
        rep.whs
    );
    // both must meet
    assert!(
        rep.wns > 0.0 && rep.whs > 0.0,
        "55nm path should meet: wns={} whs={}",
        rep.wns,
        rep.whs
    );
    // setup launched by a real flop Q (CK->Q arc)
    assert!(
        rep.worst_path.iter().any(|p| p.label.ends_with("/Q")),
        "{:?}",
        rep.worst_path
    );
}

#[test]
fn icsprout55_pocv_shrinks_hold_margin() {
    // at 55nm the hold margin is tiny; a 3-sigma POCV band must shrink it further.
    let flat = analyze_inputs(NL, LIB, &job()).unwrap();
    let mut p = job();
    p.pocv_sigma = 0.06;
    let pocv = analyze_inputs(NL, LIB, &p).unwrap();
    assert!(
        pocv.whs < flat.whs,
        "POCV hold {} !< flat {}",
        pocv.whs,
        flat.whs
    );
    assert!(pocv.whs > 0.0, "still meets: {}", pocv.whs);
}

// Rule: a check exists for ONE data edge (the reference matches the check arc's edge to the
// path end's), so the reported path is that edge's own, followed back lane by lane — through an
// inverting arc onto the other input edge. Printing the earliest lane at each node instead put
// the rising-at-D path (r1/Q fall 0.0862, slack 0.1118) beside the falling-at-D path's slack
// 0.1105. Reference at the pin, same job as Tcl: r1/Q ^ 0.1122 (slew 0.0213), g1/Y v 0.1264
// (slew 0.0127), slack 0.1105.
#[test]
fn icsprout55_hold_path_is_the_edge_that_set_the_slack() {
    let rep = analyze_inputs(NL, LIB, &job()).unwrap();
    let at = |pin: &str| rep.worst_hold_path.iter().find(|p| p.label == pin).map(|p| (p.arrival, p.slew)).unwrap();
    let r4 = |x: f64| (x * 1e4).round() / 1e4;
    assert_eq!(rep.worst_hold_endpoint, "r2/D");
    assert_eq!(r4(rep.whs), 0.1105);
    let (q, qs) = at("r1/Q");
    let (y, ys) = at("g1/Y");
    assert_eq!((r4(q), r4(qs)), (0.1122, 0.0213));
    assert_eq!((r4(y), r4(ys)), (0.1264, 0.0127));
    // one edge's path never gets earlier along its own nets
    assert!(rep.worst_hold_path.windows(2).all(|w| w[1].arrival >= w[0].arrival - 1e-12));
}
