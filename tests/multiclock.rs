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
const NL: &str =
    "module mc ( clk1, clk2, din, dout ); input clk1, clk2, din; output dout; wire q1, n1;\n\
                  DFF r1 ( .CK(clk1), .D(din), .Q(q1) );\n\
                  INV g1 ( .A(q1),    .Y(n1) );\n\
                  DFF r2 ( .CK(clk2), .D(n1),  .Q(dout) );\n\
                  endmodule";

fn job(clocks: Vec<(String, String, f64)>) -> StaJob {
    StaJob {
        input_delay_declared: true,
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
        metadata: None,
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
    assert_eq!(
        cross.worst_endpoint, "r2/D",
        "cross worst {}",
        cross.worst_endpoint
    );
    // required = 0 + 2.0 - setup(0.05); arrival ~0.21 -> slack ~1.74, well under 2.0
    assert!(
        cross.wns > 1.3 && cross.wns < 1.95,
        "cross-domain wns={} (expect ~1.74)",
        cross.wns
    );
}

#[test]
fn cross_domain_is_tighter_than_either_single_domain() {
    let cross = analyze_inputs(NL, LIB, &job(vec![ck("clk1", 10.0), ck("clk2", 4.0)])).unwrap();
    // same netlist as one 10 ns domain (clk2 unknown -> falls back to primary 10 ns)
    let one10 = analyze_inputs(NL, LIB, &job(vec![])).unwrap();
    // ... and as a single 4 ns domain on both
    let one4 = analyze_inputs(NL, LIB, &job(vec![ck("clk1", 4.0), ck("clk2", 4.0)])).unwrap();
    // 2 ns relation is tighter than 10 ns and even tighter than 4 ns
    assert!(
        cross.wns < one10.wns - 5.0,
        "cross {} should be << 10ns-domain {}",
        cross.wns,
        one10.wns
    );
    assert!(
        cross.wns < one4.wns,
        "cross {} should be < 4ns-domain {}",
        cross.wns,
        one4.wns
    );
}

// A CLOCK DECLARED ON A HIERARCHICAL PIN. Synthesis flattens `core/u_div` into one escaped
// identifier, so a generated clock on that divider's output is `core/u_div/Q` — and every SDC
// writes it that way. Resolving it needs the split at the LAST separator; at the first it names
// instance `core`, matches no node, and the clock quietly never attaches to anything.
const NL_HIER: &str =
    "module mc ( clk1, din, dout ); input clk1, din; output dout; wire clk2, dq, q1, n1;\n\
     DFF \\core/u_div ( .CK(clk1), .D(dq),  .Q(clk2) );\n\
     INV \\core/g0 ( .A(clk2), .Y(dq) );\n\
     DFF \\core/r1 ( .CK(clk1), .D(din), .Q(q1) );\n\
     INV \\core/g1 ( .A(q1),   .Y(n1) );\n\
     DFF \\core/r2 ( .CK(clk2), .D(n1),  .Q(dout) );\n\
     endmodule";

#[test]
fn a_clock_declared_on_a_hierarchical_pin_actually_applies() {
    // The whole test is: does declaring it change anything? A clock that fails to resolve is
    // not an error — every flop on it silently falls back to the primary period, so the report
    // looks complete and the periods in it are not the ones that were asked for.
    let fast = analyze_inputs(
        NL_HIER,
        LIB,
        &job(vec![
            ck("clk1", 10.0),
            ("clk2".into(), "core/u_div/Q".into(), 4.0),
        ]),
    )
    .unwrap();
    let slow = analyze_inputs(
        NL_HIER,
        LIB,
        &job(vec![
            ck("clk1", 10.0),
            ("clk2".into(), "core/u_div/Q".into(), 20.0),
        ]),
    )
    .unwrap();
    assert!(
        (fast.wns - slow.wns).abs() > 1.0,
        "the declared period must reach the flops it clocks: wns {} vs {}",
        fast.wns,
        slow.wns
    );
}

#[test]
fn async_clock_groups_apply_to_a_hierarchically_named_clock_source() {
    let clocks = vec![
        ck("clk1", 10.0),
        ("clk2".into(), "core/u_div/Q".into(), 4.0),
    ];
    let timed = analyze_inputs(NL_HIER, LIB, &job(clocks.clone())).unwrap();
    assert_eq!(
        timed.worst_endpoint, "core/r2/D",
        "the cross-domain path is the worst while it is still timed"
    );

    let mut grouped = job(clocks);
    grouped.async_groups = vec![vec!["clk1".into()], vec!["clk2".into()]];
    let cut = analyze_inputs(NL_HIER, LIB, &grouped).unwrap();
    assert_ne!(
        cut.worst_endpoint, "core/r2/D",
        "declared asynchronous, the cross-domain setup check is cut"
    );
    assert_eq!(
        cut.endpoints,
        timed.endpoints - 1,
        "exactly one endpoint leaves the setup analysis"
    );
}
