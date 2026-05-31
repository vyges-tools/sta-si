// Unate (rise/fall-split) propagation. With a strongly asymmetric inverter
// (rise 0.30, fall 0.05), a chain's true delay ALTERNATES edges (negative_unate:
// out-rise from in-fall), so 4 stages ≈ 0.30+0.05+0.30+0.05 = 0.70 — far below the
// naive max-per-stage 4×0.30 = 1.20. A non_unate cell can't alternate and hits the
// pessimistic 1.20. Flat (slew-independent) tables make the arithmetic exact.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

fn lib(sense: &str) -> String {
    format!(
        r#"
library (u) {{
  cell (INV) {{
    pin (A) {{ direction : input; capacitance : 0.0015; }}
    pin (Y) {{
      direction : output;
      timing () {{
        related_pin : "A";
        timing_sense : {sense};
        cell_rise (t)       {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.30, 0.30", "0.30, 0.30" ); }}
        cell_fall (t)       {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }}
        rise_transition (t) {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }}
        fall_transition (t) {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }}
      }}
    }}
  }}
}}
"#
    )
}

// a -> u1 -> u2 -> u3 -> u4 -> y  (4 inverter stages)
const NL: &str = "module ch ( a, y ); input a; output y; wire n1, n2, n3;\n\
                  INV u1 ( .A(a),  .Y(n1) );\n\
                  INV u2 ( .A(n1), .Y(n2) );\n\
                  INV u3 ( .A(n2), .Y(n3) );\n\
                  INV u4 ( .A(n3), .Y(y)  );\n\
                  endmodule";

fn job() -> StaJob {
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
        crpr: true,
        pba: false,
        base_dir: String::new(),
    }
}

#[test]
fn negative_unate_chain_alternates_edges() {
    let rep = analyze_inputs(NL, &lib("negative_unate"), &job()).unwrap();
    // period 2.0; arrival = 2.0 - WNS. The alternating path is 0.70 ns, NOT 1.20.
    let arrival = 2.0 - rep.wns;
    assert!((arrival - 0.70).abs() < 1e-6, "alternating chain arrival should be 0.70, got {arrival}");
}

#[test]
fn unate_beats_non_unate_pessimism() {
    let neg = analyze_inputs(NL, &lib("negative_unate"), &job()).unwrap();
    let non = analyze_inputs(NL, &lib("non_unate"), &job()).unwrap();
    // non_unate cannot alternate -> hits 4×0.30 = 1.20; negative_unate alternates -> 0.70.
    let non_arr = 2.0 - non.wns;
    assert!((non_arr - 1.20).abs() < 1e-6, "non_unate should be 1.20, got {non_arr}");
    assert!(neg.wns > non.wns + 0.4, "unate must beat non_unate pessimism: {} vs {}", neg.wns, non.wns);
}
