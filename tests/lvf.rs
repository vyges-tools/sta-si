// LVF/POCV: an arc's ocv_sigma_cell_rise/fall table gives a per-(slew,load) delay
// sigma. With pocv_sigma=0 (global POCV off), the presence of LVF auto-enables POCV
// and the band comes from the tables. Here sigma=0.05/stage over a 4-inverter chain
// -> var = 4·0.05² = 0.01, 3-sigma band = 3·0.1 = 0.30 ns added to the late path.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const SIGMA: &str = r#"
        ocv_sigma_cell_rise (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        ocv_sigma_cell_fall (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
"#;

fn lib(with_lvf: bool) -> String {
    let s = if with_lvf { SIGMA } else { "" };
    format!(
        r#"
library (v) {{
  cell (INV) {{
    pin (A) {{ direction : input; capacitance : 0.0015; }}
    pin (Y) {{
      direction : output;
      timing () {{
        related_pin : "A";
        timing_sense : negative_unate;
        cell_rise (t)       {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }}
        cell_fall (t)       {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }}
        rise_transition (t) {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }}
        fall_transition (t) {{ index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }}
{s}      }}
    }}
  }}
}}
"#
    )
}

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
        pocv_sigma: 0.0, // global POCV OFF — only LVF can enable it
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
fn lvf_tables_drive_pocv_band() {
    let nominal = analyze_inputs(NL, &lib(false), &job()).unwrap(); // no LVF -> no band
    let lvf = analyze_inputs(NL, &lib(true), &job()).unwrap(); // LVF -> 3-sigma band
                                                               // band = 3·sqrt(4·0.05²) = 0.30 ns shaved off the late slack
    let band = nominal.wns - lvf.wns;
    assert!(
        (band - 0.30).abs() < 0.02,
        "LVF 3-sigma band should be ~0.30 ns, got {band}"
    );
    assert!(
        lvf.wns < nominal.wns,
        "LVF must add pessimism: {} !< {}",
        lvf.wns,
        nominal.wns
    );
}
