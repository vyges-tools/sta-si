// End-to-end: when an arc carries CCS output-current waveforms, the engine uses
// the current-source delay instead of the NLDM tables. The CCS waveform here is a
// constant 1 mA over 0..1 ns -> V(t) ramps linearly -> 50% at t=0.5 ns, so the
// inverter delay is 0.5 ns regardless of load — deliberately different from the
// 0.1 ns NLDM tables, so we can tell which path ran.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const NLDM: &str = r#"
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
"#;

const CCS_BLOCKS: &str = r#"
        output_current_rise () {
          vector (ccs) {
            reference_time : 0.0 ;
            index_1 ("0.01"); index_2 ("0.005");
            index_3 ("0.0, 0.25, 0.5, 0.75, 1.0");
            values  ("1.0, 1.0, 1.0, 1.0, 1.0");
          }
        }
        output_current_fall () {
          vector (ccs) {
            reference_time : 0.0 ;
            index_1 ("0.01"); index_2 ("0.005");
            index_3 ("0.0, 0.25, 0.5, 0.75, 1.0");
            values  ("1.0, 1.0, 1.0, 1.0, 1.0");
          }
        }
"#;

fn lib(with_ccs: bool) -> String {
    let ccs = if with_ccs { CCS_BLOCKS } else { "" };
    format!(
        r#"
library (c) {{
  cell (INV) {{
    pin (A) {{ direction : input; capacitance : 0.0015; }}
    pin (Y) {{
      direction : output;
      timing () {{
        related_pin : "A";
        timing_sense : negative_unate;
{NLDM}{ccs}
      }}
    }}
  }}
}}
"#
    )
}

const NL: &str = "module c ( a, y ); input a; output y; INV u1 ( .A(a), .Y(y) ); endmodule";

fn job() -> StaJob {
    StaJob {
        design: "c".into(),
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
fn engine_uses_ccs_when_present() {
    let nldm = analyze_inputs(NL, &lib(false), &job()).unwrap();
    let ccs = analyze_inputs(NL, &lib(true), &job()).unwrap();
    // single endpoint y; arrival = inverter delay; required = period 2.0
    let nldm_arr = 2.0 - nldm.wns;
    let ccs_arr = 2.0 - ccs.wns;
    assert!(
        (nldm_arr - 0.10).abs() < 1e-6,
        "NLDM inverter delay should be 0.10, got {nldm_arr}"
    );
    assert!(
        (ccs_arr - 0.50).abs() < 1e-6,
        "CCS inverter delay should be 0.50, got {ccs_arr}"
    );
}
