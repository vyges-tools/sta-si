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
    lib_with(if with_lvf { SIGMA } else { "" })
}

/// The inverter library with `s` (LVF groups) inside its timing group.
fn lib_with(s: &str) -> String {
    format!(
        r#"
library (v) {{
  lu_table_template (t) {{ variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }}
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
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "ch".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
        clocks: vec![],
        input_slew: 0.02,
        input_slews: Vec::new(),
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
        clock_groups: vec![],
        ideal_clocks: Default::default(),
        driving_cells: Default::default(),
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
        output_delay_declared: false,
        unconstrained_outputs: false,
        io_input_delays: vec![],
        io_output_delays: vec![],
        io_input_all_but: vec![],
        io_output_all_but: vec![],
        case_values: Vec::new(),
        disabled_timing: Vec::new(),
        data_check_cmds: Vec::new(),
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        metadata: None,
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

// Early and late sigma apart. Reference rule (Liberty reader `readEarlyLateTableModels`, then
// `GateTableModel::gateDelayPocv` in normal mode): each `ocv_sigma_cell_*` group sets the half
// its `sigma_type` names; a max (setup) path takes the LATE sigma, a min (hold) path the EARLY
// one; an `ocv_std_dev_cell_*` table, when given, is the standard deviation for both. The early
// group is written FIRST here, so a reader that kept only the first group would give the setup
// path the hold path's sigma.
const EARLY_LATE: &str = r#"
        ocv_sigma_cell_rise (t) { sigma_type : "early"; index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.01, 0.01", "0.01, 0.01" ); }
        ocv_sigma_cell_rise (t) { sigma_type : "late"; index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        ocv_sigma_cell_fall (t) { sigma_type : "early"; index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.01, 0.01", "0.01, 0.01" ); }
        ocv_sigma_cell_fall (t) { sigma_type : "late"; index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
"#;

const STD_DEV: &str = r#"
        ocv_std_dev_cell_rise (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
        ocv_std_dev_cell_fall (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.02, 0.02", "0.02, 0.02" ); }
"#;

fn bands(tables: &str) -> (f64, f64) {
    let mut j = job();
    j.output_delay_declared = true; // an output delay of 0 checks setup AND hold at y
    let nominal = analyze_inputs(NL, &lib(false), &j).unwrap();
    let with = analyze_inputs(NL, &lib_with(tables), &j).unwrap();
    (nominal.wns - with.wns, nominal.whs - with.whs)
}

#[test]
fn setup_takes_late_sigma_and_hold_takes_early_sigma() {
    // four stages: late band 3·sqrt(4·0.05²) = 0.30, early band 3·sqrt(4·0.01²) = 0.06
    let (setup, hold) = bands(EARLY_LATE);
    assert!((setup - 0.30).abs() < 1e-6, "setup band {setup}, want 0.30 (late sigma)");
    assert!((hold - 0.06).abs() < 1e-6, "hold band {hold}, want 0.06 (early sigma)");
}

#[test]
fn std_dev_is_the_sigma_of_both_analyses() {
    // 3·sqrt(4·0.02²) = 0.12 on both sides, early/late sigma or not
    let (setup, hold) = bands(&format!("{EARLY_LATE}{STD_DEV}"));
    assert!((setup - 0.12).abs() < 1e-6, "setup band {setup}, want 0.12 (std_dev)");
    assert!((hold - 0.12).abs() < 1e-6, "hold band {hold}, want 0.12 (std_dev)");
}
