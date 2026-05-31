use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

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

fn job(period: f64) -> StaJob {
    StaJob {
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: period,
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        base_dir: String::new(),
    }
}

#[test]
fn two_inverter_chain_timing() {
    let rep = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    assert_eq!(rep.endpoints, 1);
    // worst path: a -> u1/A -> u1/Y -> u2/A -> u2/Y -> y
    assert_eq!(rep.worst_path.first().unwrap().label, "a");
    assert_eq!(rep.worst_path.last().unwrap().label, "y");
    assert_eq!(rep.worst_path.len(), 6);
    // arrival at y is two inverter stages (~0.25 ns); slack = period - arrival
    let arr_y = rep.worst_path.last().unwrap().arrival;
    assert!(arr_y > 0.15 && arr_y < 0.40, "arrival={arr_y}");
    assert!((rep.wns - (1.0 - arr_y)).abs() < 1e-9);
    assert!(rep.wns > 0.0); // 1 ns period easily met
}

#[test]
fn tight_period_violates() {
    let rep = analyze_inputs(NL, LIB, &job(0.1)).unwrap(); // 100 ps: too tight
    assert!(rep.wns < 0.0, "wns={}", rep.wns);
    assert!(rep.tns < 0.0);
}

#[test]
fn derate_increases_delay() {
    let base = analyze_inputs(NL, LIB, &job(1.0)).unwrap();
    let mut j = job(1.0);
    j.late_derate = 1.5;
    let der = analyze_inputs(NL, LIB, &j).unwrap();
    // larger late derate -> larger arrival -> smaller slack
    assert!(der.wns < base.wns);
}
