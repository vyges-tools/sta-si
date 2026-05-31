use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::spef::Spef;
use vyges_sta_si::sta::analyze;

const SPEF: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*2 n2
*3 u1
*4 u2
*D_NET *1 20.000000
*CONN
*I *3:Y O
*CAP
1 *3:Y 10.000000
2 *1 10.000000
3 *1 *2 5.000000
*RES
1 *1 *3:Y 500.000000
2 *1 *4:A 500.000000
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
        period_ns: 1.0,
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        miller: 2.0,
        base_dir: String::new(),
    }
}

#[test]
fn parses_total_cap_and_summed_res() {
    let s = Spef::parse(SPEF);
    let n1 = s.nets.get("n1").unwrap();
    assert!((n1.cap_ff - 20.0).abs() < 1e-9); // *D_NET total
    assert!((n1.res_ohm - 1000.0).abs() < 1e-9); // 500 + 500 summed *RES
    assert!((s.wire_load_pf("n1") - 0.020).abs() < 1e-9); // fF -> pF
    // Elmore = R*C -> ns : 1000 * 20 * 1e-6 = 0.02 ns
    assert!((s.net_delay_ns("n1") - 0.02).abs() < 1e-9);
    assert!((n1.coupling_ff - 5.0).abs() < 1e-9); // two-node *CAP entry
}

#[test]
fn crosstalk_reduces_slack_vs_quiet() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let spef = Spef::parse(SPEF);
    let mut j = job();
    j.miller = 2.0; // worst-case aggressor
    let with_si = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    j.miller = 1.0; // quiet aggressor -> coupling acts as plain ground (no extra)
    let quiet = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    assert!(with_si.wns < quiet.wns, "SI {} !< quiet {}", with_si.wns, quiet.wns);
}

#[test]
fn spef_adds_delay_and_load_reducing_slack() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job();
    let ideal = analyze(&nl, &lib, &j, None).unwrap();
    let withrc = analyze(&nl, &lib, &j, Some(&Spef::parse(SPEF))).unwrap();
    // parasitics (net delay + wire load) increase arrival -> less slack
    assert!(withrc.wns < ideal.wns, "withrc {} !< ideal {}", withrc.wns, ideal.wns);
}
