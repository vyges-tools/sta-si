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
*5 u3
*D_NET *1 20.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *3:Y 10.000000
2 *1 10.000000
3 *1 *2 5.000000
*RES
1 *1 *3:Y 500.000000
2 *1 *4:A 500.000000
*END
*D_NET *2 20.000000
*CONN
*I *4:Y O
*I *5:A I
*CAP
1 *4:Y 10.000000
2 *2 10.000000
*RES
1 *2 *4:Y 500.000000
2 *2 *5:A 500.000000
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

// 3-inverter chain: n1 (driver u1) and n2 (driver u2) both exist and switch at
// different times, so window-aware coupling between them is testable.
const NL: &str = "module top ( a, y ); input a; output y; wire n1, n2;\n\
                  INV u1 ( .A(a), .Y(n1) ); INV u2 ( .A(n1), .Y(n2) );\n\
                  INV u3 ( .A(n2), .Y(y) ); endmodule";

fn job() -> StaJob {
    StaJob {
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 1.0,
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
        xtalk_window: 0.2,
        scenarios: vec![],
        exceptions: vec![],
        crpr: true,
        pba: false,
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
    j.xtalk_window = 1.0; // wide window so the aggressor is in scope (test the Miller effect)
    j.miller = 2.0; // worst-case aggressor
    let with_si = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    j.miller = 1.0; // quiet aggressor -> coupling acts as plain ground (no extra)
    let quiet = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    assert!(with_si.wns < quiet.wns, "SI {} !< quiet {}", with_si.wns, quiet.wns);
}

#[test]
fn per_pin_elmore_differentiates_sinks() {
    use vyges_sta_si::spef::NetRc;
    // D --10Ω--> C ; C --5Ω--> s1 (near) ; C --50Ω--> s2 (far). Caps C=1, s1=2, s2=3 fF.
    let rc = NetRc {
        net_node: "C".into(),
        res: vec![
            ("D".into(), "C".into(), 10.0),
            ("C".into(), "s1".into(), 5.0),
            ("C".into(), "s2".into(), 50.0),
        ],
        ground: vec![("C".into(), 1.0), ("s1".into(), 2.0), ("s2".into(), 3.0)],
        ..Default::default()
    };
    let d = rc.elmore("D", 0.0).unwrap();
    // s1 = 10·6 + 5·2 = 70 (×1e-6 ns) ; s2 = 10·6 + 50·3 = 210 (×1e-6 ns)
    assert!((d["s1"] - 7e-5).abs() < 1e-12, "s1={}", d["s1"]);
    assert!((d["s2"] - 2.1e-4).abs() < 1e-12, "s2={}", d["s2"]);
    assert!(d["s2"] > d["s1"]); // far sink is slower — the whole point of per-pin
    // a crosstalk cap at the net node raises every downstream sink
    let dx = rc.elmore("D", 10.0).unwrap();
    assert!(dx["s1"] > d["s1"] && dx["s2"] > d["s2"]);
}

#[test]
fn window_filters_non_overlapping_aggressors() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let spef = Spef::parse(SPEF);
    // n1 and n2 (coupled) switch ~0.1 ns apart in the chain.
    let mut j = job();
    j.miller = 2.0;
    j.xtalk_window = 1.0; // wide -> windows overlap -> crosstalk counted
    let wide = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    j.xtalk_window = 0.0; // narrow -> no overlap -> crosstalk filtered out
    let narrow = analyze(&nl, &lib, &j, Some(&spef)).unwrap();
    assert!(wide.wns < narrow.wns, "wide {} !< narrow {}", wide.wns, narrow.wns);
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
