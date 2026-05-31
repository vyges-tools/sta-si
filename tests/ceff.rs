// CCS-into-RC: effective capacitance. A driver behind a resistive net sees less
// than the total cap (the far cap is shielded by the wire resistance), so its cell
// delay is smaller than with a lumped load. Here a 200 fF far cap sits behind a
// 5 kΩ resistor with the sink at the near node, so the shielding is large.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::spef::Spef;
use vyges_sta_si::sta::analyze;

// net n1 (u1 -> u2): near cap 2 fF at the driver pin, sink u2/A near the driver via
// a 1 Ω link, and a 200 fF cap on a dangling branch behind a 5 kΩ resistor.
const SPEF_R: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*3 u1
*4 u2
*D_NET *1 202.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *3:Y 2.000000
2 *fcap 200.000000
*RES
1 *3:Y *4:A 1.000000
2 *3:Y *fcap 5000.000000
*END
"#;

// same caps, NO resistors -> lumped load (pi_reduce returns None)
const SPEF_LUMPED: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*3 u1
*4 u2
*D_NET *1 202.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *3:Y 2.000000
2 *fcap 200.000000
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
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.20"); values ( "0.08, 2.00", "0.12, 2.40" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.20"); values ( "0.07, 1.80", "0.11, 2.20" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.04, 0.40" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.04, 0.40" ); }
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
        period_ns: 5.0,
        clocks: vec![],
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 1.0, // disable SI so we isolate the Ceff effect
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions: vec![],
        crpr: true,
        base_dir: String::new(),
    }
}

#[test]
fn ceff_shields_far_cap_and_cuts_driver_delay() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let with_r = analyze(&nl, &lib, &job(), Some(&Spef::parse(SPEF_R))).unwrap();
    let lumped = analyze(&nl, &lib, &job(), Some(&Spef::parse(SPEF_LUMPED))).unwrap();
    // resistive shielding -> driver drives ~near cap -> much smaller cell delay ->
    // more slack than driving the full 202 fF lumped.
    assert!(
        with_r.wns > lumped.wns + 0.5,
        "Ceff should cut driver delay: with_r.wns={} lumped.wns={}",
        with_r.wns,
        lumped.wns
    );
}
