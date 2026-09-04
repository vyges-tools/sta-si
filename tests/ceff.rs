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
        rc_model: "elmore".into(),
        input_delay_declared: true,
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

#[test]
fn a_driver_too_weak_to_shield_gets_no_invented_shielding() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let with_r = analyze(&nl, &lib, &job(), Some(&Spef::parse(SPEF_R))).unwrap();
    let lumped = analyze(&nl, &lib, &job(), Some(&Spef::parse(SPEF_LUMPED))).unwrap();
    // ⛔ This used to assert `with_r.wns > lumped.wns + 0.5` — strong shielding — and that
    // expectation came from `ceff_iter`, which **never sees the driver's resistance**: its
    // signature is `(c_near, c_far, tau, slew_at)`, so it shields by the wire's RC ratio
    // alone and predicts the same shielding for any driver.
    //
    // Effective capacitance depends on Rd. Shielding needs a driver FASTER than the wire;
    // this fixture's table runs 0.08 ns at 0.001 pF to 2.00 ns at 0.20 pF, a slope of
    // ~9.6 ns/pF, so `gateModelRd` puts Rd at about 6.7 kΩ against the net's 5 kΩ Rpi.
    // A driver that weak charges the far capacitance nearly as fast as it charges the near
    // one, and Ceff correctly approaches the total. The reference's `setCeffAlgorithm`
    // makes the same point at the other extreme: `rd < 1e-2` ⇒ the load is treated as a
    // lump, because "zero Rd means the table is constant and thus independent of load cap".
    //
    // So what this fixture asserts is that NO large shielding effect is invented for a
    // driver too weak to shield. The two runs may differ by the wire delay itself — the
    // distributed net really does add one — but not by the half-nanosecond a shielding
    // model would produce. ⚠️ A model that ignores Rd fails this: `ceff_iter` gave
    // `with_r` more than 0.5 ns of extra slack here.
    //
    // The magnitude case — a FAST driver behind a real resistance, where shielding is
    // genuine — is pinned in `vyges-loom`'s `dmp` tests, which set Rd explicitly rather
    // than leaving it implied by a table.
    assert!(
        (with_r.wns - lumped.wns).abs() < 0.01,
        "a driver this weak (Rd ~6.7k vs Rpi 5k) cannot shield, so the distributed and \
         lumped runs should differ only by the wire delay: with_r.wns={} lumped.wns={}",
        with_r.wns,
        lumped.wns
    );
}
