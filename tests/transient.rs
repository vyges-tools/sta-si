// Waveform-into-RC: a single RC (R=10kΩ, C=100fF -> τ=1.0 ns) driven by a fast
// edge has a 50% delay of 0.69·RC = 0.693 ns — the true step response — versus
// Elmore's first-moment R·C = 1.0 ns. The transient solver must hit 0.69, and be
// below Elmore.
use vyges_sta_si::spef::Spef;

const SPEF: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*3 u1
*4 u2
*D_NET *1 100.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *4:A 100.000000
*RES
1 *3:Y *4:A 10000.000000
*END
"#;

#[test]
fn single_rc_step_response_is_069_rc() {
    let spef = Spef::parse(SPEF);
    let rc = spef.nets.get("n1").expect("net n1");

    // transient with a fast (near-step) driver edge
    let tr = rc.transient("3:Y", 0.001, 0.0).expect("tree");
    let (delay, slew) = tr.get("4:A").copied().expect("sink");
    assert!(
        (delay - 0.693).abs() < 0.03,
        "RC step 50% should be ~0.693 ns, got {delay}"
    );
    assert!(slew > 0.0, "sink should have a finite slew, got {slew}");

    // Elmore (first moment) over-estimates: R·C = 1.0 ns
    let elmore = rc.elmore("3:Y", 0.0).expect("elmore");
    let e = elmore.get("4:A").copied().expect("sink elmore");
    assert!((e - 1.0).abs() < 1e-6, "Elmore should be 1.0 ns, got {e}");
    assert!(delay < e, "transient {delay} should be below Elmore {e}");
}
