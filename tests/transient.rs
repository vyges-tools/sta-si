// Waveform-into-RC: a single RC (R=10kΩ, C=100fF -> τ=1.0 ns) driven by a fast
// edge has a 50% delay of 0.69·RC = 0.693 ns — the true step response — versus
// Elmore's first-moment R·C = 1.0 ns. The transient solver must hit 0.69, and be
// below Elmore.
//
// Nodes are named `u1:Y`, not `3:Y`: the SPEF reader resolves a node reference through the
// name map, so a node carries the INSTANCE's name and not the index the file happened to give
// it. An index means nothing to any consumer — and to the writer it was indistinguishable from
// a net someone had named `3:Y`, which is how it came to emit networks that no longer joined
// their own pins.
use vyges_sta_si::liberty::Thresholds;
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
    let tr = // a near-step edge: 0.001 ns of Liberty slew, expanded to a full 0->100% ramp by
    // the library's own thresholds (20/80 by default -> /0.6)
    rc.transient("u1:Y", 0.001, 0.0, Thresholds::default()).expect("tree");
    let (delay, slew) = tr.get("u2:A").copied().expect("sink");
    assert!(
        (delay - 0.693).abs() < 0.03,
        "RC step 50% should be ~0.693 ns, got {delay}"
    );
    assert!(slew > 0.0, "sink should have a finite slew, got {slew}");

    // Elmore (first moment) over-estimates: R·C = 1.0 ns
    let elmore = rc.elmore("u1:Y", 0.0).expect("elmore");
    let e = elmore.get("u2:A").copied().expect("sink elmore");
    assert!((e - 1.0).abs() < 1e-6, "Elmore should be 1.0 ns, got {e}");
    assert!(delay < e, "transient {delay} should be below Elmore {e}");
}
