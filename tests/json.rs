use vyges_sta_si::engine::{demo, report_json};

#[test]
fn json_report() {
    let (job, rep) = demo();
    let j = report_json(&job, &rep);
    assert!(j.contains("\"design\":\"demo\""));
    assert!(j.contains("\"met\":true"));
    assert!(j.contains("\"worst_endpoint\":\"y\""));
    assert!(j.contains("\"wns_ns\":"));
    assert!(j.trim_end().ends_with('}'));
}

/// `timing_met` is the verdict over both checks, and distinguishes "nothing was
/// analyzed" from "a check failed" — `met`/`hold_met` collapse those into `false`.
#[test]
fn timing_met_spans_setup_and_hold() {
    let (job, mut rep) = demo();
    assert!(report_json(&job, &rep).contains("\"timing_met\":true"));

    // Hold violation with setup still clean: `met` stays true, so a consumer
    // reading it alone would call this design timing-clean.
    let clean = rep.clone();
    rep.hold_endpoints = 1;
    rep.whs = -0.05;
    let j = report_json(&job, &rep);
    assert!(j.contains("\"met\":true"), "setup unaffected: {j}");
    assert!(
        j.contains("\"timing_met\":false"),
        "hold violation must fail the verdict: {j}"
    );

    // Nothing analyzed at all → no evidence, so no verdict (not a failure).
    rep = clean;
    rep.endpoints = 0;
    rep.hold_endpoints = 0;
    let j = report_json(&job, &rep);
    assert!(
        j.contains("\"met\":false"),
        "met collapses no-endpoints to false: {j}"
    );
    assert!(
        j.contains("\"timing_met\":null"),
        "no evidence is not a failure: {j}"
    );
}
