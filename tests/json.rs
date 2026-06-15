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
