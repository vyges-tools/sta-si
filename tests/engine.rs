//! End-to-end: the example design runs offline (v0 is pure-std, no subprocess).

use vyges_sta_si::engine::analyze_job;
use vyges_sta_si::job::StaJob;

#[test]
fn example_top_analyzes() {
    let job_path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
    let job = StaJob::load(job_path).unwrap();
    let rep = analyze_job(&job).unwrap();

    assert_eq!(rep.endpoints, 1);
    assert_eq!(rep.worst_endpoint, "y");
    // three inverters: a, g1/A, g1/Y, g2/A, g2/Y, g3/A, g3/Y, y
    assert_eq!(rep.worst_path.len(), 8);
    assert_eq!(rep.worst_path[0].label, "a");
    assert_eq!(rep.worst_path.last().unwrap().label, "y");
    assert!(rep.wns > 0.0 && rep.wns < 1.0);
}
