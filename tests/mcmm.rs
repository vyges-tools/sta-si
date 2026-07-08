// MCMM (multi-corner / multi-mode): worst setup and worst hold are taken across
// scenarios. Aggregation is unit-tested; the committed icsprout55 multi-corner
// job is run end-to-end.
use vyges_sta_si::engine::{analyze_mcmm, McmmReport, ScenarioResult};
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::TimingReport;

fn rep(wns: f64, whs: f64) -> TimingReport {
    TimingReport {
        wns,
        tns: 0.0,
        endpoints: 1,
        worst_endpoint: "d".into(),
        worst_path: vec![],
        whs,
        ths: 0.0,
        hold_slacks: vec![],
        hold_endpoints: 1,
        worst_hold_endpoint: "d".into(),
        worst_hold_path: vec![],
        pba_wns: None,
    }
}

#[test]
fn aggregates_worst_per_check_across_corners() {
    let m = McmmReport {
        scenarios: vec![
            ScenarioResult { name: "ss".into(), period_ns: 2.0, report: rep(0.10, 0.30) },
            ScenarioResult { name: "tt".into(), period_ns: 2.0, report: rep(0.40, 0.20) },
            ScenarioResult { name: "ff".into(), period_ns: 2.0, report: rep(0.80, -0.05) },
        ],
    };
    // setup is worst at the slow corner, hold at the fast corner — different winners
    assert_eq!(m.worst_setup(), Some(("ss", 0.10)));
    assert_eq!(m.worst_hold(), Some(("ff", -0.05)));
}

#[test]
fn runs_committed_icsprout55_mcmm_job() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/icsprout55/mcmm.sta");
    let job = StaJob::load(path).expect("load mcmm.sta");
    assert!(job.is_mcmm(), "should be detected as MCMM");
    let m = analyze_mcmm(&job).expect("analyze");
    assert_eq!(m.scenarios.len(), 3, "ss/tt/ff");
    // setup binds at the SLOW corner (slowest paths). Rows are labelled by the
    // scenario file stem (the corner identity), not the shared design name.
    let (sname, swns) = m.worst_setup().expect("setup");
    assert_eq!(sname, "corner_ss", "worst setup corner");
    assert!(swns > 0.0, "setup meets at the slow corner: {swns}");
    // hold binds at the FAST corner (fastest data) — the textbook MCMM split, now
    // that constraints are slew-interpolated rather than table-max
    let (hname, whs) = m.worst_hold().expect("hold");
    assert_eq!(hname, "corner_ff", "worst hold corner");
    assert!(whs > 0.0, "hold meets at the fast corner: {whs}");
}
