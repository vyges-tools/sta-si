//! End-to-end: the example design runs offline (v0 is pure-std, no subprocess).

use vyges_sta_si::engine::{analyze_job, analyze_job_opts, render_report, MarginAdvisory};
use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::LibOpts;
use vyges_sta_si::sta::TimingReport;

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

// ---- timing-health advisory (#10) ---------------------------------------

/// Build a synthetic report: `hold_critical` of `hold_endpoints` sit at 0 ns (≤ margin),
/// the rest at a comfortable +1 ns. Setup is a single worst path with slack `wns`.
fn synthetic_report(wns: f64, hold_endpoints: usize, hold_critical: usize) -> TimingReport {
    let hold_slacks = (0..hold_endpoints)
        .map(|i| (i, if i < hold_critical { 0.0 } else { 1.0 }))
        .collect();
    TimingReport {
        wns,
        tns: 0.0,
        endpoints: 100,
        worst_endpoint: "q".into(),
        worst_path: vec![],
        whs: if hold_critical > 0 { 0.0 } else { 1.0 },
        ths: 0.0,
        hold_endpoints,
        worst_hold_endpoint: "q".into(),
        worst_hold_path: vec![],
        hold_slacks,
        pba_wns: None,
    }
}

#[test]
fn advisory_reports_achievable_frequency() {
    // 40 ns clock, +26 ns setup margin → closes at ~14 ns → ~71.4 MHz.
    let rep = synthetic_report(26.0, 0, 0);
    let adv = MarginAdvisory::compute(40.0, &rep).expect("advisory");
    assert!((adv.achievable_ns - 14.0).abs() < 1e-9);
    assert!((adv.max_freq_mhz.unwrap() - 1000.0 / 14.0).abs() < 1e-6);
    assert!((adv.target_freq_mhz - 25.0).abs() < 1e-9);
    assert!((adv.over_margin_ratio.unwrap() - 40.0 / 14.0).abs() < 1e-9);
}

#[test]
fn advisory_warns_on_over_margin_hold_flood() {
    // Over-margined (2.86× faster) AND 50/100 hold endpoints hold-critical → warn.
    let rep = synthetic_report(26.0, 100, 50);
    let adv = MarginAdvisory::compute(40.0, &rep).expect("advisory");
    assert!(adv.warn);
    assert_eq!(adv.hold_critical, 50);

    // The human report surfaces the warning + the achievable-frequency hint.
    let job = StaJob::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta")).unwrap();
    let mut job = job;
    job.period_ns = 40.0;
    let text = render_report(&job, &rep);
    assert!(text.contains("WARNING: over-margin"));
    assert!(text.contains("MHz"));
}

#[test]
fn advisory_no_warn_when_well_clocked_or_no_flood() {
    // Well-clocked (only 1.05× margin) even with a hold flood → no warning.
    let rep = synthetic_report(2.0, 100, 50);
    assert!(!MarginAdvisory::compute(40.0, &rep).unwrap().warn);

    // Over-margined but only 3 hold-critical (< floor of 8) → no warning.
    let rep = synthetic_report(26.0, 100, 3);
    assert!(!MarginAdvisory::compute(40.0, &rep).unwrap().warn);
}

#[test]
fn advisory_handles_near_combinational_and_no_endpoints() {
    // achievable ≤ 0 (WNS ≥ period): no finite frequency, but still flags over-margin.
    let rep = synthetic_report(40.0, 100, 50);
    let adv = MarginAdvisory::compute(40.0, &rep).unwrap();
    assert!(adv.max_freq_mhz.is_none());
    assert!(adv.over_margin_ratio.is_none());
    assert!(adv.warn);

    // No setup endpoints → no advisory at all.
    let mut empty = synthetic_report(0.0, 0, 0);
    empty.endpoints = 0;
    assert!(MarginAdvisory::compute(40.0, &empty).is_none());
}

// ---- CCS pruning / --liberty-nldm-only (#33) ----------------------------

#[test]
fn nldm_only_is_bit_identical_on_a_nldm_lib() {
    // The example lib carries no CCS, so skipping CCS at load must be a no-op —
    // this is the acceptance regression: bit-identical STA for NLDM runs.
    let p = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
    let job = StaJob::load(p).unwrap();
    let full = analyze_job(&job).unwrap();
    let nldm = analyze_job_opts(&job, LibOpts { skip_ccs: true }).unwrap();

    assert_eq!(full.wns.to_bits(), nldm.wns.to_bits());
    assert_eq!(full.tns.to_bits(), nldm.tns.to_bits());
    assert_eq!(full.endpoints, nldm.endpoints);
    assert_eq!(full.worst_endpoint, nldm.worst_endpoint);
}
