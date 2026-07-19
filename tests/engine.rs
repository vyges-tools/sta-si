//! End-to-end: the example design runs offline (v0 is pure-std, no subprocess).

use vyges_sta_si::engine::{
    analyze_job, analyze_job_opts, liberty_json_for_job, render_report, MarginAdvisory,
    SlackDistribution,
};
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

// ---- shared Liberty IR JSON dump / --emit-liberty-json (#34) -------------

#[test]
fn liberty_json_dumps_the_shared_ir() {
    let p = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
    let job = StaJob::load(p).unwrap();
    let js = liberty_json_for_job(&job, LibOpts::default()).unwrap();
    assert!(js.starts_with('{') && js.trim_end().ends_with('}'));
    assert!(js.contains("\"cell_count\":"));
    assert!(js.contains("\"cells\":{"));
    assert!(js.contains("\"direction\":")); // at least one pin serialized
}

// ---- what a machine consumer reads (#10) ----
//
// The advisory existed in the text report and the event stream but not in the JSON. The
// soc-generator closure-lesson loop tunes per-PDK knob defaults from these numbers, and it
// can neither scrape a text report nor reconstruct them from an event stream.

fn health_json(period: f64, rep: &vyges_sta_si::sta::TimingReport) -> String {
    let mut job =
        StaJob::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta")).unwrap();
    job.period_ns = period;
    vyges_sta_si::engine::report_json(&job, rep)
}

#[test]
fn json_carries_the_timing_health_advisory() {
    let rep = synthetic_report(26.0, 100, 50);
    let j = health_json(40.0, &rep);
    // the decision-relevant numbers, not just a boolean
    assert!(j.contains("\"timing_health\""), "{j}");
    assert!(j.contains("\"achievable_ns\":14.000000"), "{j}");
    assert!(j.contains("\"over_margin_warn\":true"), "{j}");
    assert!(j.contains("\"hold_critical\":50"), "{j}");
    // ~71.4 MHz achievable against a 25 MHz target
    assert!(j.contains("\"target_freq_mhz\":25.000000"), "{j}");
    assert!(j.contains("\"max_freq_mhz\":71.4"), "{j}");
}

/// A quantity that is not meaningful must be `null`, never a plausible number: a consumer
/// has to be able to tell "not applicable" from "zero".
#[test]
fn json_reports_null_rather_than_inventing_a_frequency() {
    // no setup endpoints at all → no advisory to give
    let mut empty = synthetic_report(0.0, 0, 0);
    empty.endpoints = 0;
    let j = health_json(40.0, &empty);
    assert!(j.contains("\"timing_health\":null"), "{j}");

    // critical path ≈ 0 at this clock → a finite max frequency says nothing
    let comb = synthetic_report(40.0, 0, 0);
    let j = health_json(40.0, &comb);
    assert!(j.contains("\"max_freq_mhz\":null"), "{j}");
    assert!(j.contains("\"over_margin_ratio\":null"), "{j}");
}

/// A hold flood and a handful of bad paths share the same WHS. The distribution is what
/// separates them, and therefore what predicts the hold-fix burden.
#[test]
fn the_hold_distribution_separates_a_flood_from_a_few_bad_paths() {
    let flood = synthetic_report(26.0, 100, 90); // 90 of 100 at the cliff
    let few = synthetic_report(26.0, 100, 3); //  3 of 100
    let d_flood = SlackDistribution::of(&flood.hold_slacks).expect("flood");
    let d_few = SlackDistribution::of(&few.hold_slacks).expect("few");

    assert_eq!(d_flood.count, 100);
    assert_eq!(d_flood.critical, 90);
    assert_eq!(d_few.critical, 3);
    // identical worst slack...
    assert_eq!(d_flood.min_ns, d_few.min_ns);
    // ...but the median tells them apart: the flood's middle endpoint is at the cliff.
    assert_eq!(d_flood.median_ns, 0.0);
    assert_eq!(d_few.median_ns, 1.0);
}

#[test]
fn an_empty_population_has_no_distribution_rather_than_a_zero_one() {
    // Zeros would read as "every endpoint is exactly on the line", which is a claim.
    assert!(SlackDistribution::of(&[]).is_none());
    let j = health_json(40.0, &synthetic_report(26.0, 0, 0));
    assert!(j.contains("\"hold_slack_distribution\":null"), "{j}");
}

#[test]
fn percentiles_are_real_endpoint_values_not_interpolations() {
    // Slacks 0,1,2,3,4 — a median of 2 is an endpoint that exists; 2.5 would not be.
    let s: Vec<(usize, f64)> = (0..5).map(|i| (i, i as f64)).collect();
    let d = SlackDistribution::of(&s).expect("dist");
    assert_eq!((d.min_ns, d.median_ns, d.max_ns), (0.0, 2.0, 4.0));
    assert!(s.iter().any(|&(_, v)| v == d.p10_ns));
    assert!(s.iter().any(|&(_, v)| v == d.p90_ns));
}

/// The payload is assembled by hand, so its realistic failure modes are a trailing comma
/// before a closing brace and unbalanced braces — both of which make it unparseable for the
/// consumer this block exists to serve. Checked without pulling in a JSON parser, since this
/// crate is deliberately std-only.
#[test]
fn the_json_stays_well_formed_with_the_new_blocks() {
    for rep in [
        synthetic_report(26.0, 100, 50), // advisory + distribution present
        synthetic_report(-5.0, 10, 0),   // setup violated, no hold flood
        synthetic_report(26.0, 0, 0),    // both blocks null
    ] {
        let j = health_json(40.0, &rep);
        let compact: String = j.chars().filter(|c| !c.is_whitespace()).collect();
        assert!(
            !compact.contains(",}") && !compact.contains(",]"),
            "trailing comma makes the payload unparseable:\n{j}"
        );
        let mut depth = 0i32;
        let mut in_str = false;
        let mut prev_escape = false;
        for c in j.chars() {
            match c {
                '"' if !prev_escape => in_str = !in_str,
                '{' | '[' if !in_str => depth += 1,
                '}' | ']' if !in_str => depth -= 1,
                _ => {}
            }
            prev_escape = c == '\\' && !prev_escape;
            assert!(depth >= 0, "unbalanced close in:\n{j}");
        }
        assert_eq!(depth, 0, "unbalanced braces in:\n{j}");
        assert!(!in_str, "unterminated string in:\n{j}");
    }
}
