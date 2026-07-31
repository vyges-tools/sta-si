// SPDX-License-Identifier: Apache-2.0
//! What a repair plan **costs** — G6.
//!
//! Planning is dominated by re-timing: every candidate is judged rather than assumed, so a plan
//! with N fixes costs rather more than N analyses. The timer already has a cone-localized
//! incremental path; these tests make its use a *measured, regression-guarded* property of
//! planning rather than an assumption.
//!
//! Two misses are expected and principled, and are asserted as such rather than papered over:
//!
//! - **Inserting a cell changes topology**, so the incremental graph is rebuilt.
//! - **Resizing a sequential cell touches the clock network**, which the cone recompute
//!   explicitly declines — it validates itself against the netlist and degrades to a full
//!   analysis rather than risking a wrong answer. Extending it to clock arcs is real work on a
//!   delicate engine and wants a real design to justify it, not a three-instance fixture.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::repair::{
    plan_hold_repair, plan_repair, plan_setup_repair, CombinedOpts, RepairOpts, SetupRepairOpts,
};
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/eco_demo/");

fn timer(period: f64, hold_uncertainty: f64) -> Timer {
    let mut job = StaJob::load(&format!("{D}eco_demo.sta")).unwrap();
    job.period_ns = period;
    job.hold_uncertainty = hold_uncertainty;
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

fn hold_opts() -> RepairOpts {
    RepairOpts { delay_cell: "BUF".into(), ..Default::default() }
}

#[test]
fn a_plan_reports_its_own_cost_not_the_timers_lifetime_total() {
    // The counters are deltas. A timer that has already done work must not have that work
    // charged to the next plan.
    let mut t = timer(1.0, 0.25);
    t.resize("g1", "INV_X2");
    t.update().unwrap(); // one re-timing before planning starts
    let (inc0, full0) = t.update_stats();
    assert!(inc0 + full0 > 0, "the timer should have some history to be charged for");

    let plan = plan_hold_repair(&mut t, &hold_opts()).unwrap();
    let (inc1, full1) = t.update_stats();

    assert_eq!(plan.updates_incremental, inc1 - inc0, "incremental cost is a delta");
    assert_eq!(plan.updates_full, full1 - full0, "full cost is a delta");
    assert_eq!(plan.updates(), plan.updates_incremental + plan.updates_full);
}

#[test]
fn planning_costs_more_re_timings_than_it_makes_fixes() {
    // Not a defect — it is what judging rather than assuming buys. Worth stating so the cost
    // model is explicit: rejected candidates are re-timed too.
    let mut t = timer(1.0, 0.25);
    let plan = plan_hold_repair(&mut t, &hold_opts()).unwrap();
    assert!(!plan.is_empty());
    assert!(
        plan.updates() >= plan.fixes.len() as u64,
        "every accepted fix costs at least one re-timing"
    );
}

#[test]
fn hold_repair_is_served_mostly_by_the_incremental_path() {
    // Regression guard. Hold repair tries cell swaps (incremental) before insertions (full), so
    // most of its re-timings should be cone-localized. If a change silently disables the fast
    // path — which is exactly the bug I assumed existed before measuring — this catches it.
    let mut t = timer(1.0, 0.25);
    let plan = plan_hold_repair(&mut t, &hold_opts()).unwrap();
    assert!(plan.updates() > 0);
    assert!(
        plan.incremental_rate() > 0.5,
        "expected mostly incremental, got {}/{} ({:.0}%)",
        plan.updates_incremental,
        plan.updates(),
        plan.incremental_rate() * 100.0
    );
}

#[test]
fn a_topology_change_costs_a_full_re_analysis() {
    // Insertion adds an instance, so the cone graph cannot be reused. Asserted rather than
    // assumed, because "why is this slow" is much easier to answer when the expected misses are
    // written down.
    let mut t = timer(1.0, 0.25);
    let plan = plan_hold_repair(
        &mut t,
        &RepairOpts { prefer_swap: false, ..hold_opts() }, // force insertion-only
    )
    .unwrap();
    assert!(!plan.is_empty());
    assert!(
        plan.updates_full >= plan.fixes.len() as u64,
        "each insertion should cost a full re-analysis"
    );
}

#[test]
fn resizing_a_sequential_cell_falls_back_to_a_full_analysis() {
    // The measured gap. Setup repair on this design upsizes a FLOP, and the incremental path
    // declines sequential cells rather than guessing at clock arcs. Documented as a known miss:
    // closing it is real work on the timing engine and wants a real design to justify it.
    let mut t = timer(0.16, 0.0);
    let plan = plan_setup_repair(&mut t, &SetupRepairOpts::default()).unwrap();
    assert!(!plan.is_empty(), "this should produce a fix");
    assert_eq!(
        plan.updates_incremental, 0,
        "a flop resize is not localizable, so nothing should be served incrementally"
    );
    assert!(plan.updates_full > 0);
}

#[test]
fn nothing_to_do_costs_nothing() {
    let mut t = timer(1.0, 0.0);
    let plan = plan_repair(&mut t, &CombinedOpts { hold: hold_opts(), ..Default::default() })
        .unwrap();
    assert!(plan.is_empty());
    assert_eq!(plan.updates(), 0, "a met design should not be re-timed at all");
    assert_eq!(plan.incremental_rate(), 1.0, "and that is not a slow answer");
}

#[test]
fn the_cost_is_carried_into_the_emitted_plan() {
    // A flow reading the plan should be able to see what producing it cost, without re-running.
    let mut t = timer(1.0, 0.25);
    let plan = plan_hold_repair(&mut t, &hold_opts()).unwrap();
    let json = plan.to_json("eco_demo");
    assert!(json.contains("\"cost\""), "the plan should carry its cost: {json}");
    assert!(json.contains(&format!("\"retimings\":{}", plan.updates())));
    assert!(json.contains(&format!("\"incremental\":{}", plan.updates_incremental)));
}
