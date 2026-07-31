// SPDX-License-Identifier: Apache-2.0
//! Addressability: turning timing results into things a downstream tool can act on.
//!
//! `PinId` is a graph index and `pin_label` is display text — neither can be looked up in a
//! netlist or a physical database. These tests pin the bridge: `pin_site` resolves a pin to
//! instance / library cell / pin / net, `violations` hands back failing endpoints already
//! resolved, and `worst_path_stages` adds per-stage delay so a consumer can tell *which arc*
//! on a long path is worth attacking.
//!
//! This is G1 of the timing-driven ECO loop: without it the timer and the ODB write path
//! cannot be joined at all.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::{Check, Timer};
use vyges_sta_si::{liberty::Lib, netlist};

const TOP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/");
const SEQ: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/");

fn timer(dir: &str, job_file: &str) -> Timer {
    let job = StaJob::load(&format!("{dir}{job_file}")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load(&job.resolve(l)).unwrap();
        lib.cells.extend(one.cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

#[test]
fn an_instance_pin_resolves_to_instance_cell_pin_and_net() {
    let t = timer(TOP, "top.sta");
    // find any resolved instance pin and check every field is real
    let site = (0..t.num_pins())
        .map(|p| t.pin_site(p))
        .find(|s| s.is_instance_pin())
        .expect("expected at least one instance pin");

    let inst = site.inst.as_deref().unwrap();
    let pin = site.pin.as_deref().unwrap();
    assert_eq!(site.label, format!("{inst}/{pin}"), "label should be inst/pin");
    assert!(!site.is_port);

    // the identity must agree with the netlist, not just look plausible
    let nl = t.netlist();
    let i = nl.insts.iter().find(|i| i.name == inst).expect("instance must exist");
    assert_eq!(site.master.as_deref(), Some(i.cell.as_str()), "master is the instance's cell");
    let net = i.conns.iter().find(|(p, _)| p == pin).map(|(_, n)| n.as_str());
    assert_eq!(site.net.as_deref(), net, "net must match the netlist connection");
}

#[test]
fn a_primary_port_resolves_as_a_port_not_a_bogus_instance() {
    let t = timer(TOP, "top.sta");
    let nl = t.netlist();
    let port = nl.inputs.first().or_else(|| nl.outputs.first()).unwrap().clone();
    let p = t.pin(&port).expect("the port should be a timing pin");
    let site = t.pin_site(p);

    assert!(site.is_port);
    assert!(!site.is_instance_pin());
    assert_eq!(site.inst, None);
    assert_eq!(site.master, None);
    // a primary port drives a net of the same name — that is still addressable
    assert_eq!(site.net.as_deref(), Some(port.as_str()));
}

#[test]
fn every_pin_resolves_to_something_addressable() {
    // No pin may resolve to "nothing": a consumer iterating the graph must always get either an
    // instance pin or a port, never a site with no identity at all.
    for (dir, job) in [(TOP, "top.sta"), (SEQ, "seq.sta")] {
        let t = timer(dir, job);
        for p in 0..t.num_pins() {
            let s = t.pin_site(p);
            assert_eq!(s.pin_id, p, "site must carry the id it was asked about");
            assert!(!s.label.is_empty());
            assert!(
                s.is_instance_pin() || s.is_port,
                "{} resolved to neither an instance pin nor a port",
                s.label
            );
            assert!(s.net.is_some(), "{} has no net to address", s.label);
        }
    }
}

#[test]
fn a_hierarchical_instance_name_is_not_split_at_the_wrong_slash() {
    // Instance names contain '/' in a hierarchical design, so splitting the label at the FIRST
    // separator would silently produce a wrong instance and a wrong net. Resolution splits at
    // the LAST one and confirms the result against the netlist.
    let mut nl = netlist::load(&format!("{TOP}top.v")).unwrap();
    let original = nl.insts[0].name.clone();
    let hier = format!("u_top/u_mid/{original}");
    nl.insts[0].name = hier.clone();

    let job = StaJob::load(&format!("{TOP}top.sta")).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let t = Timer::build(&nl, &lib, &job, None).unwrap();

    let pin_name = nl.insts[0].conns[0].0.clone();
    let p = t.pin(&format!("{hier}/{pin_name}")).expect("hierarchical pin should exist");
    let site = t.pin_site(p);
    assert_eq!(site.inst.as_deref(), Some(hier.as_str()), "the full hierarchical path is the instance");
    assert_eq!(site.pin.as_deref(), Some(pin_name.as_str()));
    assert!(site.net.is_some());
}

#[test]
fn violations_are_resolved_ranked_and_failing_only() {
    // The design is expected to meet timing, so an unconstrained query returns nothing — that
    // is the contract: `violations` answers "what is broken", not "rank everything".
    let t = timer(TOP, "top.sta");
    assert!(t.wns() > 0.0, "fixture should meet setup timing");
    assert!(t.violations(Check::Setup, 0).is_empty(), "no failing endpoints when WNS > 0");

    // Squeeze the clock until it cannot be met, then the same query must produce actionable work.
    let mut job = StaJob::load(&format!("{TOP}top.sta")).unwrap();
    job.period_ns = 0.05; // 50 ps period — unmeetable (period_ns is what the engine reads)
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let t = Timer::build(&nl, &lib, &job, None).unwrap();

    let v = t.violations(Check::Setup, 0);
    assert!(!v.is_empty(), "a 50 ps clock must fail");
    for w in &v {
        assert!(w.slack < 0.0, "only failing endpoints are returned");
        assert_eq!(w.check, Check::Setup);
        assert!(w.site.net.is_some(), "a violation must be addressable");
    }
    // worst first
    for pair in v.windows(2) {
        assert!(pair[0].slack <= pair[1].slack, "violations must be ranked worst first");
    }
    // and the worst one is the report's worst endpoint
    assert_eq!(v[0].site.label, t.report().worst_endpoint);
    // limit takes the N worst, not an arbitrary N
    let one = t.violations(Check::Setup, 1);
    assert_eq!(one.len().min(1), 1);
    assert_eq!(one[0].site.label, v[0].site.label);
}

#[test]
fn path_stages_carry_identity_and_per_stage_delay() {
    let t = timer(TOP, "top.sta");
    let stages = t.worst_path_stages();
    assert!(!stages.is_empty(), "there should be a critical path");
    assert_eq!(stages.len(), t.worst_path().len(), "one stage per path node");

    assert_eq!(stages[0].stage_delay, 0.0, "the launch point has no incoming stage");
    for (i, s) in stages.iter().enumerate() {
        assert_eq!(s.arrival, t.worst_path()[i].arrival);
        assert!(s.site.is_instance_pin() || s.site.is_port, "every stage must be addressable");
        if i > 0 {
            // arrival is monotonic along the path, so stage delay is the arrival delta
            assert!((s.stage_delay - (s.arrival - stages[i - 1].arrival)).abs() < 1e-12);
        }
    }
    // the stage delays must account for the whole path
    let total: f64 = stages.iter().map(|s| s.stage_delay).sum();
    let span = stages.last().unwrap().arrival - stages[0].arrival;
    assert!((total - span).abs() < 1e-9, "stage delays should sum to the path's arrival span");
}

#[test]
fn a_resolved_site_is_what_a_resize_move_needs() {
    // The point of the whole exercise: a violation must yield the exact strings a mutation
    // takes. Feed the worst path's instance straight into `resize` and it applies.
    let mut t = timer(TOP, "top.sta");
    let stage = t
        .worst_path_stages()
        .into_iter()
        .find(|s| s.site.is_instance_pin())
        .expect("the critical path should touch an instance");
    let inst = stage.site.inst.clone().unwrap();
    let master = stage.site.master.clone().unwrap();

    // resizing to its own cell is a no-op electrically but proves the identity resolves
    assert!(t.resize(&inst, &master), "the site's instance name must be one `resize` accepts");
}
