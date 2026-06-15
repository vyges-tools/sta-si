//! Phase-0 gate for the persistent `Timer`: building a `Timer` and reading its report must
//! be bit-identical to the one-shot `analyze` (they share the same code path now). This
//! locks the refactor as behavior-preserving before later phases add incremental update.
#![allow(clippy::float_cmp)] // exact equality is the point — same computation, same bits

use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::sta::{analyze, Timer};

const LIB: &str = r#"
library (d) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
  cell (INV2) {
    pin (A) { direction : input; capacitance : 0.0010; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.04, 0.10", "0.06, 0.14" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.035, 0.09", "0.055, 0.13" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.015, 0.045", "0.02, 0.055" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.015, 0.04", "0.02, 0.05" ); }
      }
    }
  }
}
"#;

const NL: &str = "module top ( a, y ); input a; output y; wire n1;\n\
                  INV u1 ( .A(a), .Y(n1) ); INV u2 ( .A(n1), .Y(y) ); endmodule";

fn job(period: f64) -> StaJob {
    StaJob {
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: period,
        clocks: vec![],
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
        xtalk_window: 0.2,
        scenarios: vec![],
        exceptions: vec![],
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
        io_input_delays: vec![],
        io_output_delays: vec![],
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        sdc: None,
        base_dir: String::new(),
    }
}

/// `Timer::build(..).report()` == `analyze(..)`, field for field, on a met design.
#[test]
fn timer_report_is_identical_to_analyze() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job(1.0);

    let direct = analyze(&nl, &lib, &j, None).unwrap();
    let timer = Timer::build(&nl, &lib, &j, None).unwrap();
    let r = timer.report();

    // aggregates via the accessors and via report()
    assert_eq!(timer.wns(), direct.wns);
    assert_eq!(timer.tns(), direct.tns);
    assert_eq!(timer.whs(), direct.whs);
    assert_eq!(timer.ths(), direct.ths);
    assert_eq!(r.wns, direct.wns);
    assert_eq!(r.tns, direct.tns);
    assert_eq!(r.endpoints, direct.endpoints);
    assert_eq!(r.worst_endpoint, direct.worst_endpoint);

    // the worst path, node for node (label + arrival + slew bit-identical)
    assert_eq!(r.worst_path.len(), direct.worst_path.len());
    for (a, b) in r.worst_path.iter().zip(&direct.worst_path) {
        assert_eq!(a.label, b.label);
        assert_eq!(a.arrival, b.arrival);
        assert_eq!(a.slew, b.slew);
    }
}

/// Same equivalence on a violating (tight-period) design — wns/tns negative, still identical.
#[test]
fn timer_matches_analyze_on_violation() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job(0.1);

    let direct = analyze(&nl, &lib, &j, None).unwrap();
    let timer = Timer::build(&nl, &lib, &j, None).unwrap();

    assert!(timer.wns() < 0.0, "expected a violation");
    assert_eq!(timer.wns(), direct.wns);
    assert_eq!(timer.tns(), direct.tns);
}

/// Phase-1 query API: labels resolve, per-pin arrival matches the path, endpoint slacks
/// are consistent with the report, and non-endpoints have no slack/required.
#[test]
fn timer_query_api() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job(1.0);
    let t = Timer::build(&nl, &lib, &j, None).unwrap();
    let r = t.report();

    assert!(t.num_pins() > 0);

    // output `y` is the lone setup endpoint; label round-trips to its handle.
    let y = t.pin("y").expect("output y is a pin");
    assert_eq!(t.pin_label(y), "y");
    assert!(t.is_endpoint(y));

    // per-pin arrival at y equals the worst path's final node (same committed array).
    assert_eq!(t.arrival(y), r.worst_path.last().unwrap().arrival);

    // single endpoint -> its slack is the WNS; slack == required − arrival (definitional).
    let eps = t.endpoint_slacks();
    assert_eq!(eps.len(), 1);
    assert_eq!(eps[0].0, y);
    assert_eq!(eps[0].1, r.wns);
    assert_eq!(t.slack(y), Some(r.wns));
    assert!(t.required(y).is_some());
    assert_eq!(t.slack(y), t.required(y).map(|req| req - t.arrival(y)));

    // a primary input is reached but is not an endpoint -> no required/slack.
    let a = t.pin("a").expect("input a");
    assert!(!t.is_endpoint(a));
    assert!(t.arrival(a).is_finite());
    assert_eq!(t.required(a), None);
    assert_eq!(t.slack(a), None);
}

use vyges_sta_si::sta::Move;

/// Phase-2: a resize is staged, `update()` recomputes, timing changes, and the result equals
/// a fresh build of the mutated netlist (the shadow-check). Unknown instances are no-ops.
#[test]
fn timer_resize_updates_and_matches_rebuild() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job(1.0);
    let mut t = Timer::build(&nl, &lib, &j, None).unwrap();
    let before = t.wns();

    // swap u1 from INV to the faster INV2 (also via the explicit Move form once)
    assert!(t.stage(Move::Resize { inst: "u1".into(), cell: "INV2".into() }));
    assert!(t.is_dirty());
    t.update().unwrap();
    assert!(!t.is_dirty());
    let after = t.wns();
    assert!(after > before, "faster cell should improve slack: {before} -> {after}");
    assert_eq!(
        t.netlist().insts.iter().find(|i| i.name == "u1").unwrap().cell,
        "INV2"
    );

    // shadow-check: update() == a fresh build of the mutated netlist (it IS a rebuild).
    let fresh = Timer::build(t.netlist(), &lib, &j, None).unwrap();
    assert_eq!(t.wns(), fresh.wns());
    assert_eq!(t.tns(), fresh.tns());
    let y = t.pin("y").unwrap();
    assert_eq!(t.arrival(y), fresh.arrival(fresh.pin("y").unwrap()));

    // unknown instance -> no-op, stays clean.
    assert!(!t.resize("does_not_exist", "INV2"));
    assert!(!t.is_dirty());
}

// ---- Phase 2b: cone-localized incremental update, shadow-checked on a sequential design ----

const SEQ_LIB: &str = r#"
library (seq) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
  cell (INV2) {
    pin (A) { direction : input; capacitance : 0.0010; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.04, 0.10", "0.06, 0.14" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.035, 0.09", "0.055, 0.13" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.015, 0.045", "0.02, 0.055" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.015, 0.04", "0.02, 0.05" ); }
      }
    }
  }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input; capacitance : 0.001;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); } }
      timing () { related_pin : "CK"; timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); } }
    }
    pin (Q) {
      direction : output;
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.22", "0.14, 0.30" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); } }
    }
  }
}
"#;

// reg-to-reg with a 3-stage combinational cone between the flops (and feedback r2.Q -> r1.D),
// so there are both setup and hold endpoints and a multi-node cone to resize within.
const SEQ_NL: &str = "module seq ( clk, y ); input clk; output y; wire q1, a, b, n;\n\
                      DFF r1 ( .CK(clk), .D(y), .Q(q1) );\n\
                      INV g1 ( .A(q1), .Y(a) );\n\
                      INV g2 ( .A(a),  .Y(b) );\n\
                      INV g3 ( .A(b),  .Y(n) );\n\
                      DFF r2 ( .CK(clk), .D(n), .Q(y) );\n\
                      endmodule";

fn seq_job(period: f64) -> StaJob {
    let mut j = job(period);
    j.design = "seq".into();
    j.clock_port = "clk".into();
    j
}

/// Assert two timers agree field-for-field, bit-for-bit: aggregates, worst endpoints, the
/// worst setup/hold paths, and every pin's arrival / earliest-arrival / slew.
fn assert_timers_identical(a: &Timer, b: &Timer) {
    let ra = a.report();
    let rb = b.report();
    assert_eq!(ra.wns, rb.wns, "wns");
    assert_eq!(ra.tns, rb.tns, "tns");
    assert_eq!(ra.whs, rb.whs, "whs");
    assert_eq!(ra.ths, rb.ths, "ths");
    assert_eq!(ra.endpoints, rb.endpoints, "endpoints");
    assert_eq!(ra.hold_endpoints, rb.hold_endpoints, "hold_endpoints");
    assert_eq!(ra.worst_endpoint, rb.worst_endpoint, "worst_endpoint");
    assert_eq!(ra.worst_hold_endpoint, rb.worst_hold_endpoint, "worst_hold_endpoint");
    assert_eq!(ra.worst_path.len(), rb.worst_path.len(), "worst_path len");
    for (x, y) in ra.worst_path.iter().zip(&rb.worst_path) {
        assert_eq!(x.label, y.label);
        assert_eq!(x.arrival, y.arrival, "setup path arrival at {}", x.label);
        assert_eq!(x.slew, y.slew, "setup path slew at {}", x.label);
    }
    for (x, y) in ra.worst_hold_path.iter().zip(&rb.worst_hold_path) {
        assert_eq!(x.arrival, y.arrival, "hold path arrival at {}", x.label);
    }
    // every pin, matched by label (node ids are deterministic but compare by label anyway).
    assert_eq!(a.num_pins(), b.num_pins(), "pin count");
    for i in 0..a.num_pins() {
        let label = a.pin_label(i);
        let j = b.pin(label).expect("same labels");
        assert_eq!(a.arrival(i), b.arrival(j), "arrival at {label}");
        assert_eq!(a.arrival_min(i), b.arrival_min(j), "arrival_min at {label}");
        assert_eq!(a.slew(i), b.slew(j), "slew at {label}");
    }
}

/// The core Phase-2b gate: drive a randomized sequence of resizes through the incremental
/// `update()` and assert it stays byte-identical to a fresh full build at every step — on a
/// sequential design with setup + hold endpoints, across both violating and met periods.
#[test]
fn incremental_update_matches_full_rebuild_under_random_resizes() {
    let lib = Lib::parse(SEQ_LIB).unwrap();
    let gates = ["g1", "g2", "g3"];
    let cells = ["INV", "INV2"];

    for &period in &[0.6_f64, 0.25] {
        let nl = netlist::parse(SEQ_NL).unwrap();
        let j = seq_job(period);
        let mut t = Timer::build(&nl, &lib, &j, None).unwrap();
        // sanity: this is the simple context, so the fast path is live and the baseline matches.
        assert_timers_identical(&t, &Timer::build(t.netlist(), &lib, &j, None).unwrap());

        let mut rng: u64 = 0x9e3779b97f4a7c15 ^ period.to_bits();
        for _ in 0..40 {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let gate = gates[((rng >> 33) as usize) % gates.len()];
            let cell = cells[((rng >> 29) as usize) % cells.len()];
            t.resize(gate, cell);
            t.update().unwrap();
            let fresh = Timer::build(t.netlist(), &lib, &j, None).unwrap();
            assert_timers_identical(&t, &fresh);
        }
        // the fast path must have actually carried these updates (else the test is vacuous).
        let (inc, full) = t.update_stats();
        assert!(inc >= 35, "expected mostly incremental updates, got {inc} inc / {full} full");
    }
}

/// Speculation must roll back exactly: checkpoint, apply a tentative resize, update, then
/// restore — and the restored timer (and a subsequent real update) match a fresh build.
#[test]
fn incremental_checkpoint_restore_is_exact_then_resumes() {
    let lib = Lib::parse(SEQ_LIB).unwrap();
    let nl = netlist::parse(SEQ_NL).unwrap();
    let j = seq_job(0.3);
    let mut t = Timer::build(&nl, &lib, &j, None).unwrap();

    // commit one move so the cached state is non-trivial, then snapshot.
    t.resize("g2", "INV2");
    t.update().unwrap();
    let ckpt = t.checkpoint();
    let committed = Timer::build(t.netlist(), &lib, &j, None).unwrap();

    // speculate, update, then roll back — must equal the committed state exactly.
    t.resize("g1", "INV2");
    t.resize("g3", "INV2");
    t.update().unwrap();
    t.restore(ckpt);
    assert_timers_identical(&t, &committed);

    // the incremental path still works after a restore.
    t.resize("g3", "INV2");
    t.update().unwrap();
    assert_timers_identical(&t, &Timer::build(t.netlist(), &lib, &j, None).unwrap());
}

/// Phase-2: checkpoint → mutate → update → restore returns to the exact prior state, no recompute.
#[test]
fn timer_checkpoint_restore_round_trips() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let j = job(1.0);
    let mut t = Timer::build(&nl, &lib, &j, None).unwrap();
    let base = t.wns();
    let ckpt = t.checkpoint();

    t.resize("u1", "INV2");
    t.update().unwrap();
    assert!(t.wns() != base, "mutation should change wns");

    t.restore(ckpt);
    assert_eq!(t.wns(), base); // back to baseline (cached, no recompute)
    assert_eq!(t.netlist().insts.iter().find(|i| i.name == "u1").unwrap().cell, "INV");
    assert!(!t.is_dirty());
}
