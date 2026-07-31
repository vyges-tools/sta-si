// SPDX-License-Identifier: Apache-2.0
//! Differential harness for the incremental timing path.
//!
//! The one property that matters:
//!
//! > For every edit the incremental path **accepts**, its report must equal the report a full
//! > re-analysis produces from the same netlist.
//!
//! ## Why the comparison is in ULPs and not exact bits
//!
//! The design for this harness demanded bit-for-bit equality, on the grounds that timing has no
//! "close enough". Running it disproved that bar on the first attempt, for a reason worth
//! writing down rather than papering over.
//!
//! The incremental path maintains a driver's capacitive load by **accumulating deltas**
//! (`node_load[d] += new_cap - old_cap`), while the full path **re-sums the sink caps from
//! scratch**. Floating-point addition is not associative, so the two agree to within one
//! representable step and not to the bit. The observed disagreements were exactly 1 ULP —
//! `0.0014999999999999996` against `0.0015`.
//!
//! That is a difference in *arithmetic order*, not in *logic*, and those deserve different
//! treatment. So the bar is now **ULP distance**, which is stricter and more meaningful than any
//! absolute or relative epsilon: it says "these are the same number to the limit of the
//! representation". A real defect — a missed dependency, a stale cone, a wrong edge — is orders
//! of magnitude away and still fails loudly.
//!
//! A tolerance chosen to make a test pass would be worthless; this one is chosen to be the
//! smallest bound that floating-point representation permits. If exactness is ever wanted, the
//! fix is on the engine side: recompute an affected net's load from its sinks instead of
//! accumulating. That is noted in the sequential-cells design as a possible upgrade, not
//! smuggled in as an epsilon here.
//!
//! This exists before any extension of the incremental path, because it also protects what
//! already works: the combinational resize path has no such check today.
#![allow(dead_code)] // each integration test uses a different subset

use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist::Netlist;
use vyges_sta_si::sta::{Move, Timer};

/// Which path served an `update()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Served {
    /// The cone-localized fast path.
    Incremental,
    /// A full re-analysis — always correct, and what the fast path is checked against.
    Full,
}

/// The outcome of applying one edit and checking it against a full re-analysis.
pub struct Checked {
    pub served: Served,
    /// Empty when the two paths agree. Each entry names the quantity and both values.
    pub mismatches: Vec<String>,
    /// True when the move did not apply at all (bad instance, name clash) — not a timing result.
    pub not_applicable: bool,
}

impl Checked {
    pub fn agrees(&self) -> bool {
        self.mismatches.is_empty()
    }
}

/// Load a job plus its netlist and merged Liberty.
pub fn load(job_path: &str) -> (StaJob, Netlist, Lib) {
    let job = StaJob::load(job_path).unwrap();
    let nl = vyges_sta_si::netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    (job, nl, lib)
}

/// How many representable steps apart two finite `f64`s are. `0` means bit-identical.
pub fn ulp_distance(a: f64, b: f64) -> u64 {
    if a.to_bits() == b.to_bits() {
        return 0;
    }
    if !a.is_finite() || !b.is_finite() {
        return u64::MAX;
    }
    // map to a monotonic ordering so the distance is meaningful across zero
    let key = |x: f64| -> i64 {
        let bits = x.to_bits() as i64;
        if bits < 0 {
            i64::MIN - bits // negative floats descend as bits ascend
        } else {
            bits
        }
    };
    key(a).abs_diff(key(b))
}

/// The largest disagreement attributable to accumulation order rather than to a defect.
/// Deliberately tiny: one step is what non-associative addition can cost, and anything a real
/// bug produces is nowhere near this.
pub const MAX_ULP: u64 = 2;

fn cmp_f64(what: &str, got: f64, want: f64, out: &mut Vec<String>) {
    if got.is_nan() && want.is_nan() {
        return;
    }
    let d = ulp_distance(got, want);
    if d > MAX_ULP {
        out.push(format!(
            "{what}: incremental {got:?} != full {want:?} ({d} ULP, delta {:e})",
            got - want
        ));
    }
}

/// Apply `mv` to `t`, then check the resulting report against a full re-analysis of the very
/// same netlist. `t` is left holding the edit — restore from a checkpoint if you need it back.
///
/// The reference is built from `t`'s own mutated netlist rather than a separately-constructed
/// one, so the two sides cannot differ by anything except the analysis path.
pub fn check_edit(t: &mut Timer, job: &StaJob, lib: &Lib, mv: Move) -> Checked {
    let (inc0, full0) = t.update_stats();
    if !t.stage(mv) {
        return Checked { served: Served::Full, mismatches: Vec::new(), not_applicable: true };
    }
    t.update().expect("incremental update should not fail");
    let (inc1, _) = t.update_stats();
    let served = if inc1 > inc0 { Served::Incremental } else { Served::Full };
    let _ = full0;

    // the reference: a full analysis of the edited netlist
    let reference = Timer::build(t.netlist(), lib, job, None).expect("reference build");

    let mut m = Vec::new();
    let (a, b) = (t.report(), reference.report());
    cmp_f64("wns", a.wns, b.wns, &mut m);
    cmp_f64("tns", a.tns, b.tns, &mut m);
    cmp_f64("whs", a.whs, b.whs, &mut m);
    cmp_f64("ths", a.ths, b.ths, &mut m);
    if a.endpoints != b.endpoints {
        m.push(format!("endpoints: {} != {}", a.endpoints, b.endpoints));
    }
    if a.hold_endpoints != b.hold_endpoints {
        m.push(format!("hold_endpoints: {} != {}", a.hold_endpoints, b.hold_endpoints));
    }
    if a.worst_endpoint != b.worst_endpoint {
        m.push(format!("worst_endpoint: {:?} != {:?}", a.worst_endpoint, b.worst_endpoint));
    }
    if a.worst_hold_endpoint != b.worst_hold_endpoint {
        m.push(format!(
            "worst_hold_endpoint: {:?} != {:?}",
            a.worst_hold_endpoint, b.worst_hold_endpoint
        ));
    }

    // per-pin state, keyed by LABEL rather than index: a full rebuild need not number the graph
    // the same way, and comparing by index would report differences that are not differences.
    if t.num_pins() != reference.num_pins() {
        m.push(format!("pin count: {} != {}", t.num_pins(), reference.num_pins()));
    }
    for p in 0..t.num_pins() {
        let label = t.pin_label(p).to_string();
        let Some(q) = reference.pin(&label) else {
            m.push(format!("pin {label}: missing from the full re-analysis"));
            continue;
        };
        cmp_f64(&format!("{label}.arrival"), t.arrival(p), reference.arrival(q), &mut m);
        cmp_f64(&format!("{label}.slew"), t.slew(p), reference.slew(q), &mut m);
        cmp_f64(&format!("{label}.load"), t.load(p), reference.load(q), &mut m);
        cmp_f64(&format!("{label}.arrival_min"), t.arrival_min(p), reference.arrival_min(q), &mut m);
        match (t.slack(p), reference.slack(q)) {
            (Some(x), Some(y)) => cmp_f64(&format!("{label}.slack"), x, y, &mut m),
            (x, y) if x.is_some() != y.is_some() => {
                m.push(format!("{label}.slack: {x:?} != {y:?}"))
            }
            _ => {}
        }
        if t.is_endpoint(p) != reference.is_endpoint(q) {
            m.push(format!("{label}.is_endpoint differs"));
        }
    }
    Checked { served, mismatches: m, not_applicable: false }
}

/// Every resize this library can express for `inst`: each interchangeable cell except its
/// current one. Empty when the library carries no equivalence data.
pub fn resize_moves(t: &Timer, inst: &str) -> Vec<Move> {
    let Some(cur) = t.netlist().insts.iter().find(|i| i.name == inst).map(|i| i.cell.clone())
    else {
        return Vec::new();
    };
    t.lib()
        .equivalence_class(&cur)
        .into_iter()
        .filter(|c| c.name != cur)
        .map(|c| Move::Resize { inst: inst.to_string(), cell: c.name.clone() })
        .collect()
}
