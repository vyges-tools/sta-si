//! STA engine: build the timing graph, propagate, report slack.
//!
//! From a netlist + Liberty + clock it builds a directed timing graph — cell
//! arcs (input pin → output pin, delay from the NLDM tables given input slew and
//! output load) and net arcs (driver → sinks) — topologically orders it,
//! propagates arrival time + slew forward with **rise/fall split by arc
//! unateness** (so a chain alternates edges instead of taking max(rise,fall) per
//! stage), and reports WNS / TNS and the worst path.
//!
//! When a SPEF is supplied, the driver sees an **effective capacitance** (resistive
//! shielding via the net π-model, iterated with the output slew; CCS current-source
//! delay when the lib provides it) and each sink's interconnect delay comes from a
//! **transient waveform-into-RC** solve (Elmore is the fallback); crosstalk (see
//! `si`) adds a
//! window-filtered margin. Analysis covers **max-delay setup** (combinational
//! input → output, and register-to-register: a flop's Q launches via its CK→Q
//! arc, its D pins are capture endpoints with required = period − setup) **and
//! min-delay hold** (a second, min-corner forward pass; earliest data arrival at
//! each flop D must clear its hold constraint). On-chip variation is flat scalar
//! derates by default, or **AOCV** (depth-dependent derate table) / **POCV**
//! (per-stage sigma — from LVF tables when present: `std_dev`, else the late sigma on setup
//! and the early sigma on hold; else a global fraction — N-sigma band growing as
//! sqrt(depth)) when configured. Multiple
//! clocks (incl. generated) are supported — cross-domain paths use the tightest
//! launch→capture edge relation — and false-path / multicycle **exceptions** drop or
//! shift paths. With `pba` on, a path-based pass re-times the critical path and its
//! fan-in alternatives (path-local slew), without GBA's merged worst slew. Pure std
//! — unit-tested.

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap};

use crate::inc::{checks_edge, HoldRec, InEdge, IncGraph, IncState, IncTopo, Lanes, Pred, SetupRec};
use crate::job::StaJob;
use crate::liberty::{Arc, ClkEdge, Constraint, Dir, Lib};
use crate::names::{instance_of, split_inst_pin};
use crate::netlist::Netlist;
use crate::spef::compact::{Name, NO_NAME};
use crate::spef::{CompactNet, CompactSpef, Spef};

#[derive(Debug)]
pub enum StaError {
    Parse(String),
    Io(String),
    UnknownCell(String),
    /// A loop the topological order cannot pass, as the pins of one cycle in it, in order (the
    /// first pin is not repeated at the end).
    CombinationalLoop(Vec<String>),
    /// Reserved: SI/crosstalk analysis was requested but isn't modeled in v0.
    SiNotModeled,
}

impl std::fmt::Display for StaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StaError::Parse(m) => write!(f, "parse error: {m}"),
            StaError::Io(m) => write!(f, "io error: {m}"),
            StaError::UnknownCell(c) => write!(f, "cell not in any .lib: {c}"),
            StaError::CombinationalLoop(pins) => {
                // the whole cycle when it is short; a long one by its two ends
                const SHOW: usize = 24;
                let mut path: Vec<&str> = pins.iter().map(String::as_str).collect();
                let more = path.len().saturating_sub(SHOW);
                if more > 0 {
                    path.truncate(SHOW / 2);
                    path.push("\u{2026}");
                    path.extend(pins[pins.len() - SHOW / 2..].iter().map(String::as_str));
                }
                if let Some(first) = pins.first() {
                    path.push(first);
                }
                write!(
                    f,
                    "combinational loop through {} pin(s): {} (timing loops are not broken here; cut one with set_disable_timing)",
                    pins.len(),
                    path.join(" -> ")
                )
            }
            StaError::SiNotModeled => write!(f, "SI/crosstalk not modeled in v0"),
        }
    }
}
impl std::error::Error for StaError {}

/// A pin carrying only power / ground / well-bias (no signal). Used to recognise
/// physical-only cells (fill / tap / decap / antenna diode / endcap) in a
/// post-route netlist: they are absent from every timing `.lib` and connect
/// nothing, or only these rails, so they are skipped during linking rather than
/// erroring as an unknown cell.
///
/// PDK-agnostic: the exact list covers sky130 (VPWR/VGND/VPB/VNB), gf180 wells
/// (VNW/VPW), body-bias (VBP/VBN/VBB) and analog (AVDD/AVSS); the `starts_with`
/// fallbacks catch VDD*/VSS*/VPWR*/VGND* variants (VDD_CORE, VSSIO2, …) in any
/// PDK. Logic cells always carry signal pins (A/B/Y/Q/…), so broadening the rail
/// set cannot mask a genuinely missing logic library.
pub(crate) fn is_power_pin(pin: &str) -> bool {
    let p = pin.trim_start_matches('\\').to_ascii_uppercase();
    const RAILS: &[&str] = &[
        "VPWR", "VGND", "VPB", "VNB", // sky130 std cell
        "VDD", "VSS", "VDDA", "VSSA", "VCC", "VEE", "VPP", "GND", "VNEG", "VCCD", "VCCD1", "VCCD2",
        "VSSD", "VSSD1", "VSSD2", "VDDIO", "VSSIO", "VDDPST", "VSSPST", "KAPWR", "VSWITCH",
        "VCCHIB", "VNW", "VPW", "VNWELL", "VPWELL", "VWELL", "VSUBS", // wells / substrate
        "VBP", "VBN", "VBB", "VBG", "VB", "VBODY", // body bias
        "AVDD", "AVSS", "DVDD", "DVSS", "VPWRIN",
    ];
    RAILS.contains(&p.as_str())
        || p.starts_with("VDD")
        || p.starts_with("VSS")
        || p.starts_with("VPWR")
        || p.starts_with("VGND")
}

/// Interpolate an AOCV derate from a `(stages, derate)` table at `depth`, clamping
/// past the table ends. Empty table -> 1.0 (no derate).
fn aocv_lookup(tbl: &[(f64, f64)], depth: f64) -> f64 {
    if tbl.is_empty() {
        return 1.0;
    }
    if depth <= tbl[0].0 {
        return tbl[0].1;
    }
    let last = tbl[tbl.len() - 1];
    if depth >= last.0 {
        return last.1;
    }
    for w in tbl.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        if depth <= x1 {
            let t = (depth - x0) / (x1 - x0);
            return y0 + (y1 - y0) * t;
        }
    }
    last.1
}

/// Is `arc` a latch's D->Q arc (`TimingRole::latchDtoQ`)? As the reference's Liberty builder makes
/// one (`makeRegLatchArcs` -> `makeLatchDtoQArcs`): a combinational arc into a latch output from a
/// pin the cell's `latch` group names in `data_in`. ⚠️ A cell with no `latch` group (or a bank) is
/// not inferred here (`LibertyCell::inferLatchRoles`).
fn is_latch_d_to_q(cell: &crate::liberty::Cell, arc: &Arc) -> bool {
    arc.clk_edge.is_none() && is_latch_data_pin(cell, &arc.related_pin)
}

/// Is `func` (a gate output's function) a clock gate of `clk` by `enable`, and if so the logic
/// value of the enable that lets the clock through: the reference's `GatedClk::isClkGatingFunc`.
/// Outer negations are stripped; the gate must then be an AND (active value one) or an OR (zero).
/// Its operands are the subexpressions under the top operator that are not that operator; the
/// clock must be one of them, plain or negated (a negated clock flips the active value), and the
/// enable must appear somewhere among them.
fn clk_gating_active_one(func: &vyges_loom::liberty_func::Func, enable: &str, clk: &str) -> Option<bool> {
    use vyges_loom::liberty_func::Func;
    let mut f = func;
    while let Func::Not(x) = f {
        f = x;
    }
    let (mut one, and) = match f {
        Func::And(..) => (true, true),
        Func::Or(..) => (false, false),
        _ => return None,
    };
    fn operands<'a>(and: bool, e: &'a Func, out: &mut Vec<&'a Func>) {
        match (and, e) {
            (true, Func::And(a, b)) | (false, Func::Or(a, b)) => {
                operands(and, a, out);
                operands(and, b, out);
            }
            _ => out.push(e),
        }
    }
    let mut ops = Vec::new();
    operands(and, f, &mut ops);
    let mut need = false;
    for e in &ops {
        match e {
            Func::Not(x) if matches!(&**x, Func::Port(p) if p == clk) => {
                need = true;
                one = !one;
            }
            Func::Port(p) if p == clk => need = true,
            _ => {}
        }
    }
    (need && ops.iter().any(|e| e.ports().contains(&enable))).then_some(one)
}

/// Is `pin` a latch's data input: named in the `data_in` of the cell's `latch` group. Its setup
/// check is a latch check (`PathEndLatchCheck`, made where `isLatchData` holds), not a register's.
fn is_latch_data_pin(cell: &crate::liberty::Cell, pin: &str) -> bool {
    let Some(g) = cell.seq_group.as_ref().filter(|g| g.latch) else { return false };
    g.attrs.get("data_in").and_then(|d| vyges_loom::liberty_func::Func::parse_in(d.trim_matches('"'), |p| cell.pins.contains_key(p))).is_some_and(|f| f.ports().contains(&pin))
}

/// A latch D setup check, as the reference's `Latches::latchRequired` decides it, in this timer's
/// frame (times from the launch edge). `open`: the time the latch opens, as the check sees it
/// (the enable edge, its early clock latency and CRPR, less the setup uncertainty); `close`: the
/// last time data may arrive, the disable edge's ordinary setup requirement, which is `open` plus
/// the maximum borrow; `arrival`: the data. Returns `(slack, borrow)`:
/// - data at or before the opening: required is the opening, nothing borrowed;
/// - data while the latch is open: it borrows `arrival - open` and the check is met exactly
///   (required is the data's own arrival, slack 0);
/// - past the maximum borrow: borrow is that maximum and required is `close`.
fn latch_setup(open: f64, close: f64, arrival: f64) -> (f64, f64) {
    if arrival <= open {
        return (open - arrival, 0.0);
    }
    let (borrow, max_borrow) = (arrival - open, close - open);
    if borrow <= max_borrow {
        (0.0, borrow)
    } else {
        (close - arrival, max_borrow)
    }
}

/// One cycle among the nodes a topological order left out (`done[v]` false), its nodes in edge
/// order, or `None` when they hold none. A node a Kahn walk cannot reach sits on a cycle or
/// downstream of one, so a depth-first walk over the left-out nodes, in index order, finds one.
fn find_cycle(adj: &[Vec<usize>], done: &[bool]) -> Option<Vec<usize>> {
    // 0 unvisited, 1 on the current path, 2 finished
    let mut state = vec![0u8; adj.len()];
    for root in (0..adj.len()).filter(|&v| !done[v]) {
        if state[root] != 0 {
            continue;
        }
        let mut stack: Vec<(usize, usize)> = vec![(root, 0)];
        state[root] = 1;
        while let Some(&mut (u, ref mut next)) = stack.last_mut() {
            if let Some(&w) = adj[u].get(*next) {
                *next += 1;
                if done[w] {
                    continue;
                }
                match state[w] {
                    0 => {
                        state[w] = 1;
                        stack.push((w, 0));
                    }
                    1 => {
                        let at = stack.iter().position(|&(x, _)| x == w).expect("on the path");
                        return Some(stack[at..].iter().map(|&(x, _)| x).collect());
                    }
                    _ => {}
                }
            } else {
                state[u] = 2;
                stack.pop();
            }
        }
    }
    None
}

/// The reference's comparison tolerance (`Fuzzy.cc`), in its `float` arithmetic.
pub(crate) fn fuzzy_equal(v1: f32, v2: f32) -> bool {
    if v1 == v2 {
        true
    } else if v1 == 0.0 {
        v2.abs() < 1e-15
    } else if v2 == 0.0 {
        v1.abs() < 1e-15
    } else {
        (v1 - v2).abs() < 1e-6 * v1.abs().max(v2.abs())
    }
}

/// The setup and hold relationships between a LAUNCHING clock edge (`src_time` into a clock of
/// period `src_period`) and a CAPTURING one, as `(setup, hold)`: setup is the shortest time
/// from a launch to a capture edge strictly after it, hold the shortest from a capture edge at
/// or before a launch to that launch. The reference's `CycleAccting::findDelays` (setup and
/// hold roles), transcribed: its cycle walk, float tolerance and convergence test. A path's
/// arrival here is measured from its launch edge, so its setup required time is `setup` after
/// that edge and its hold required time `hold` BEFORE it.
///
/// One clock, launch on the fall (half a period) and capture on the rise: `(T/2, T/2)` — the
/// register is captured half a period after it launched, where a period-only relation gives T.
/// The time of clock `clk`'s edge `edge` (0 rise, 1 fall) within its period `period`
/// (`ClockEdge::time`): its waveform's, as `create_clock -waveform` or `create_generated_clock`
/// (`Clock::generate`) set it; with none, or no clock named, the rise at 0 and the fall half a
/// period on.
pub(crate) fn clock_edge_time(job: &StaJob, clk: Option<&str>, edge: usize, period: f64) -> f64 {
    match clk.and_then(|c| job.clock_waveforms.get(c)) {
        Some(w) => w[edge],
        None if edge == 1 => period / 2.0,
        None => 0.0,
    }
}

pub(crate) fn cycle_accting(src_period: f64, src_time: f64, tgt_period: f64, tgt_time: f64) -> (f64, f64) {
    let (setup, hold, _) = cycle_accting_gated(src_period, src_time, tgt_period, tgt_time, tgt_time);
    (setup, hold)
}

/// [`cycle_accting`], with the clock gating hold relation as well (`tgt_opp_time`: the target
/// clock's OTHER edge, which opens the gate). The reference makes a gated clock's hold check in
/// the same cycle as its setup check (`CycleAccting::findDelays`, the gated clock hold role): of
/// the target edges whose cycle's opposite edge is at or before the source edge, the one with the
/// least `src - tgt`, which may be negative (an AND gate's enable launched on the rise is held
/// against the fall after it). Returns (setup, hold, gated hold) as `src - tgt` for the holds.
pub(crate) fn cycle_accting_gated(src_period: f64, src_time: f64, tgt_period: f64, tgt_time: f64, tgt_opp_time: f64) -> (f64, f64, f64) {
    let less = |a: f32, b: f32| a < b && !fuzzy_equal(a, b);
    let less_equal = |a: f32, b: f32| a < b || fuzzy_equal(a, b);
    let greater = |a: f32, b: f32| a > b && !fuzzy_equal(a, b);
    // `MinMax::min()->initValue()`: the delays start at INF and shrink. Compared in float as the
    // reference compares them; the edges chosen are returned as their f64 difference, the
    // arithmetic the rest of this timer is in (a same-clock setup relation is the period exactly).
    let (mut setup, mut hold, mut gclk) = (1e30f32, 1e30f32, 1e30f32);
    let (mut setup_t, mut hold_t, mut gclk_t) = (tgt_period, 0.0, 0.0);
    if !(tgt_period > 0.0 && src_period > 0.0) {
        return (tgt_period, 0.0, 0.0);
    }
    let first_cycle = |time: f64, period: f64| -> i64 {
        if time < 0.0 {
            1
        } else if time < period {
            0
        } else {
            -1
        }
    };
    // related clocks (a generated clock and its source) get enough cycles to find a common period
    let tgt_max_cycle = if tgt_period > src_period { 100 } else { ((src_period / tgt_period).ceil() as i64).max(1000) };
    let (mut tgt_past_src, mut src_past_tgt) = (false, false);
    let mut tgt_cycle = first_cycle(tgt_time, tgt_period);
    while tgt_cycle <= tgt_max_cycle {
        let tgt_cycle_start = tgt_cycle as f64 * tgt_period;
        let tgt_t = tgt_cycle_start + tgt_time;
        let tgt_opp_t = tgt_cycle_start + tgt_opp_time;
        let mut src_cycle = first_cycle(src_time, src_period);
        loop {
            let src_cycle_start = src_cycle as f64 * src_period;
            let src_t = src_cycle_start + src_time;
            // both found, and the two clocks back in step
            if tgt_past_src && src_past_tgt && fuzzy_equal(src_cycle_start as f32, tgt_cycle_start as f32) {
                return (setup_t, hold_t, gclk_t);
            }
            if greater(src_cycle_start as f32, (tgt_cycle_start + tgt_period) as f32) && src_past_tgt {
                break;
            }
            // setup: the target AFTER the source
            if greater(tgt_t as f32, src_t as f32) {
                tgt_past_src = true;
                let d = (tgt_t - src_t) as f32;
                if less(d, setup) {
                    setup = d;
                    setup_t = tgt_t - src_t;
                }
            }
            // hold: the target BEFORE (or at) the source
            if less_equal(tgt_t as f32, src_t as f32) {
                src_past_tgt = true;
                let d = (src_t - tgt_t) as f32;
                if less(d, hold) {
                    hold = d;
                    hold_t = src_t - tgt_t;
                }
            }
            // gated clock hold: the same cycle as the setup check
            if less_equal(tgt_opp_t as f32, src_t as f32) {
                let d = (src_t - tgt_t) as f32;
                if less(d, gclk) {
                    gclk = d;
                    gclk_t = src_t - tgt_t;
                }
            }
            src_cycle += 1;
        }
        tgt_cycle += 1;
    }
    // no common period within the cycle limit: what was found
    (setup_t, hold_t, gclk_t)
}

/// What reaches a latch output through its D->Q arc on one launching edge's pass, per Q edge:
/// the arrival (measured from the latch's ENABLE edge, which launches it) and the predecessor
/// it is reported through (the enable pin). See the latch D->Q iteration.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LatchSeed {
    arr: [f64; 2],
    from: [Pred; 2],
}

/// One launching clock edge's arrivals (lane 0 rise, 1 fall), timed alone; see `fall_launch`.
struct EdgeRun {
    edge: usize,
    late_arr: Vec<[f64; 2]>,
    late_from: Vec<[Pred; 2]>,
    from: Vec<Option<usize>>,
    early_arr: Vec<[f64; 2]>,
    early_from: Vec<[Pred; 2]>,
    from_min: Vec<Option<usize>>,
}

/// A launching edge's arrivals by reference: an [`EdgeRun`], or the merged pass itself.
/// One arc relaxation of a recording pass: from `u`'s edge `il` to `v`'s edge `ol`, the delay it
/// added (derated, as the pass added it). See `crpr_cands`.
#[derive(Clone, Copy)]
struct ArcRec {
    u: u32,
    v: u32,
    il: u8,
    ol: u8,
    d: f64,
}

/// One launch's arrival at a pin edge: (arrival, the pin it starts at: a register clock pin or
/// an input port). See [`crpr_cands`].
type Cand = (f64, u32);

/// The arrivals a pin keeps per launch where a CRPR credit could make a later one (an earlier
/// one, setup) the worse: the reference keeps a path per CRPR clock pin and prunes, at every
/// pin, each whose arrival cannot beat the best one even with the best one's largest credit
/// (`ArrivalVisitor::pruneCrprArrivals`: drop a path when the best arrival moved by
/// `CheckCrpr::maxCrpr` of the best path's clock is still better). `recs`: one recording pass's
/// relaxations in its order; `arr` its arrivals; `start_spread(k)`: the clock path spread at
/// launch `k`'s clock pin (`maxCrpr`), 0 for an input. Returns the launches left at each pin in
/// `keep`, per edge: each edge's own, where the merged pass tracks one launch per pin. With `trace`, also that one launch's arrival and predecessor per pin edge,
/// as it survived the pruning: the path a report of it follows.
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn crpr_cands(
    recs: &[ArcRec],
    arr: &[[f64; 2]],
    is_ck: &[bool],
    skip_from: &dyn Fn(usize) -> bool,
    start_spread: &dyn Fn(usize) -> f64,
    late: bool,
    keep: &[bool],
    trace: Option<u32>,
) -> (HashMap<usize, [Vec<Cand>; 2]>, Option<(Vec<[f64; 2]>, Vec<[crate::inc::Pred; 2]>)>) {
    let n = arr.len();
    let better = |a: f64, b: f64| if late { a > b } else { a < b };
    let mut sets: Vec<[Vec<Cand>; 2]> = (0..n).map(|_| [Vec::new(), Vec::new()]).collect();
    let mut done = vec![[false; 2]; n];
    let init = if late { f64::NEG_INFINITY } else { f64::INFINITY };
    let mut tr = trace.map(|_| (vec![[init; 2]; n], vec![[None; 2]; n]));
    let finalize = |set: &mut Vec<Cand>| {
        if set.len() < 2 {
            return;
        }
        let b = set.iter().copied().reduce(|x, y| if better(y.0, x.0) { y } else { x }).unwrap();
        let m = start_spread(b.1 as usize);
        let thr = if late { b.0 - m } else { b.0 + m };
        // pruned when the best moved by its own largest credit is still better
        set.retain(|c| c.1 == b.1 || !better(thr, c.0));
    };
    let mut src: Vec<Cand> = Vec::new();
    for r in recs {
        let (u, v, il, ol) = (r.u as usize, r.v as usize, r.il as usize, r.ol as usize);
        if skip_from(u) {
            continue;
        }
        src.clear();
        if is_ck[u] {
            src.push((arr[u][il], u as u32));
        } else {
            if !done[u][il] {
                finalize(&mut sets[u][il]);
                done[u][il] = true;
            }
            if sets[u][il].is_empty() {
                if arr[u][il].is_finite() {
                    src.push((arr[u][il], u as u32));
                }
            } else {
                src.extend_from_slice(&sets[u][il]);
            }
        }
        for &(a, k) in &src {
            let na = a + r.d;
            let set = &mut sets[v][ol];
            match set.iter_mut().find(|e| e.1 == k) {
                Some(e) => {
                    if better(na, e.0) {
                        e.0 = na;
                    } else {
                        continue;
                    }
                }
                None => set.push((na, k)),
            }
            if let (Some(t), Some((ta, tp))) = (trace, tr.as_mut()) {
                if k == t && better(na, ta[v][ol]) {
                    ta[v][ol] = na;
                    tp[v][ol] = Some((u, il));
                }
            }
        }
    }
    if let (Some(t), Some((ta, _))) = (trace, tr.as_mut()) {
        // the launch's own pin
        let t = t as usize;
        ta[t] = arr[t];
    }
    let mut out = HashMap::new();
    for v in 0..n {
        if !keep[v] {
            continue;
        }
        let mut both = std::mem::take(&mut sets[v]);
        for set in both.iter_mut() {
            finalize(set);
        }
        if both.iter().any(|s| !s.is_empty()) {
            out.insert(v, both);
        }
    }
    (out, tr)
}

#[derive(Clone, Copy)]
struct EdgeView<'a> {
    edge: usize,
    late_arr: &'a [[f64; 2]],
    late_from: &'a [[Pred; 2]],
    from: &'a [Option<usize>],
    early_arr: &'a [[f64; 2]],
    early_from: &'a [[Pred; 2]],
    from_min: &'a [Option<usize>],
}

impl EdgeRun {
    fn view(&self) -> EdgeView<'_> {
        EdgeView {
            edge: self.edge,
            late_arr: &self.late_arr,
            late_from: &self.late_from,
            from: &self.from,
            early_arr: &self.early_arr,
            early_from: &self.early_from,
            from_min: &self.from_min,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PathNode {
    pub label: String,
    pub arrival: f64,
    pub slew: f64,
}

/// Where a timing pin actually **is** in the design.
///
/// [`PinId`] is an index into the timing graph and `label` is display text; neither can be used
/// to look an object up in a physical database. A `PinSite` carries the identity a downstream
/// consumer — an ECO, a report, a viewer — needs to act on the pin: the instance, its library
/// cell, the pin, and the net it sits on.
///
/// Kept separate from [`PathNode`] on purpose: resolution costs a netlist lookup, and the
/// one-shot `analyze` path never pays for it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinSite {
    pub pin_id: PinId,
    /// The timing graph's own label: `"inst/pin"`, or the port name for a primary port.
    pub label: String,
    /// Instance name — `None` for a primary port. May itself contain `/` when hierarchical.
    pub inst: Option<String>,
    /// Pin name on that instance — `None` for a primary port.
    pub pin: Option<String>,
    /// The net this pin connects to. Known for instance pins and primary ports alike.
    pub net: Option<String>,
    /// The instance's current library cell — the thing a resize move replaces.
    pub master: Option<String>,
    pub is_port: bool,
}

impl PinSite {
    /// True when the site resolved to a real instance pin, i.e. an ECO can address it.
    pub fn is_instance_pin(&self) -> bool {
        self.inst.is_some() && self.pin.is_some()
    }
}

/// Which timing check a [`Violation`] came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Check {
    /// Late / max-delay path — fixed by making the path faster (resize, buffer).
    Setup,
    /// Early / min-delay path — fixed by adding delay at the endpoint.
    Hold,
}

/// A failing (or near-failing) endpoint, resolved to something actionable.
///
/// This is the hand-off record between the timer and anything that wants to *change* the
/// design: it pairs the numbers with the identity, so a consumer never has to reverse-engineer
/// a label.
#[derive(Debug, Clone)]
pub struct Violation {
    pub check: Check,
    pub site: PinSite,
    /// Negative means failing. Worst (most negative) first when returned by
    /// [`Timer::violations`].
    pub slack: f64,
    pub arrival: f64,
    pub required: Option<f64>,
    pub slew: f64,
    /// Capacitive load on the pin — the quantity a resize or buffer move is trying to move.
    pub load: f64,
}

/// What a candidate fix did to the design's timing — `after - before`, so **positive is
/// better** for every field.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimingDelta {
    pub wns: f64,
    pub tns: f64,
    pub whs: f64,
    pub ths: f64,
}

/// Why a candidate fix was rejected. Kept explicit so a run is auditable — "reverted" on its
/// own tells you nothing when you are trying to understand why a loop failed to converge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevertReason {
    /// The metric the fix was aimed at did not move (or moved the wrong way).
    NoImprovement,
    /// It helped its target but broke setup — pushed WNS below zero, or made an existing
    /// setup violation worse.
    BrokeSetup,
    /// Same, for hold.
    BrokeHold,
}

/// The verdict on a speculative fix.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Keep,
    Revert(RevertReason),
}

impl Verdict {
    pub fn keep(&self) -> bool {
        matches!(self, Verdict::Keep)
    }
}

/// `after - before` for each headline metric.
pub fn timing_delta(before: &TimingReport, after: &TimingReport) -> TimingDelta {
    TimingDelta {
        wns: after.wns - before.wns,
        tns: after.tns - before.tns,
        whs: after.whs - before.whs,
        ths: after.ths - before.ths,
    }
}

/// Decide whether a fix aimed at `check` earned its place.
///
/// Two conditions, and the second is the interesting one:
///
/// 1. **It must help.** The targeted metric (WNS for setup, WHS for hold) must improve by more
///    than `eps`. A fix that changes nothing is not free — it costs area and a legalization
///    disturbance — so "no worse" is not good enough.
/// 2. **It must not break the other check.** Deliberately this is *not* "must not regress":
///    trading setup margin for a hold fix is normal and correct, and forbidding it would reject
///    most real hold repairs. What is forbidden is doing *harm* — pushing a met check into
///    violation, or deepening a violation that already exists.
///
/// `eps` guards against accepting floating-point noise as progress; `1e-9` (1 ps at ns units)
/// is a sane default.
pub fn judge(before: &TimingReport, after: &TimingReport, check: Check, eps: f64) -> Verdict {
    let d = timing_delta(before, after);

    // "harm" = the check ends up violating AND worse than it was
    let harmed = |before_v: f64, after_v: f64| after_v < 0.0 && after_v < before_v - eps;

    match check {
        Check::Setup => {
            if d.wns <= eps {
                return Verdict::Revert(RevertReason::NoImprovement);
            }
            if harmed(before.whs, after.whs) {
                return Verdict::Revert(RevertReason::BrokeHold);
            }
        }
        Check::Hold => {
            if d.whs <= eps {
                return Verdict::Revert(RevertReason::NoImprovement);
            }
            if harmed(before.wns, after.wns) {
                return Verdict::Revert(RevertReason::BrokeSetup);
            }
        }
    }
    Verdict::Keep
}

/// One stage of a resolved path: the pin, plus what that stage *cost*.
///
/// `stage_delay` is the arrival delta from the previous node, which is what identifies the arc
/// worth attacking — the worst path is a list of stages, and only some of them are worth fixing.
#[derive(Debug, Clone)]
pub struct PathStage {
    pub site: PinSite,
    pub arrival: f64,
    pub slew: f64,
    pub load: f64,
    /// Arrival delta from the previous stage; `0.0` for the launch point.
    pub stage_delay: f64,
}

#[derive(Debug, Clone)]
pub struct TimingReport {
    pub wns: f64, // setup worst negative slack (ns); >0 means met
    pub tns: f64, // setup total negative slack over endpoints (ns)
    pub endpoints: usize,
    pub worst_endpoint: String,
    pub worst_path: Vec<PathNode>,
    // hold (early / min-delay path) — only meaningful when there are flop endpoints
    pub whs: f64, // worst hold slack (ns); >0 means met
    pub ths: f64, // total hold negative slack (ns)
    pub hold_endpoints: usize,
    pub worst_hold_endpoint: String,
    pub worst_hold_path: Vec<PathNode>,
    // every hold endpoint and its hold slack (node index, slack ns) — the per-endpoint
    // list a hold-fix ECO ranks candidate delay insertions from. Consistent with whs/ths.
    pub hold_slacks: Vec<(PinId, f64)>,
    // path-based analysis: worst setup slack after re-timing critical paths with
    // path-local slews (Some only when `pba` is enabled): no merged worst slew at each node,
    // so never more pessimistic than the graph-based number.
    pub pba_wns: Option<f64>,
    /// The units every time and capacitance above is in: the first library's, as the
    /// reference reports (the "ns" in the field comments is that unit for a ns library).
    pub units: crate::liberty::LibUnits,
    /// The worst path of every path group, setup side then hold side, in the reference's
    /// report order (see [`CheckPath`]). Filled by a full analysis; empty after an incremental
    /// update.
    pub check_paths: Vec<CheckPath>,
    /// Every latch data pin's setup check and the time it borrows from the latch's transparent
    /// window (`actual time borrow` in the reference's report), node index and time. Empty after
    /// an incremental update (a design with latches never takes that path).
    pub latch_borrow: Vec<(PinId, f64)>,
}

/// A clock edge as a report line names it: `clock clk (rise edge)`, at `time`, with the
/// ideal clock's transition (`set_clock_transition`) where it is not propagated.
#[derive(Debug, Clone)]
pub struct ClockLine {
    pub name: String,
    pub rise: bool,
    pub time: f64,
    pub slew: f64,
    pub propagated: bool,
}

/// One node of a reported data path: the pin as `pin (cell)` / `port (in)`, its edge, its
/// absolute time and slew, and the load it drives (`None` for a load pin).
#[derive(Debug, Clone)]
pub struct PathLine {
    pub desc: String,
    pub rise: bool,
    pub time: f64,
    pub slew: f64,
    pub cap: Option<f64>,
    /// the launching register's clock pin (always printed, increment 0)
    pub clk_start: bool,
}

/// One path end in the reference's `report_checks` terms: the worst of its path group, with
/// everything the full report prints — start and end with their reasons, the launch clock and
/// its network delay (or an input's external delay), every node of the data path, the capture
/// clock, uncertainty, reconvergence pessimism, the check pin, the margin and the slack. Times
/// are absolute: the launch edge's own time added back to arrivals measured from it.
#[derive(Debug, Clone)]
pub struct CheckPath {
    /// `max` (setup / recovery / output) or `min` (hold / removal / output)
    pub max: bool,
    /// the capture clock's name, or `asynchronous` for recovery and removal checks
    pub group: String,
    pub startpoint: String,
    pub start_reason: String,
    pub endpoint: String,
    pub end_reason: String,
    pub src_clock: ClockLine,
    /// the launch clock's network delay to the register clock pin (0 from an input)
    pub src_network_delay: f64,
    /// `set_input_delay` of a path from an input port: (value, rising)
    pub input_delay: Option<(f64, bool)>,
    pub lines: Vec<PathLine>,
    pub arrival: f64,
    pub tgt_clock: ClockLine,
    pub tgt_network_delay: f64,
    /// the clock uncertainty as the line adds it (negative on the setup side)
    pub uncertainty: f64,
    /// the reconvergence pessimism as the line adds it
    pub crpr: f64,
    /// the check's clock pin, `pin (cell)`, and its edge (none for an output port)
    pub tgt_pin: Option<(String, bool)>,
    /// `library setup time`, `library hold time`, `library recovery time`,
    /// `library removal time` or `output external delay`
    pub margin_what: String,
    pub required: f64,
    pub slack: f64,
}

/// ⛔ A cell edge BORROWS its library arc: the graph lives only while `build_report` runs, and
/// the library outlives it. A boxed copy per edge deep-copied every instance's tables; on a
/// 1.6M-instance design that was most of the timer's peak memory, built and dropped each run.
enum EdgeKind<'l> {
    Net(usize), // net index — delay looked up from the per-net delay table
    Cell(&'l Arc),
}

struct Edge<'l> {
    to: usize,
    kind: EdgeKind<'l>,
    /// `[from edge][to edge]` the constants let through (0 rise, 1 fall) — see [`crate::sim`]: an
    /// arc's sense once the held pins are substituted, and a pin held rising or falling. All
    /// true without case analysis; a net arc reads the diagonal.
    lanes: [[bool; 2]; 2],
    /// A latch's D->Q arc (`TimingRole::latchDtoQ`). The reference's levelization does not search
    /// through it (`Levelize::searchThru`): a latch output is ordered by its enable alone, so a loop
    /// closed only through a latch is no loop, and what reaches Q through D is found separately.
    latch_dq: bool,
}

const ALL_LANES: [[bool; 2]; 2] = [[true; 2]; 2];

impl Edge<'_> {
    /// Does any arrival cross this arc? (A cut arc stays for the slews after it.)
    fn searched(&self) -> bool {
        self.lanes != [[false; 2]; 2]
    }
}

/// A receiver pin's load, `[analysis][edge]`: analysis 0 is EARLY (min caps, hold), 1 LATE
/// (max caps, setup); edge 0 rise, 1 fall of the driving output. The rise / fall capacitance —
/// its `_range` minimum for an early analysis — and the plain `capacitance` where the library
/// states no per-edge value. One rule for the full and the incremental timer.
///
/// ⛔ A CCS `receiver_capacitance` model does NOT replace it: the reference's default delay
/// calculator reduces each net with `Sdc::pinCapacitance` (`ReduceToPi::pinCapacitance`), the
/// Liberty port's capacitance for that edge and analysis, whatever receiver model the pin also
/// carries. Measured on the reference's reg1_asap7 test: u2/B loads u1/Y's rising edge with
/// 0.526025 fF (`rise_capacitance`, the range maximum); the receiver model gave 0.3034.
pub(crate) fn pin_load_caps(p: &crate::liberty::Pin) -> [[f64; 2]; 2] {
    [[p.caps.get(true, false), p.caps.get(false, false)], [p.caps.get(true, true), p.caps.get(false, true)]]
}

struct Net {
    driver: Option<usize>,
    sinks: Vec<usize>,
    /// Receiver pin caps + port loads, per edge of the DRIVER's output (lane 0 rise, 1 fall):
    /// the reference loads each edge with the pins' capacitance for that edge.
    load: [[f64; 2]; 2],
}

/// Opaque handle to a timing-graph pin/node — an index into the committed graph. Returned
/// by the query API (e.g. [`Timer::endpoint_slacks`]) and accepted by the per-pin queries.
pub type PinId = usize;

/// Per-pin snapshot the [`Timer`] retains for its query API. Captured at the end of a build;
/// indexed by [`PinId`]. (A later phase grows this into a mutable graph for dirty-cone
/// incremental recompute.)
#[derive(Clone)]
pub(crate) struct Timing {
    labels: Vec<String>,
    label2idx: HashMap<String, usize>,
    is_endpoint: Vec<bool>,
    excluded_setup: Vec<bool>, // false-path endpoints (no setup check)
    arrival: Vec<f64>,         // latest (setup) arrival, ns
    // Latest arrival carried from a clock SOURCE's own edge (rise at 0, fall at half a
    // period), stopped at every flop CK pin. `arrival` measures delay from the launch
    // edge, which cannot express a path that *is* the clock leaving the block; where both
    // reach an endpoint the later one is the one it has to meet. NEG_INFINITY = no such path.
    clk_arrival: Vec<f64>,
    slew: Vec<f64>,         // setup-corner output slew, ns
    arr_min: Vec<f64>,      // earliest (hold) arrival, ns
    // Hold-corner output slew, ns. Carried beside `slew` because a slew is what a
    // downstream delay is looked up at: when two timers disagree about a DELAY, the
    // question is almost always whether they agree about the SLEW that produced it, and
    // comparing our late slew against a reference's min-path slew answers nothing.
    slew_min: Vec<f64>,
    node_load: Vec<f64>,    // driver-node capacitive load, pF
    endpoint_req: Vec<f64>, // required time (meaningful at setup endpoints)
    // Per EDGE (lane 0 rise, 1 fall), setup corner: the slew at each pin and the load on each
    // driver that the timer looked its delays up at. The collapsed `slew` / `node_load` above
    // are one value per pin (the worst lane), which no single arc is computed from.
    slew_rf: Vec<[f64; 2]>,
    load_rf: Vec<[f64; 2]>,
    // The arrival each setup end's slack was taken against: the checked launching edge's and
    // data edge's, not the latest over them (`arrival`). NaN = not recorded (the incremental
    // path keeps none; see `Timer::check_arrival`).
    check_arr: Vec<f64>,
}

impl Timing {
    /// Assemble the query snapshot from the propagated arrays — shared by the full builder
    /// and the incremental fast path so both produce an identical snapshot.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        labels: &[String],
        is_endpoint: &[bool],
        excluded_setup: &[bool],
        arrival: &[f64],
        clk_arrival: &[f64],
        slew: &[f64],
        arr_min: &[f64],
        slew_min: &[f64],
        node_load: &[f64],
        endpoint_req: &[f64],
        slew_rf: &[[f64; 2]],
        load_rf: &[[f64; 2]],
    ) -> Timing {
        Timing {
            label2idx: labels
                .iter()
                .enumerate()
                .map(|(i, l)| (l.clone(), i))
                .collect(),
            labels: labels.to_vec(),
            is_endpoint: is_endpoint.to_vec(),
            excluded_setup: excluded_setup.to_vec(),
            arrival: arrival.to_vec(),
            clk_arrival: clk_arrival.to_vec(),
            slew: slew.to_vec(),
            arr_min: arr_min.to_vec(),
            slew_min: slew_min.to_vec(),
            node_load: node_load.to_vec(),
            endpoint_req: endpoint_req.to_vec(),
            slew_rf: slew_rf.to_vec(),
            load_rf: load_rf.to_vec(),
            check_arr: Vec::new(),
        }
    }
}

/// A netlist edit an optimizer stages on a [`Timer`]. (Cell swap — resize / Vt-swap — and
/// delay insertion; removal lands with the topology-aware recompute.)
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Move {
    /// Replace an instance's library cell with another (same logic function): the resize /
    /// Vt-swap move. `inst` is the instance name; `cell` the new library cell name.
    Resize { inst: String, cell: String },
    /// Splice a delay element into the net feeding `inst`/`pin` — the hold-repair move.
    ///
    /// The sink is re-pointed at a new net driven by a new instance of `cell`, so the
    /// topology becomes `driver -> (old net) -> new cell -> (new net) -> inst/pin`. Nothing
    /// else on the old net moves, so only this one sink gets the extra delay.
    ///
    /// `cell` must be non-inverting for the result to be logically equivalent — the timer
    /// does not check that (it has no notion of function), so the caller owns it.
    InsertDelay {
        inst: String,
        pin: String,
        cell: String,
        /// Instance name for the new cell. Must not already exist.
        name: String,
    },
}

/// An opaque saved state for speculative apply/undo: `checkpoint()` captures it, `restore()`
/// rolls back to it (a swap of the cached state + working netlist; the incremental graph
/// state, when present, rolls back with it so speculation stays cheap).
pub struct Checkpoint {
    nl: Netlist,
    // must travel WITH `nl`: a rolled-back InsertDelay removes an instance, and an index still
    // naming it would resolve to the wrong element (or past the end) on the next lookup.
    inst_idx: HashMap<String, usize>,
    report: TimingReport,
    timing: Timing,
    dirty: bool,
    inc_state: Option<IncState>,
    pending: HashMap<String, (String, String)>,
}

/// A persistent timing session.
///
/// Built once from a netlist + Liberty (+ SPEF), it caches the [`TimingReport`] + a per-pin
/// snapshot (the **query API**: per-pin arrival/slew/load, endpoint required/slack, the
/// endpoint-slack ranking, the critical path) and **retains the inputs** so an optimizer can
/// stage netlist mutations ([`Timer::stage`] / [`Timer::resize`]), recompute ([`Timer::update`]),
/// and speculate ([`Timer::checkpoint`] / [`Timer::restore`]). `analyze` does not use the
/// `Timer` (it calls the builder directly), so the one-shot report stays byte-identical.
pub struct Timer {
    report: TimingReport,
    timing: Timing,
    // retained inputs — the working design the optimizer mutates and the Timer recomputes from.
    nl: Netlist,
    lib: Lib,
    job: StaJob,
    // the analysis form of the SPEF, shared: a rebuild and a checkpoint read the same one
    spef: Option<std::sync::Arc<CompactSpef>>,
    dirty: bool, // a mutation is staged but not yet folded into the cached report
    // incremental fast path (Some only in the simple timing context — see `inc`).
    inc: Option<IncGraph>,
    // cell swaps staged since the last `update`: inst → (original cell, current cell). The
    // original is the cell as of the last update, so a re-staged instance keeps one delta.
    pending: HashMap<String, (String, String)>,
    n_inc: u64,  // updates served by the cone-localized fast path
    n_full: u64, // updates that fell back to a full re-analysis
    // instance name -> position in `nl.insts`, for O(1) site resolution and staging. Stable:
    // `stage` only rewrites an instance's `cell` in place, never adds, removes or reorders.
    inst_idx: HashMap<String, usize>,
}

impl Timer {
    /// Build the timing state, run the analysis once, and retain the inputs for mutation. `O(N)`.
    ///
    /// The SPEF is converted to its analysis form ([`CompactSpef`]) for the Timer to keep; a
    /// caller that has no further use for the [`Spef`] saves that copy with
    /// [`Timer::build_compact`].
    pub fn build(
        nl: &Netlist,
        lib: &Lib,
        job: &StaJob,
        spef: Option<&Spef>,
    ) -> Result<Timer, StaError> {
        Timer::build_compact(nl, lib, job, spef.map(|s| std::sync::Arc::new(CompactSpef::from_ref(s))))
    }

    /// [`Timer::build`] from the SPEF's analysis form, which the Timer keeps (shared, not copied).
    pub fn build_compact(
        nl: &Netlist,
        lib: &Lib,
        job: &StaJob,
        spef: Option<std::sync::Arc<CompactSpef>>,
    ) -> Result<Timer, StaError> {
        let (report, timing, inc) = build_report(nl, lib, job, spef.as_deref(), true)?;
        Ok(Timer {
            report,
            timing,
            nl: nl.clone(),
            lib: lib.clone(),
            job: job.clone(),
            spef,
            dirty: false,
            inc,
            pending: HashMap::new(),
            n_inc: 0,
            n_full: 0,
            inst_idx: nl
                .insts
                .iter()
                .enumerate()
                .map(|(i, inst)| (inst.name.clone(), i))
                .collect(),
        })
    }

    /// The cached sign-off report (WNS/TNS/WHS/THS + worst paths).
    pub fn report(&self) -> &TimingReport {
        &self.report
    }

    /// Setup worst negative slack (ns); `> 0` means met.
    pub fn wns(&self) -> f64 {
        self.report.wns
    }
    /// Setup total negative slack over endpoints (ns).
    pub fn tns(&self) -> f64 {
        self.report.tns
    }
    /// Hold worst slack (ns); `> 0` means met.
    pub fn whs(&self) -> f64 {
        self.report.whs
    }
    /// Hold total negative slack (ns).
    pub fn ths(&self) -> f64 {
        self.report.ths
    }

    // ---- Phase 1 query API (reads of the committed snapshot) ----

    /// Number of pins (timing-graph nodes).
    pub fn num_pins(&self) -> usize {
        self.timing.labels.len()
    }
    /// Resolve a human label (`"port"` or `"inst/pin"`) to its handle.
    pub fn pin(&self, label: &str) -> Option<PinId> {
        self.timing.label2idx.get(label).copied()
    }
    /// The label of a pin.
    pub fn pin_label(&self, p: PinId) -> &str {
        self.timing.labels.get(p).map(String::as_str).unwrap_or("")
    }
    /// Whether `p` is a setup capture endpoint (a primary output or a flop data pin).
    pub fn is_endpoint(&self, p: PinId) -> bool {
        self.timing.is_endpoint.get(p).copied().unwrap_or(false)
    }
    /// Latest (setup / late) arrival time at `p`, ns.
    pub fn arrival(&self, p: PinId) -> f64 {
        self.timing
            .arrival
            .get(p)
            .copied()
            .unwrap_or(f64::NEG_INFINITY)
    }
    /// Earliest (hold / early) arrival time at `p`, ns.
    pub fn arrival_min(&self, p: PinId) -> f64 {
        self.timing.arr_min.get(p).copied().unwrap_or(f64::INFINITY)
    }
    /// Setup-corner output slew at `p`, ns.
    pub fn slew(&self, p: PinId) -> f64 {
        self.timing.slew.get(p).copied().unwrap_or(0.0)
    }
    /// Hold-corner (early) output slew at `p`, ns — the slew the min-delay path was
    /// looked up at. Compare this, not [`Timer::slew`], against another timer's min path.
    pub fn slew_min(&self, p: PinId) -> f64 {
        self.timing.slew_min.get(p).copied().unwrap_or(0.0)
    }
    /// Capacitive load on `p` when it drives a net, pF (0 if `p` is not a net driver).
    pub fn load(&self, p: PinId) -> f64 {
        self.timing.node_load.get(p).copied().unwrap_or(0.0)
    }
    /// Setup-corner slew at `p` on ONE edge (`rise`, else fall), ns: the input slew of every
    /// arc that edge drives. [`Timer::slew`] is the worst lane's, which is the right input slew
    /// for only one of an arc's two output edges.
    pub fn slew_edge(&self, p: PinId, rise: bool) -> f64 {
        self.timing.slew_rf.get(p).map_or(0.0, |s| s[usize::from(!rise)])
    }
    /// Setup-corner load on driver `p` for ONE output edge (`rise`, else fall), pF: Liberty
    /// pin caps may differ by edge (`rise_capacitance` / `fall_capacitance`), and each output
    /// edge's delay is looked up at its own. [`Timer::load`] is the larger of the two.
    pub fn load_edge(&self, p: PinId, rise: bool) -> f64 {
        self.timing.load_rf.get(p).map_or(0.0, |l| l[usize::from(!rise)])
    }
    /// The arrival a setup check at `p` must meet: the later of the launch-edge-relative
    /// arrival and a clock-source-launched one. See [`Timing::clk_arrival`].
    pub fn setup_arrival(&self, p: PinId) -> f64 {
        let a = self.arrival(p);
        let c = self
            .timing
            .clk_arrival
            .get(p)
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        if c > a {
            c
        } else {
            a
        }
    }
    /// Required time at `p` — `Some` only at a reached, non-false-path setup endpoint.
    pub fn required(&self, p: PinId) -> Option<f64> {
        let excluded = self.timing.excluded_setup.get(p).copied().unwrap_or(false);
        (self.is_endpoint(p) && self.setup_arrival(p).is_finite() && !excluded)
            .then(|| self.timing.endpoint_req[p])
    }
    /// The arrival the setup slack at `p` was taken against: that of the launching edge and data
    /// edge whose check set it, which a reference timer's path to `p` reports. [`Timer::
    /// setup_arrival`] is the latest over them. Falls back to it where none was recorded (an
    /// end launched from a clock source, or after an incremental update). `Some` only at a setup
    /// endpoint.
    pub fn check_arrival(&self, p: PinId) -> Option<f64> {
        self.required(p).map(|_| self.timing.check_arr.get(p).copied().filter(|a| a.is_finite()).unwrap_or_else(|| self.setup_arrival(p)))
    }
    /// Setup slack at `p` (`required − arrival`) — `Some` only at a setup endpoint.
    pub fn slack(&self, p: PinId) -> Option<f64> {
        self.required(p).map(|r| r - self.setup_arrival(p))
    }
    /// Every setup endpoint and its slack, worst (most negative) first — the list an
    /// optimizer ranks candidate moves from. Consistent with `wns`/`tns`.
    pub fn endpoint_slacks(&self) -> Vec<(PinId, f64)> {
        let t = &self.timing;
        let mut v: Vec<(PinId, f64)> = (0..t.labels.len())
            .filter(|&p| {
                t.is_endpoint[p] && self.setup_arrival(p).is_finite() && !t.excluded_setup[p]
            })
            .map(|p| (p, t.endpoint_req[p] - self.setup_arrival(p)))
            .collect();
        v.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        v
    }
    /// The worst (critical) setup path.
    pub fn worst_path(&self) -> &[PathNode] {
        &self.report.worst_path
    }
    /// Every hold endpoint and its hold slack, worst (most negative) first — the list a
    /// hold-fix ECO ranks candidate delay insertions from. Consistent with `whs`/`ths`.
    pub fn hold_endpoint_slacks(&self) -> Vec<(PinId, f64)> {
        let mut v = self.report.hold_slacks.clone();
        v.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        v
    }

    // ---- addressability: turning timing results into things you can act on ----

    /// Resolve a [`PinId`] to **where it is** — instance, library cell, pin and net.
    ///
    /// This is the bridge from timing results to a physical database: `PinId` is a graph index
    /// and `pin_label` is display text, but a `PinSite` names objects a netlist or an `.odb`
    /// can look up.
    ///
    /// Labels are `"inst/pin"`, and an instance name may itself contain `/` when the design is
    /// hierarchical — so the split is on the **last** separator, and the result is confirmed
    /// against the netlist rather than trusted. Anything that does not resolve to a known
    /// instance is reported as a primary port (ports drive a net of the same name).
    pub fn pin_site(&self, p: PinId) -> PinSite {
        let label = self.timing.labels[p].clone();
        if let Some(cut) = label.rfind('/') {
            let (inst_name, pin_name) = (&label[..cut], &label[cut + 1..]);
            if let Some(&i) = self.inst_idx.get(inst_name) {
                let inst = &self.nl.insts[i];
                return PinSite {
                    pin_id: p,
                    inst: Some(inst.name.clone()),
                    pin: Some(pin_name.to_string()),
                    net: inst
                        .conns
                        .iter()
                        .find(|(pin, _)| pin == pin_name)
                        .map(|(_, net)| net.clone()),
                    master: Some(inst.cell.clone()),
                    is_port: false,
                    label,
                };
            }
        }
        PinSite {
            pin_id: p,
            net: Some(label.clone()), // a primary port drives a net of the same name
            label,
            inst: None,
            pin: None,
            master: None,
            is_port: true,
        }
    }

    /// Failing endpoints for `check`, worst first, resolved to actionable sites.
    ///
    /// Only endpoints with **negative** slack are returned — this answers "what is broken",
    /// not "what is the ranking". `limit` caps the result (`0` means no cap); the ordering is
    /// the same worst-first order [`endpoint_slacks`](Self::endpoint_slacks) and
    /// [`hold_endpoint_slacks`](Self::hold_endpoint_slacks) already guarantee, so taking the
    /// first N is taking the N worst.
    pub fn violations(&self, check: Check, limit: usize) -> Vec<Violation> {
        let ranked = match check {
            Check::Setup => self.endpoint_slacks(),
            Check::Hold => self.hold_endpoint_slacks(),
        };
        let mut out = Vec::new();
        for (p, slack) in ranked {
            if slack >= 0.0 {
                break; // sorted worst-first: the first non-negative ends the failing run
            }
            out.push(Violation {
                check,
                site: self.pin_site(p),
                slack,
                arrival: match check {
                    Check::Setup => self.arrival(p),
                    Check::Hold => self.arrival_min(p),
                },
                required: self.required(p),
                slew: self.slew(p),
                load: self.load(p),
            });
            if limit != 0 && out.len() >= limit {
                break;
            }
        }
        out
    }

    /// The critical setup path as resolved stages, each carrying what it cost.
    ///
    /// [`worst_path`](Self::worst_path) gives labels and arrivals; this adds identity and the
    /// per-stage delay, which is what picks the arc worth attacking — a long path is rarely
    /// uniformly slow.
    pub fn worst_path_stages(&self) -> Vec<PathStage> {
        let mut prev: Option<f64> = None;
        self.report
            .worst_path
            .iter()
            .map(|n| {
                let stage_delay = prev.map_or(0.0, |a| n.arrival - a);
                prev = Some(n.arrival);
                // the report's path carries labels; recover the graph id to reach the site
                let pin_id = self.timing.label2idx.get(&n.label).copied();
                PathStage {
                    site: pin_id.map_or_else(
                        || PinSite {
                            pin_id: usize::MAX,
                            label: n.label.clone(),
                            inst: None,
                            pin: None,
                            net: None,
                            master: None,
                            is_port: false,
                        },
                        |p| self.pin_site(p),
                    ),
                    arrival: n.arrival,
                    slew: n.slew,
                    load: pin_id.map_or(0.0, |p| self.load(p)),
                    stage_delay,
                }
            })
            .collect()
    }

    // ---- mutation + update (Phase 2) ----

    /// The current (possibly mutated) working netlist — materialize the resized design.
    pub fn netlist(&self) -> &Netlist {
        &self.nl
    }

    /// The merged Liberty the timer is working from — what a repair rule queries for
    /// interchangeable cells (`Lib::upsize_candidates` and friends).
    pub fn lib(&self) -> &Lib {
        &self.lib
    }

    /// Stage a netlist mutation. Returns `false` if it doesn't apply (e.g. unknown instance).
    /// Nothing is recomputed until [`update`](Self::update); the cached report/queries reflect
    /// the *last updated* state until then.
    pub fn stage(&mut self, m: Move) -> bool {
        match m {
            Move::Resize { inst, cell } => {
                // O(1) via the same index `pin_site` uses, instead of scanning every instance
                // on every staged move.
                match self.inst_idx.get(&inst).map(|&i| &mut self.nl.insts[i]) {
                    Some(i) => {
                        let old = i.cell.clone();
                        i.cell = cell.clone();
                        // record the move for the incremental path, preserving the cell as of the
                        // last update as the "original" so repeated stages of one instance coalesce.
                        self.pending
                            .entry(inst)
                            .and_modify(|e| e.1 = cell.clone())
                            .or_insert((old, cell));
                        self.dirty = true;
                        true
                    }
                    None => false,
                }
            }
            Move::InsertDelay {
                inst,
                pin,
                cell,
                name,
            } => {
                if self.inst_idx.contains_key(&name) {
                    return false; // the new instance name is already taken
                }
                // the delay cell's own pins come from the library, not from a naming convention
                let Some(lib_cell) = self.lib.cells.get(&cell) else {
                    return false;
                };
                let pin_named = |d: crate::liberty::Dir| {
                    lib_cell
                        .pins
                        .values()
                        .find(|p| p.direction == d && !p.clock)
                        .map(|p| p.name.clone())
                };
                let (Some(cell_in), Some(cell_out)) = (
                    pin_named(crate::liberty::Dir::In),
                    pin_named(crate::liberty::Dir::Out),
                ) else {
                    return false; // not usable as a delay element
                };

                // find the sink and the net it currently sits on
                let Some(&si) = self.inst_idx.get(&inst) else {
                    return false;
                };
                let Some(ci) = self.nl.insts[si].conns.iter().position(|(p, _)| *p == pin) else {
                    return false;
                };
                let old_net = self.nl.insts[si].conns[ci].1.clone();

                // a fresh net between the new cell and the sink; derived from the instance name
                // so a plan replayed elsewhere produces the same names.
                let new_net = format!("{name}_n");
                if self
                    .nl
                    .insts
                    .iter()
                    .any(|i| i.conns.iter().any(|(_, n)| *n == new_net))
                {
                    return false; // refuse to collide rather than silently merge two nets
                }

                self.nl.insts[si].conns[ci].1 = new_net.clone();
                self.nl.insts.push(crate::netlist::Inst {
                    cell,
                    name: name.clone(),
                    conns: vec![(cell_in, old_net), (cell_out, new_net)],
                    ties: Vec::new(),
                });
                self.inst_idx.insert(name, self.nl.insts.len() - 1);
                // topology changed, so the cone-localized incremental path cannot be trusted:
                // drop it and let `update` fall back to a full re-analysis.
                self.inc = None;
                self.dirty = true;
                true
            }
        }
    }

    /// Convenience for the common move: swap `inst`'s library cell (resize / Vt-swap).
    pub fn resize(&mut self, inst: &str, cell: &str) -> bool {
        self.stage(Move::Resize {
            inst: inst.to_string(),
            cell: cell.to_string(),
        })
    }

    /// Whether a staged mutation is pending an [`update`](Self::update).
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Recompute the timing after staged mutations and refresh the cached report + query
    /// snapshot.
    ///
    /// Takes the **cone-localized fast path** when an [`IncGraph`] is present and the staged
    /// moves are localizable (combinational cell swaps in the simple timing context) —
    /// recomputing only the forward cone, `O(cone)`. Otherwise (SPEF/SI, AOCV/POCV, a
    /// clock-network or footprint-changing edit, or a launch-flop change) it falls back to a
    /// full re-analysis, which is always correct and re-captures a fresh graph.
    pub fn update(&mut self) -> Result<&TimingReport, StaError> {
        if !self.dirty {
            return Ok(&self.report);
        }
        // fast path: try the incremental cone recompute.
        if self.inc.is_some() {
            let moves: Vec<(String, String, String)> = self
                .pending
                .iter()
                .map(|(inst, (old, new))| (inst.clone(), old.clone(), new.clone()))
                .collect();
            let inc = self.inc.as_mut().unwrap();
            if let Some((report, timing)) = inc.try_update(&self.nl, &self.lib, &moves) {
                self.report = report;
                self.timing = timing;
                self.dirty = false;
                self.pending.clear();
                self.n_inc += 1;
                return Ok(&self.report);
            }
            // not localizable — drop the stale graph; the full rebuild re-captures it.
            self.inc = None;
        }
        // full re-analysis (and re-capture the incremental graph when eligible).
        let (report, timing, inc) =
            build_report(&self.nl, &self.lib, &self.job, self.spef.as_deref(), true)?;
        self.report = report;
        self.timing = timing;
        self.inc = inc;
        self.dirty = false;
        self.pending.clear();
        self.n_full += 1;
        Ok(&self.report)
    }

    /// How many `update()` calls were served by the cone-localized fast path vs a full
    /// re-analysis: `(incremental, full)`. Lets an optimizer report its incremental hit rate.
    pub fn update_stats(&self) -> (u64, u64) {
        (self.n_inc, self.n_full)
    }

    /// Capture the current state for speculative apply/undo (the optimizer's keep-best loop:
    /// checkpoint → stage candidate → update → read → [`restore`](Self::restore) if rejected).
    pub fn checkpoint(&self) -> Checkpoint {
        Checkpoint {
            nl: self.nl.clone(),
            inst_idx: self.inst_idx.clone(),
            report: self.report.clone(),
            timing: self.timing.clone(),
            dirty: self.dirty,
            // snapshot only the mutable incremental state (the immutable topology is shared
            // and never changes), so a rejected speculative move rolls back in `O(N)` array
            // copies rather than a graph rebuild.
            inc_state: self.inc.as_ref().map(|g| g.state.clone()),
            pending: self.pending.clone(),
        }
    }

    /// Roll back to a [`Checkpoint`] — restores the working netlist and cached timing with no
    /// recompute.
    pub fn restore(&mut self, c: Checkpoint) {
        self.nl = c.nl;
        self.inst_idx = c.inst_idx;
        self.report = c.report;
        self.timing = c.timing;
        self.dirty = c.dirty;
        self.pending = c.pending;
        if let (Some(g), Some(state)) = (self.inc.as_mut(), c.inc_state) {
            g.state = state;
        }
    }
}

/// Run combinational max-delay STA and return the slack report.
///
/// Thin wrapper over [`Timer::build`] — kept for one-shot callers; the report is identical
/// to building a [`Timer`] and reading [`Timer::report`].
pub fn analyze(
    nl: &Netlist,
    lib: &Lib,
    job: &StaJob,
    spef: Option<&Spef>,
) -> Result<TimingReport, StaError> {
    analyze_compact(nl, lib, job, spef.map(CompactSpef::from_ref).as_ref())
}

/// [`analyze`] from the SPEF's analysis form.
pub fn analyze_compact(
    nl: &Netlist,
    lib: &Lib,
    job: &StaJob,
    spef: Option<&CompactSpef>,
) -> Result<TimingReport, StaError> {
    Ok(build_report(nl, lib, job, spef, false)?.0)
}

/// The factors that put a parasitic's femtofarads and ohms in the library's units: femtofarads
/// per capacitance unit, ohms per resistance unit (time unit / capacitance unit, so R·C is in the
/// time unit: 1 kΩ for both ns·pF and ps·fF) and time units per femtosecond (an Ω·fF product).
/// 1000, 1000 and 1e-6 for a ns, pF library — each an exact power of ten, so those runs are
/// unchanged to the bit.
fn spef_scales(u: crate::liberty::LibUnits) -> (f64, f64, f64) {
    (pow10_exact(u.cap_f / 1e-15), pow10_exact(u.time_s / u.cap_f), pow10_exact(1e-15 / u.time_s))
}

/// A register pin's checks of one kind (setup, hold, recovery or removal): the library's own list,
/// borrowed, when `keep` passes every check, as it does unless `set_disable_timing` names one;
/// otherwise the checks it keeps, in library order.
fn checks_of<'l>(cons: &'l [Constraint], keep: impl Fn(&Constraint) -> bool) -> Cow<'l, [Constraint]> {
    if cons.iter().all(&keep) {
        Cow::Borrowed(cons)
    } else {
        Cow::Owned(cons.iter().filter(|c| keep(c)).cloned().collect())
    }
}

/// A net's reduced Elmore as the walk lists it (`elmore_of`), held for lookup by node: sorted by
/// node, a node the walk lists twice keeping its LATER value, which is what collecting the walk
/// into a map gave (an insert replaces). Each pass reads it once per load, so it is searched,
/// not hashed: no per-node hash and no table growth on every net of every pass.
fn elmore_by_node(mut walk: Vec<(Name, f64)>) -> Vec<(Name, f64)> {
    walk.sort_by_key(|e| e.0); // stable: a node's repeats stay in walk order
    let mut by_node: Vec<(Name, f64)> = Vec::with_capacity(walk.len());
    for e in walk {
        match by_node.last_mut() {
            Some(last) if last.0 == e.0 => *last = e,
            _ => by_node.push(e),
        }
    }
    by_node
}

/// The Elmore `elmore_by_node` holds for `node`.
fn elmore_at(by_node: &[(Name, f64)], node: Name) -> Option<f64> {
    by_node.binary_search_by_key(&node, |e| e.0).ok().map(|j| by_node[j].1)
}

/// `DelayCalcBase::thresholdAdjust`: a wire delay and load slew found at the DRIVER library's
/// thresholds, re-expressed at the LOAD library's. The reference calls it only when the two
/// libraries differ; with equal thresholds it is the identity. Its thresholds are floats, and so
/// is its arithmetic, as here. ⚠️ The delay shift is ADDED on a rising edge and SUBTRACTED on a
/// falling one, as the reference writes it.
fn threshold_adjust(
    drv: &crate::liberty::Thresholds,
    load: &crate::liberty::Thresholds,
    rise: bool,
    wire_delay: f64,
    load_slew: f64,
) -> (f64, f64) {
    let pick = |t: &crate::liberty::Thresholds, out: bool| {
        let vth = match (rise, out) {
            (true, true) => t.output_rise,
            (false, true) => t.output_fall,
            (true, false) => t.input_rise,
            (false, false) => t.input_fall,
        };
        let (lo, hi) = if rise { (t.slew_lower_rise, t.slew_upper_rise) } else { (t.slew_lower_fall, t.slew_upper_fall) };
        (vth as f32, lo as f32, hi as f32, t.slew_derate as f32)
    };
    let (drvr_vth, d_lo, d_hi, drvr_slew_derate) = pick(drv, true);
    let (load_vth, l_lo, l_hi, load_slew_derate) = pick(load, false);
    let drvr_slew_delta = d_hi - d_lo;
    let wire_delay_delta = (load_slew as f32) * ((load_vth - drvr_vth) / drvr_slew_delta);
    let wire_delay = wire_delay + if rise { wire_delay_delta as f64 } else { -(wire_delay_delta as f64) };
    let load_slew_delta = l_hi - l_lo;
    let load_slew = load_slew * ((load_slew_delta / load_slew_derate) / (drvr_slew_delta / drvr_slew_derate)) as f64;
    (wire_delay, load_slew)
}

/// `x` as the exact power of ten it rounds to (`1.0 / 10^k` below one, as the literal reads),
/// else `x` itself.
pub(crate) fn pow10_exact(x: f64) -> f64 {
    let k = x.log10().round() as i32;
    let p = if k >= 0 { 10f64.powi(k) } else { 1.0 / 10f64.powi(-k) };
    if ((p - x) / x).abs() < 1e-9 {
        p
    } else {
        x
    }
}

/// Build the timing graph, propagate arrival/required, and return the report.
///
/// (Phase 1+ splits this into persistent graph construction + propagation that the
/// [`Timer`] retains for per-pin queries and incremental dirty-cone updates.)
fn build_report(
    nl: &Netlist,
    lib: &Lib,
    job: &StaJob,
    spef: Option<&CompactSpef>,
    capture: bool,
) -> Result<(TimingReport, Timing, Option<IncGraph>), StaError> {
    // A clock source written as a pattern (`create_clock … [get_ports *clk*]`) is every port it
    // matches, as the reference's `get_ports` resolves it. Kept literal it named no port, the
    // clock reached nothing, and every register went unchecked ("no constrained paths").
    // the reference's `makeGraph`, `findDelays`, `findArrivals` and `findPathEnds`, one stage line
    // each with VYGES_STAGE_TIMES=1
    let stage_graph = crate::debug::stage("makeGraph");
    // the debug groups, read once per run: they print, they never change a result
    let dmp_trace = crate::debug::on(crate::debug::DMP_CEFF);
    let expanded;
    let job = if job.clocks.iter().any(|(_, s, _)| s.contains(['*', '?'])) {
        let ports: Vec<&String> = nl.inputs.iter().chain(nl.outputs.iter()).collect();
        let mut j = job.clone();
        j.clocks = job
            .clocks
            .iter()
            .flat_map(|(n, s, p)| {
                if s.contains(['*', '?']) {
                    ports.iter().filter(|q| vyges_loom::sdc::object_matches(s, q)).map(|q| (n.clone(), (*q).clone(), *p)).collect()
                } else {
                    vec![(n.clone(), s.clone(), *p)]
                }
            })
            .collect();
        if j.clock_port.contains(['*', '?']) {
            j.clock_port = j.clocks.first().map(|c| c.1.clone()).unwrap_or_default();
        }
        expanded = j;
        &expanded
    } else {
        job
    };
    // The parasitics' femtofarads and ohms in the library's units (see `spef_scales`).
    let (ff_per_cap, ohm_per_r, fs_to_time) = spef_scales(lib.units);
    let r_scale = 1.0 / ohm_per_r;
    // `gateModelRd`'s 1e-15 F step in the library's capacitance unit (1e-3 for pF, 1 for fF)
    let cap_step = 1.0 / ff_per_cap;
    // the RC solves (Elmore, transient) work in ns: their times to and from the library's unit
    let (ns_per_time, time_per_ns) = (pow10_exact(lib.units.time_s / 1e-9), pow10_exact(1e-9 / lib.units.time_s));
    let mut labels: Vec<String> = Vec::new();
    let mut key2idx: HashMap<String, usize> = HashMap::new();
    let mut is_endpoint: Vec<bool> = Vec::new();

    let node = |key: String,
                label: String,
                key2idx: &mut HashMap<String, usize>,
                labels: &mut Vec<String>,
                is_endpoint: &mut Vec<bool>|
     -> usize {
        if let Some(&i) = key2idx.get(&key) {
            return i;
        }
        let i = labels.len();
        labels.push(label);
        is_endpoint.push(false);
        key2idx.insert(key, i);
        i
    };

    let port_key = |p: &str| format!("P:{p}");
    let pin_key = |inst: &str, pin: &str| format!("I:{inst}:{pin}");

    // ---- pass 1: nodes + nets -------------------------------------------
    let mut nets: HashMap<String, Net> = HashMap::new();
    let ensure_net = |nets: &mut HashMap<String, Net>, n: &str| {
        nets.entry(n.to_string()).or_insert(Net {
            driver: None,
            sinks: Vec::new(),
            load: [[0.0; 2]; 2],
        });
    };

    // primary input ports drive a net of the same name
    let mut input_ports: Vec<(usize, String)> = Vec::new();
    for p in &nl.inputs {
        let idx = node(
            port_key(p),
            p.clone(),
            &mut key2idx,
            &mut labels,
            &mut is_endpoint,
        );
        ensure_net(&mut nets, p);
        nets.get_mut(p).unwrap().driver = Some(idx);
        input_ports.push((idx, p.clone()));
    }
    // primary output ports are endpoints + sinks of their net
    let mut output_ports: Vec<(usize, String)> = Vec::new();
    for p in &nl.outputs {
        let idx = node(
            port_key(p),
            p.clone(),
            &mut key2idx,
            &mut labels,
            &mut is_endpoint,
        );
        is_endpoint[idx] = true;
        ensure_net(&mut nets, p);
        let net = nets.get_mut(p).unwrap();
        net.sinks.push(idx);
        for v in net.load.iter_mut().flatten() {
            *v += job.output_load;
        }
        output_ports.push((idx, p.clone()));
    }

    // instance pins. A sequential cell's data pins (with a setup constraint)
    // are *capture* endpoints; its Q launches via the CK->Q delay arc.
    // (D pin node, constraint table(s), this flop's CK pin key) — the constraint is
    // interpolated at the operating slews later; the CK key resolves to a node whose
    // arrival is the clock insertion delay to this flop (skew).
    // ⛔ A register's checks BORROW the library's (`checks_of`): the reference's check arcs are
    // the library cell's own, shared by every instance. A copy per instance held 827k
    // constraints, tables included, for 824 distinct ones on a 1.6M-instance design (2.2 GiB).
    let mut flop_d: Vec<(usize, Cow<'_, [Constraint]>, Option<String>)> = Vec::new();
    // `set_disable_timing`: the pins and arcs the constraints take out (see `crate::disable`).
    let mut disable_warnings = Vec::new();
    let disables = crate::disable::Disables::new(&job.disabled_timing, nl, lib, &mut disable_warnings);
    for w in &disable_warnings {
        vyges_events::emit(
            &vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, format!("disable timing: {w}"))
                .with_code("STA-DISABLE-TIMING"),
        );
    }
    // The checks of data pin `pin` of `inst` that a disable leaves: one is not made when its
    // clock pin is disabled (`checkEdgeEnabled` needs `searchFrom` of the check's from vertex) or
    // when the instance or cell disables the check arc itself (a `-from`/`-to` names checks too;
    // "all" does not).
    let enabled_check = |inst: &crate::netlist::Inst, pin: &str, c: &Constraint| -> bool {
        !disables.pin(Some(&inst.name), Some(&inst.cell), &c.related_pin)
            && !disables.arc(&inst.name, &inst.cell, &c.related_pin, pin, true)
    };
    // the recovery / removal pins among them: the reference reports their checks in their own
    // path group, `asynchronous`
    let mut async_end: std::collections::HashSet<usize> = std::collections::HashSet::new();
    // the latch data pins among the setup ends: checked as the reference's latch checks
    let mut latch_d: std::collections::HashSet<usize> = std::collections::HashSet::new();
    let mut flop_hold: Vec<(usize, Cow<'_, [Constraint]>, Option<String>)> = Vec::new();
    // Nodes that drive a fixed logic level (tie cells: `function : "1"` / `"0"`). A net
    // driven by one can never switch, so it carries no timing — see the edge build below.
    let mut const_driver: Vec<bool> = Vec::new();
    // Liberty input capacitance at each receiver pin, by node. SPEF carries the WIRE's
    // capacitance; the reference adds this during parasitic reduction unless the SPEF said
    // it already includes pin caps (`ReduceToPi::pinCapacitance`, guarded by
    // `includesPinCaps`). Without it every subtree capacitance in the Elmore walk is short.
    //
    // ⛔ PER EDGE. The reference loads a rising output with each receiver's rise capacitance and
    // a falling one with its fall capacitance (`rise_capacitance` / `fall_capacitance`, the
    // plain `capacitance` only where those are absent). One value per pin — the `capacitance`
    // attribute — was 4.8 % off either edge on average across sky130 hd's input pins, 10.2 %
    // at worst. Lane 0 rise, 1 fall, as everywhere in this timer. A pin with a CCS receiver
    // model keeps its Miller-aware load on both lanes.
    //
    // And per ANALYSIS: an early (hold) analysis loads with the minimum of each pin's capacitance
    // range, a late one with the maximum — icsprout55 states ranges, and its reference min path
    // carries 0.000634 pF where the max is 0.000843. `[analysis][edge]`, see `pin_load_caps`.
    let mut pin_cap_node: HashMap<usize, [[f64; 2]; 2]> = HashMap::new();
    // A pin of a cell from a later library whose thresholds differ from the first's: those
    // thresholds. Every other pin, and every port, is timed with `lib.thresholds` (the reference's
    // `thresholdLibrary`). Empty for a single library, so nothing below changes there.
    let mut node_th: HashMap<usize, crate::liberty::Thresholds> = HashMap::new();
    let mut ck_node_list: Vec<usize> = Vec::new(); // nodes that are clock (CK) pins
    // (CK node, lane of the edge that triggers it) — see the push site for why Liberty's
    // CK->Q arc is where that lives.
    let mut ck_lane_list: Vec<(usize, Option<usize>)> = Vec::new();
    for inst in &nl.insts {
        // physical-only cells (fill/decap/tap/antenna) have no connections and no
        // timing view — skip them rather than erroring on a missing lib cell.
        let cell = match lib.cell(&inst.cell) {
            Some(c) => c,
            // physical-only cells (fill/decap/tap/antenna diode) have no timing
            // view: they connect nothing, or only power/ground pins. Skip them
            // rather than erroring on a missing lib cell — matches OpenSTA's
            // tolerance of physical fill in a post-route netlist.
            None if inst.conns.is_empty()
                || inst.conns.iter().all(|(pin, _)| is_power_pin(pin)) =>
            {
                continue
            }
            None => return Err(StaError::UnknownCell(inst.cell.clone())),
        };
        let ck_key = cell.clock_pin.as_ref().map(|cp| pin_key(&inst.name, cp));
        for (pin, net) in &inst.conns {
            let idx = node(
                pin_key(&inst.name, pin),
                format!("{}/{}", inst.name, pin),
                &mut key2idx,
                &mut labels,
                &mut is_endpoint,
            );
            if let Some(t) = cell.thresholds {
                node_th.insert(idx, t);
            }
            ensure_net(&mut nets, net);
            match cell.pins.get(pin).map(|p| p.direction) {
                Some(Dir::Out) => {
                    nets.get_mut(net).unwrap().driver.get_or_insert(idx);
                    if cell.pins[pin].is_constant() {
                        if const_driver.len() <= idx {
                            const_driver.resize(idx + 1, false);
                        }
                        const_driver[idx] = true;
                    }
                }
                Some(Dir::In) => {
                    // Miller-aware receiver load when the pin carries a CCS receiver
                    // model, else the static `capacitance`.
                    let cap = pin_load_caps(&cell.pins[pin]);
                    pin_cap_node.insert(idx, cap);
                    let nref = nets.get_mut(net).unwrap();
                    nref.sinks.push(idx);
                    for (load, c) in nref.load.iter_mut().flatten().zip(cap.iter().flatten()) {
                        *load += c;
                    }
                    if cell.pins[pin].clock {
                        ck_node_list.push(idx); // clock pin — root of insertion-delay paths
                        // WHICH clock edge triggers this register. Liberty states it on the
                        // CK->Q arc (`timing_type : rising_edge` / `falling_edge`), which is
                        // the only place it is written down — `timing_sense` cannot supply
                        // it and sky130 omits that on these arcs anyway. Lane 0 is rise,
                        // lane 1 is fall.
                        let lane = cell
                            .pins
                            .values()
                            .flat_map(|p| p.arcs.iter())
                            .find(|a| a.related_pin == *pin && a.clk_edge.is_some())
                            .and_then(|a| a.clk_edge)
                            .map(|e| usize::from(e == ClkEdge::Fall));
                        ck_lane_list.push((idx, lane));
                    }
                    if cell.is_seq {
                        let setup = checks_of(&cell.pins[pin].setup, |c| enabled_check(inst, pin, c));
                        if !setup.is_empty() {
                            is_endpoint[idx] = true; // data pin = setup capture endpoint
                            flop_d.push((idx, setup, ck_key.clone()));
                            if is_latch_data_pin(cell, pin) {
                                latch_d.insert(idx);
                            }
                        }
                        let hold = checks_of(&cell.pins[pin].hold, |c| enabled_check(inst, pin, c));
                        if !hold.is_empty() {
                            flop_hold.push((idx, hold, ck_key.clone()));
                        }
                        // Asynchronous set/reset. Recovery is setup's counterpart (the
                        // release must arrive early enough before the capture edge) and
                        // removal is hold's (it must stay stable long enough after it) —
                        // same slack arithmetic, different table and different pin, so
                        // they join the same endpoint lists rather than duplicating it.
                        //
                        // Without this an async pin carries no check at all: it has no
                        // setup/hold groups, so nothing above ever names it. On a real
                        // block that is not a rounding error — sky130 `dfrtp` reset pins
                        // account for roughly a fifth of the design's hold checks.
                        let recovery = checks_of(&cell.pins[pin].recovery, |c| enabled_check(inst, pin, c));
                        if !recovery.is_empty() {
                            is_endpoint[idx] = true;
                            flop_d.push((idx, recovery, ck_key.clone()));
                            async_end.insert(idx);
                        }
                        let removal = checks_of(&cell.pins[pin].removal, |c| enabled_check(inst, pin, c));
                        if !removal.is_empty() {
                            flop_hold.push((idx, removal, ck_key.clone()));
                            async_end.insert(idx);
                        }
                    }
                }
                _ => {}
            }
        }
    }

    let n = labels.len();
    // endpoint required times are filled after the forward pass (they depend on the
    // clock arrival at each capture flop — see clock-skew handling below).
    let period = job.period_ns;
    let mut endpoint_req = vec![period; n];
    // SDC I/O budget + setup uncertainty. Output ports lose the external output
    // delay and the setup guard band from the period. Input ports (other than
    // clock sources) seed the forward pass with their external arrival delay.
    let clock_srcs: std::collections::HashSet<String> = if job.clocks.is_empty() {
        std::iter::once(job.clock_port.clone()).collect()
    } else {
        job.clocks.iter().map(|(_, s, _)| s.clone()).collect()
    };
    // `in_seed[edge][analysis][node]`: an input port's arrival per data edge, for the input delays
    // relative to a clock's rising (edge 0) or falling (edge 1) edge, in the min (0) and max (1)
    // analysis — measured from that clock edge, as every launch here is (the edge's own time is
    // in the launch-capture relation). Non-finite: no arrival on that lane. `in_clock[node][edge]`:
    // the clock those delays are relative to (`None`: the primary clock stands in).
    let unset = [f64::NEG_INFINITY; 2];
    let mut in_seed: [[Vec<[f64; 2]>; 2]; 2] = [[vec![[0.0; 2]; n], vec![[0.0; 2]; n]], [vec![unset; n], vec![unset; n]]];
    // Held for the input ports that carry a delay only, by node: one pair per pin was 0.26 GiB on
    // a 1.6M-instance design, nearly all of it `None`. Looked up, never iterated.
    let mut in_clock: HashMap<usize, [Option<String>; 2]> = HashMap::new();
    let mut in_fall = false;
    let mut multi_clock_in: Vec<&str> = Vec::new();
    // A port with no declared arrival is UNCONSTRAINED: it is not a timing start point, so
    // it is left unreached exactly as a tie cell is (see the indeg-0 loop below). Seeding it
    // at 0 instead invents a launch at time zero — harmless on setup, where a whole period
    // absorbs it, and on hold it manufactures a violation the size of the capture clock's
    // insertion delay plus uncertainty. Measured against OpenSTA on a block whose SDC
    // constrained no inputs: -1.07 ns here against +0.88 there, and OpenSTA answers
    // "No paths found" for the endpoint this engine called worst.
    let mut unconstrained_in = vec![false; n];
    let mut unconstrained_names: Vec<&str> = Vec::new();
    for (idx, name) in &input_ports {
        if !clock_srcs.contains(name) {
            let delays = job.input_delays_for(name);
            if delays.is_empty() {
                unconstrained_in[*idx] = true;
                unconstrained_names.push(name.as_str());
                continue;
            }
            for per_edge in in_seed.iter_mut() {
                per_edge[0][*idx] = unset;
                per_edge[1][*idx] = unset;
            }
            // `seedInputDelayArrival`: one arrival per delay, its value on each edge it sets, the
            // latest kept for max and the earliest for min
            for d in &delays {
                let e = usize::from(d.clock_fall);
                in_fall |= e == 1;
                let ic = in_clock.entry(*idx).or_default();
                match (&ic[e], &d.clock) {
                    (None, c) => ic[e] = c.clone(),
                    (Some(a), Some(b)) if a != b => multi_clock_in.push(name.as_str()),
                    _ => {}
                }
                for (mm, row) in d.value.iter().enumerate() {
                    for (rf, v) in row.iter().enumerate() {
                        let Some(v) = *v else { continue };
                        let slot = &mut in_seed[e][mm][*idx][rf];
                        *slot = if !slot.is_finite() { v } else if mm == 1 { slot.max(v) } else { slot.min(v) };
                    }
                }
            }
        }
    }
    // Say what was skipped. A path that silently does not exist is indistinguishable from a
    // path that passes, and the whole point of leaving these unreached is that they no longer
    // appear in WNS/WHS -- so the count has to be visible or the model just got quieter.
    if !unconstrained_names.is_empty() {
        let mut shown: Vec<&str> = unconstrained_names.clone();
        shown.sort_unstable();
        shown.dedup();
        let n_shown = shown.len();
        let head: Vec<&str> = shown.into_iter().take(5).collect();
        vyges_events::emit(
            &vyges_events::Event::new(
                "vyges-sta-si",
                vyges_events::Severity::Warn,
                format!(
                    "{n_shown} input port(s) have no declared arrival and start no timed path ({}{}). Add set_input_delay to time them.",
                    head.join(", "),
                    if n_shown > head.len() { ", …" } else { "" }
                ),
            )
            .with_code("STA-UNCONSTRAINED-INPUT")
            .with_objects(head.iter().map(|p| format!("port:{p}")).collect()),
        );
    }

    // One arrival per node carries one launching clock: a port with delays relative to two
    // clocks on one edge is timed at its latest (earliest) arrival against the first clock.
    if !multi_clock_in.is_empty() {
        multi_clock_in.sort_unstable();
        multi_clock_in.dedup();
        let head: Vec<&str> = multi_clock_in.iter().take(5).copied().collect();
        vyges_events::emit(
            &vyges_events::Event::new(
                "vyges-sta-si",
                vyges_events::Severity::Warn,
                format!(
                    "{} input port(s) carry input delays relative to more than one clock on one clock edge ({}{}); each is timed at its worst arrival against the first clock, not once per clock.",
                    multi_clock_in.len(),
                    head.join(", "),
                    if multi_clock_in.len() > head.len() { ", …" } else { "" }
                ),
            )
            .with_code("STA-INPUT-MULTICLOCK")
            .with_objects(head.iter().map(|p| format!("port:{p}")).collect()),
        );
    }

    // Path-end checks the reference makes and this timer does not (non-sequential setup/hold,
    // skew), on cells the design uses: SAID, since a design checked only by them reports nothing
    // at all otherwise.
    let mut unmodelled: Vec<String> = nl
        .insts
        .iter()
        .filter_map(|i| lib.cell(&i.cell))
        // pulse width and period are `report_check_types` checks, not path ends — not these
        // and `retaining_*`: the reference times its min analysis with them, this timer with the arc
        .flat_map(|c| c.pins.values().flat_map(move |p| p.unmodelled_checks.iter().filter(|k| k.starts_with("non_seq") || k.starts_with("skew") || k.starts_with("retaining")).map(move |k| format!("{}/{} {k}", c.name, p.name))))
        .collect();
    unmodelled.sort_unstable();
    unmodelled.dedup();
    if !unmodelled.is_empty() {
        let head: Vec<&str> = unmodelled.iter().take(5).map(String::as_str).collect();
        vyges_events::emit(
            &vyges_events::Event::new(
                "vyges-sta-si",
                vyges_events::Severity::Warn,
                format!(
                    "{} library check(s) or min-analysis table(s) on cells in the design are not modelled ({}{})",
                    unmodelled.len(),
                    head.join(", "),
                    if unmodelled.len() > head.len() { ", …" } else { "" }
                ),
            )
            .with_code("STA-UNMODELLED-CHECK"),
        );
    }

    // The uncertainty a check takes: its TARGET clock's own (`PathEnd::checkTgtClkUncertainty`:
    // the capture clock path's, else the target clock edge's clock's), so an output's is its
    // delay's clock's, not the launching register's; 0 for a clock with none. Without per-clock
    // values (see `StaJob::clock_uncertainty`), the design-wide one.
    let primary_name = job.clocks.first().map_or(job.clock_port.as_str(), |c| c.0.as_str());
    let unc = |clk: Option<&str>, setup: bool| -> f64 {
        if job.clock_uncertainty.is_empty() {
            return if setup { job.setup_uncertainty } else { job.hold_uncertainty };
        }
        let c = clk.unwrap_or(primary_name);
        job.clock_uncertainty.iter().find(|u| u.0 == c).map_or(0.0, |u| if setup { u.1 } else { u.2 })
    };
    for (idx, name) in &output_ports {
        let clk = job.output_delays_for(name).first().and_then(|d| d.clock.clone());
        endpoint_req[*idx] = period - job.output_delay_for(name) - unc(clk.as_deref(), true);
    }
    let mut node_load = vec![[[0.0f64; 2]; 2]; n];
    for (netname, net) in &nets {
        if let Some(d) = net.driver {
            // driver load = receiver pin caps + wire cap from SPEF (fF -> the library's unit),
            // per output edge
            let wire = spef.and_then(|s| s.net(netname)).map_or(0.0, |rc| rc.cap_ff / ff_per_cap);
            node_load[d] = net.load.map(|e| e.map(|c| c + wire));
        }
    }

    // ---- constants: case analysis, logic values, ties (the reference's `Sim`) ----------
    // A pin held at 0 or 1 starts nothing and is reached by nothing; an arc whose output no
    // longer depends on its input is cut; a pin held rising / falling passes one edge.
    let sim = crate::sim::run(nl, lib, &job.case_values);
    for w in &sim.warnings {
        vyges_events::emit(
            &vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, format!("case analysis: {w}"))
                .with_code("STA-CASE-ANALYSIS"),
        );
    }
    let held: Vec<bool> = labels.iter().map(|l| sim.is_constant(l)).collect();
    let pass_lane = |node: usize, lane: usize| sim.passes(&labels[node], lane);
    // nodes whose every in-arc the constants cut: untimed, as a tie output is
    let mut sim_cut = vec![0usize; n];
    // `set_disable_timing`: nodes that are disabled pins (nothing leaves them — `SearchPred0` and
    // `DcalcPred` stop at `isDisabledConstraint(from_pin)`; arcs INTO them still arrive), and the
    // count of each node's in-arcs a disable removed. A disabled arc is neither searched nor
    // calculated: it is not built at all, so it sets no slew either.
    let mut dis_pin = vec![false; n];
    if !disables.is_empty() {
        for p in nl.inputs.iter().chain(&nl.outputs).chain(&nl.inouts) {
            if let Some(&v) = key2idx.get(&port_key(p)) {
                dis_pin[v] = disables.pin(None, None, p);
            }
        }
        for inst in &nl.insts {
            for (pin, _) in &inst.conns {
                if let Some(&v) = key2idx.get(&pin_key(&inst.name, pin)) {
                    dis_pin[v] = disables.pin(Some(&inst.name), Some(&inst.cell), pin);
                }
            }
        }
    }
    let mut dis_cut = vec![0usize; n];

    // ---- pass 2: edges ---------------------------------------------------
    let mut out_edges: Vec<Vec<Edge>> = (0..n).map(|_| Vec::new()).collect();
    let mut indeg = vec![0usize; n];

    // cell arcs: related input pin -> output pin (within an instance)
    for inst in &nl.insts {
        let Some(cell) = lib.cell(&inst.cell) else {
            continue; // physical-only cell skipped in pass 1
        };
        let conn: HashMap<&str, &str> = inst
            .conns
            .iter()
            .map(|(p, net)| (p.as_str(), net.as_str()))
            .collect();
        for (opin, pininfo) in &cell.pins {
            if pininfo.direction != Dir::Out || !conn.contains_key(opin.as_str()) {
                continue;
            }
            let o_idx = key2idx[&pin_key(&inst.name, opin)];
            for arc in &pininfo.arcs {
                if !conn.contains_key(arc.related_pin.as_str()) {
                    continue;
                }
                let i_idx = key2idx[&pin_key(&inst.name, &arc.related_pin)];
                let sense = sim.sense.get(&(inst.name.clone(), arc.related_pin.clone(), opin.clone()));
                // an arc FROM a held pin is gone, delay calculation included; one INTO a held
                // pin carries nothing; one whose output no longer follows its input (a
                // deselected mux input) carries no arrival but still sets the output's slews
                if held[i_idx] || held[o_idx] {
                    sim_cut[o_idx] += 1;
                    continue;
                }
                if dis_pin[i_idx] || disables.arc(&inst.name, &inst.cell, &arc.related_pin, opin, false) {
                    dis_cut[o_idx] += 1;
                    continue;
                }
                let mut lanes = ALL_LANES;
                for (il, row) in lanes.iter_mut().enumerate() {
                    for (ol, ok) in row.iter_mut().enumerate() {
                        *ok = sense.is_none_or(|s| s.pairs(il, ol)) && pass_lane(i_idx, il) && pass_lane(o_idx, ol);
                    }
                }
                let latch_dq = is_latch_d_to_q(cell, arc);
                out_edges[i_idx].push(Edge {
                    to: o_idx,
                    kind: EdgeKind::Cell(arc),
                    lanes,
                    latch_dq,
                });
                // the order is the reference's levels: not through a latch D->Q arc
                if !latch_dq {
                    indeg[o_idx] += 1;
                }
            }
        }
    }
    // index the nets so net arcs can look up a (mutable, per-pass) delay table
    // the names borrow the map's own keys (it is not changed after this); a copy was 0.13 GiB on a
    // 1.7M-net design
    let net_order: Vec<&str> = nets.keys().map(String::as_str).collect();
    let net_idx: HashMap<&str, usize> = net_order.iter().enumerate().map(|(i, nm)| (*nm, i)).collect();
    let nn = net_order.len();
    let mut net_res = vec![0.0f64; nn];
    let mut net_cap = vec![0.0f64; nn];
    let mut net_drv: Vec<Option<usize>> = vec![None; nn];
    // Whether each net's driver is an instance pin (not a port): all the timer asks of its name.
    // ⚠️ It held the (inst, pin) Strings, 0.2 GiB on a 1.7M-net design, for that one bit.
    let mut net_drv_pin: Vec<bool> = vec![false; nn];
    let mut net_cpl: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nn]; // (aggressor net idx, Cc)
    let is_inst_pin = |node: usize| split_inst_pin(&labels[node]).is_some();
    for (name, net) in &nets {
        let i = net_idx[name.as_str()];
        net_drv[i] = net.driver;
        net_drv_pin[i] = net.driver.is_some_and(is_inst_pin);
        if let Some((s, rc)) = spef.and_then(|s| s.net(name).map(|rc| (s, rc))) {
            net_res[i] = rc.res_ohm;
            net_cap[i] = rc.cap_ff;
            for &(agg, cc) in &rc.coupling {
                if let Some(&ai) = net_idx.get(s.name(agg)) {
                    net_cpl[i].push((ai, cc));
                }
            }
        }
    }
    // Each net's parasitic, and each of its pins' SPEF node, found once by name (`pin_node`'s
    // answer: the net's first `*CONN` entry for the pin). Looked up per load and per pass by
    // name, a net cost its fanout squared in string compares each pass.
    let net_rc: Vec<Option<&CompactNet>> = net_order.iter().map(|nm| spef.and_then(|s| s.net(nm))).collect();
    // Nodes are the SPEF's interned ids. A top-level port is its own node, named by the port
    // (`node_of_port`); a name the net does not hold is `NO_NAME`, which no walk finds — as an
    // unknown node NAME found nothing when nodes were strings.
    let mut node_of_pin: Vec<Option<Name>> = vec![None; labels.len()];
    let mut node_of_port: Vec<Name> = vec![NO_NAME; labels.len()];
    for (name, net) in &nets {
        let Some(rc) = net_rc[net_idx[name.as_str()]] else { continue };
        let s = spef.expect("a net's parasitic comes from the SPEF");
        // a scan for a small net; an index for a large one, where the scans add up to its
        // fanout squared
        let by: Option<HashMap<(&str, &str), Name>> = (rc.pins.len() > 32).then(|| {
            let mut by = HashMap::with_capacity(rc.pins.len());
            for &(pi, pp, nd) in &rc.pins {
                by.entry((s.name(pi), s.name(pp))).or_insert(nd);
            }
            by
        });
        for v in net.driver.into_iter().chain(net.sinks.iter().copied()) {
            if let Some((a, b)) = split_inst_pin(&labels[v]) {
                node_of_pin[v] = match &by {
                    Some(by) => by.get(&(a, b)).copied(),
                    None => s.pin_node(rc, a, b),
                };
            } else {
                node_of_port[v] = s.node(rc, &labels[v]).unwrap_or(NO_NAME);
            }
        }
    }
    let mut is_out_port = vec![false; labels.len()];
    for (o, _) in &output_ports {
        is_out_port[*o] = true;
    }

    // effective-capacitance shielding per driver node (CCS-into-RC): the driver
    // sees a near cap + a far cap shielded behind the wire resistance, so its cell
    // delay is computed at Ceff < total on resistive nets.
    // driver node -> the three-element Pi (C2 near pF, Rpi, C1 far pF), as
    // `ReduceToPi::reduceToPi` builds it by matching admittance moments. ⛔ It is NOT the
    // driver node's own capacitance: moment matching puts 0.1879 pF of near cap on
    // `fft_top`'s `input55`, where that SPEF node carries none at all — so reading the node
    // handed the gate ZERO near capacitance and `report_dcalc` disagreed by the whole Pi.
    // Per output edge: the receiver caps it sums are that edge's.
    // `[analysis][edge]`, each a (C2 near pF, Rpi, C1 far pF) Pi.
    type Pi = Option<(f64, f64, f64)>;
    let mut shield: Vec<[[Pi; 2]; 2]> = vec![[[None; 2]; 2]; n];
    if let Some(s) = spef {
        for (name, net) in &nets {
            if let Some(d) = net.driver {
                let i = net_idx[name.as_str()];
                if let (Some(rc), true) = (s.net(name), net_drv_pin[i]) {
                    if let Some(dnode) = node_of_pin[d] {
                        // the same capacitance content the reduction sees: coupling
                        // grounded at factor 1.0, plus Liberty receiver caps at the sinks
                        let cc: f64 = net_cpl[i].iter().map(|(_, c)| *c).sum();
                        for (mm, lane) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                        let mut pcap: HashMap<Name, f64> = HashMap::new();
                        for &sk in &net.sinks {
                            if split_inst_pin(&labels[sk]).is_some() {
                                if let Some(nd) = node_of_pin[sk] {
                                    *pcap.entry(nd).or_default() +=
                                        pin_cap_node.get(&sk).map_or(0.0, |c| c[mm][lane]) * ff_per_cap;
                                }
                            } else if is_out_port[sk] {
                                // A top-level output port: its external load at its own node
                                // (`ReduceToPi::pinCapacitance` → `Sdc::portExtCap`). Without
                                // it the Pi of a net driving a port carries only the wire, and
                                // the gate sees almost no load (fft_ctrl_tlul's sram_clk_o:
                                // 0.0706 ns against sign-off's 0.2125 ns at output140).
                                *pcap.entry(node_of_port[sk]).or_default() += job.output_load * ff_per_cap;
                            }
                        }
                        if let Some((c2_ff, rpi, c1_ff)) = rc.pi_reduce(dnode, cc, |k| pcap.get(&k).copied()) {
                            shield[d][mm][lane] = Some((c2_ff / ff_per_cap, rpi, c1_ff / ff_per_cap));
                        }
                        }
                    }
                }
            }
        }
    }

    // ---- input ports with a driving cell (`GraphDelayCalc::findInputDriverDelay`) -------
    // Per analysis and port edge, each arc of the cell from its `from` pin into `pin` ending in
    // that edge, at the transition given for its from-edge (`findInputArcDelay`): the gate at the
    // port net's load — DMP on the net's Pi when it has one, else lumped — and at no load (the
    // intrinsic delay). The port's slew IS the gate's; each load's wire delay and slew come from
    // the gate's waveform (`loadDelaySlew`, then `thresholdAdjust` from the cell's library), and
    // every wire edge from the port also carries `gate - intrinsic` (`annotateLoadDelays`). A
    // later arc for the same edge overwrites. ⚠️ Without `-from_pin` the reference takes the first
    // of the cell's pins (declaration order) with an arc to `pin`; this takes the first timing
    // group's — the same pin for a one-input cell.
    struct PortDrive {
        slew: [[f64; 2]; 2],
        extra: [[f64; 2]; 2],
        wave: [[Option<crate::dmp::DriverWaveform>; 2]; 2],
        th: crate::liberty::Thresholds,
    }
    // Input ports a `set_input_transition` names: their slews, `[analysis][edge]`.
    let port_slew: HashMap<usize, [[f64; 2]; 2]> = if job.input_slews.is_empty() {
        HashMap::new()
    } else {
        let clock_srcs: std::collections::HashSet<&str> = job.clocks.iter().map(|(_, s, _)| s.as_str()).collect();
        input_ports.iter().map(|(i, name)| (*i, job.input_slew_for(name, clock_srcs.contains(name.as_str()) || *name == job.clock_port))).collect()
    };
    let mut port_drive: HashMap<usize, PortDrive> = HashMap::new();
    for (idx, name) in input_ports.iter().filter(|_| !job.driving_cells.is_empty()) {
        let Some(spec) = job.driving_cells.iter().rev().find(|(o, _)| o == "*INPUTS*" || o == name).map(|(_, s)| s) else { continue };
        let Some(cell) = lib.cell(&spec.cell) else {
            let msg = format!("set_driving_cell: cell {} not in any library; {name} keeps its input slew", spec.cell);
            vyges_events::emit(&vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, msg).with_code("STA-DRIVING-CELL"));
            continue;
        };
        let to = spec.pin.clone().or_else(|| cell.pins.iter().find(|(_, p)| p.direction == Dir::Out).map(|(n, _)| n.clone()));
        let Some(tp) = to.as_ref().and_then(|t| cell.pins.get(t)) else { continue };
        let from = spec.from_pin.clone().or_else(|| tp.arcs.first().map(|a| a.related_pin.clone()));
        let arcs: Vec<&crate::liberty::Arc> = tp.arcs.iter().filter(|a| Some(&a.related_pin) == from.as_ref() && a.clk_edge.is_none()).collect();
        let th = lib.cell_thresholds(&spec.cell);
        let Some(&i) = net_idx.get(name.as_str()) else { continue };
        let Some(net) = nets.get(name) else { continue };
        let mut pd = PortDrive { slew: [[job.input_slew; 2]; 2], extra: [[0.0; 2]; 2], wave: [[None; 2]; 2], th };
        for mm in 0..2 {
            for ol in 0..2 {
                // the port net's Pi, with the same capacitance content as a gate driver's
                let pi = spef.and_then(|s| s.net(name).map(|rc| (s, rc))).and_then(|(s, rc)| {
                    let cc: f64 = net_cpl[i].iter().map(|(_, c)| *c).sum();
                    let mut pcap: HashMap<Name, f64> = HashMap::new();
                    for &sk in &net.sinks {
                        if split_inst_pin(&labels[sk]).is_some() {
                            if let Some(nd) = node_of_pin[sk] {
                                *pcap.entry(nd).or_default() += pin_cap_node.get(&sk).map_or(0.0, |c| c[mm][ol]) * ff_per_cap;
                            }
                        } else if is_out_port[sk] {
                            *pcap.entry(node_of_port[sk]).or_default() += job.output_load * ff_per_cap;
                        }
                    }
                    // the port net's driver is the node named by the net
                    let dn = s.node(rc, name).unwrap_or(NO_NAME);
                    rc.pi_reduce(dn, cc, |k| pcap.get(&k).copied()).map(|(c2, rpi, c1)| (c2 / ff_per_cap, rpi, c1 / ff_per_cap))
                });
                for arc in arcs.iter().filter(|a| a.out_edges.drives(ol == 0)) {
                    let (dt, st) = if ol == 0 { (&arc.cell_rise, &arc.rise_transition) } else { (&arc.cell_fall, &arc.fall_transition) };
                    for (il, &sin) in spec.input_transition.iter().enumerate() {
                        let feeds = arc.feeds(il == 0, ol == 0); // the reference's edge pairs (latch D->Q, tri-state by kind)
                        if !feeds {
                            continue;
                        }
                        let intrinsic = dt.lookup(sin, 0.0);
                        let vth = crate::dmp::Vth {
                            vth: if ol == 0 { th.output_rise } else { th.output_fall },
                            vl: if ol == 0 { th.slew_lower_rise } else { th.slew_lower_fall },
                            vh: if ol == 0 { th.slew_upper_rise } else { th.slew_upper_fall },
                            slew_derate: th.slew_derate,
                        };
                        let (gate, slew, wave) = match pi {
                            Some((c2, rpi, c1)) => {
                                let g = |c: f64| (dt.lookup(sin, c), st.lookup(sin, c));
                                let rd = crate::dmp::gate_model_rd_step(vth.vth, &g, c1, c2, cap_step);
                                let r = crate::dmp::solve_driver_r(rd, c2, rpi / ohm_per_r, c1, vth, &g, ohm_per_r);
                                (r.delay, r.slew, r.waveform)
                            }
                            None => (dt.lookup(sin, net.load[mm][ol]), st.lookup(sin, net.load[mm][ol]), None),
                        };
                        if dmp_trace {
                            eprintln!("drive|{name}|mm={mm}|ol={ol}|il={il}|in_slew={sin}|pi={pi:?}|gate={gate}|intrinsic={intrinsic}|slew={slew}");
                        }
                        pd.slew[mm][ol] = slew;
                        pd.extra[mm][ol] = f64::from(gate as f32 - intrinsic as f32);
                        pd.wave[mm][ol] = wave;
                    }
                }
            }
        }
        port_drive.insert(*idx, pd);
    }

    // net arcs: driver -> each sink. Each arc gets its own delay (per-pin Elmore),
    // indexed by an arc id. Record the sink's (inst,pin) to resolve its SPEF node.
    struct ArcInfo {
        net_idx: usize,
        sink: usize,
        /// The sink is an instance pin (not a port). ⚠️ It held the (inst, pin) Strings, 0.45 GiB
        /// across 4M arcs, for this one bit.
        sink_pin: bool,
    }
    let mut arcs: Vec<ArcInfo> = Vec::new();
    for (name, net) in &nets {
        if let Some(d) = net.driver {
            let i = net_idx[name.as_str()];
            for &s in &net.sinks {
                if s != d {
                    let aid = arcs.len();
                    arcs.push(ArcInfo {
                        net_idx: i,
                        sink: s,
                        sink_pin: is_inst_pin(s),
                    });
                    if held[d] || held[s] {
                        sim_cut[s] += 1;
                        continue;
                    }
                    if dis_pin[d] {
                        dis_cut[s] += 1;
                        continue;
                    }
                    let mut lanes = ALL_LANES;
                    for (l, row) in lanes.iter_mut().enumerate() {
                        row[l] = pass_lane(d, l) && pass_lane(s, l);
                    }
                    out_edges[d].push(Edge {
                        to: s,
                        kind: EdgeKind::Net(aid),
                        lanes,
                        latch_dq: false,
                    });
                    indeg[s] += 1;
                }
            }
        }
    }
    let n_arcs = arcs.len();
    // A node the constants hold, or whose every in-arc they cut, is untimed exactly as a tie
    // cell's output is: no seed, so nothing reaches its fanout.
    if const_driver.len() < n {
        const_driver.resize(n, false);
    }
    for v in 0..n {
        // A node whose every in-arc a disable removed is untimed the same way.
        if held[v] || (indeg[v] == 0 && (sim_cut[v] > 0 || dis_cut[v] > 0)) {
            const_driver[v] = true;
        }
    }
    // Its slews follow the reference's delay calculation, a walk from the graph's ROOTS
    // (`seedRootSlews`, then each driver's fanout) through the arcs `DcalcPred` allows. A root is a
    // pin with no arc into it at all — `Levelize::isRoot` does not consult the SDC, so a pin a
    // disable cut off is NOT one. So a pin the walk never reaches keeps the graph's initial slew,
    // 0, and so does all it alone drives; a driver the walk DOES reach (through another input)
    // calculates every enabled arc into it, one from an unreached pin from slew 0. Measured on
    // gcd: a flop with its arcs disabled leaves its Q and every load at 0 and a load gate's output
    // slew is still set by the arc from it; a disabled clock buffer leaves a whole leaf's flops,
    // their Q pins and an inverter fed only by one of them at 0.
    let dis_src: Vec<bool> = (0..n).map(|v| !held[v] && indeg[v] == 0 && sim_cut[v] == 0 && dis_cut[v] > 0).collect();
    let sim_narrows = out_edges.iter().flatten().any(|e| e.lanes != ALL_LANES);

    // ---- on-chip variation model ----------------------------------------
    // Refines the flat late/early scalar derate. POCV (statistical) wins when a
    // per-stage sigma is given: each cell stage adds variance sigma^2 and the path
    // delay carries an N-sigma band that grows as sqrt(depth) (RSS), not linearly.
    // Else AOCV: a depth-dependent derate table (deeper paths derate toward 1.0).
    // Else flat.
    let late_derate = job.late_derate;
    let early_derate = job.early_derate;
    // LVF present? -> per-arc slew/load-dependent delay sigma is available, which
    // auto-enables (the more accurate) POCV even without a global pocv_sigma.
    let has_lvf = lib.cells.values().any(|c| {
        c.pins.values().any(|p| {
            p.arcs.iter().any(|a| {
                a.lvf_cell
                    .iter()
                    .any(|l| !l.delay_sigma(true).values.is_empty() || !l.delay_sigma(false).values.is_empty())
            })
        })
    });
    let pocv = job.pocv_sigma > 0.0 || has_lvf;
    let aocv = !pocv && (!job.aocv_late.is_empty() || !job.aocv_early.is_empty());
    // The incremental fast path ([`Timer::update`]) is eligible only in the simple timing
    // context: ideal interconnect (no SPEF/SI), flat derate (no AOCV/POCV bands), and no
    // path-based pass. Outside it we don't capture the graph and `update` does a full pass.
    // A register launched on a clock's FALLING edge (a negative-edge flop; a memory macro whose
    // read data is clocked out on the fall): arrivals are then timed per launching edge (see
    // `EdgeRun`), which the incremental fast path does not carry.
    // An input delay relative to a falling clock edge is a launch on the fall as well.
    let fall_launch = ck_lane_list.iter().any(|&(_, l)| l == Some(1)) || in_fall;
    // The fast path keeps ONE clock-side base per endpoint, so one capture edge per pin.
    let mixed_capture =
        flop_d.iter().chain(&flop_hold).any(|(_, cons, _)| cons.windows(2).any(|w| w[0].clk_edge != w[1].clk_edge));
    // a data check's required time follows another pin's arrival, which the fast path does not
    // re-derive
    // A latch D->Q arc is outside the order, so the incremental cone walk would not see it.
    let has_latch_dq = out_edges.iter().flatten().any(|e| e.latch_dq);
    let simple_ctx = capture && spef.is_none() && !pocv && !aocv && !job.pba && !fall_launch && !mixed_capture && !sim_narrows
        && job.data_check_cmds.is_empty() && !has_latch_dq;
    let mut inc_setup: Vec<SetupRec> = Vec::new();
    let mut inc_hold: Vec<HoldRec> = Vec::new();
    let pocv_sigma = job.pocv_sigma;
    let n_sigma = job.pocv_n;
    // per-cell-stage derate on the nominal delay (1.0 for POCV — it uses sigma)
    let cell_derate = |late: bool, stage: usize| -> f64 {
        if aocv {
            aocv_lookup(
                if late {
                    &job.aocv_late
                } else {
                    &job.aocv_early
                },
                stage as f64,
            )
        } else if pocv {
            1.0
        } else if late {
            late_derate
        } else {
            early_derate
        }
    };

    // Which nodes are flop clock (CK) pins. Needed by the clock-launched pass below
    // (which stops at them) as well as by CRPR further down.
    // The effective capacitance actually used at each driver node, so it can be diffed
    // against `report_dcalc`'s `Ceff=..` rather than re-derived. Diagnostic only.
    // ⚠️ PER OUTPUT LANE (0 = rise). Recording it per NODE reports whichever lane ran
    // last, and `report_dcalc` prints a block per EDGE — diffing the two invites an
    // apples-to-oranges comparison, which cost a round here.
    let ceff_used = std::cell::RefCell::new(vec![[f64::NAN; 2]; n]);
    // The driver waveform each cell arc produced, per driver node and output lane.
    //
    // 🔑 CALL SEQUENCE. Upstream evaluates the gate and then every sink of that net inside
    // ONE arc evaluation, so `loadDelaySlew` reads what `gateDelaySlew` just set. Our net
    // delays are computed in `compute` and the gate in `relax`, so the state is carried
    // across — and the interconnect loop already alternates the two, which is the fixed
    // point that makes it converge. The FIRST `compute` has no waveform and falls back to
    // the closed form, exactly as before.
    //
    // ⛔ PER ANALYSIS AND PER ARC. Every arc into a driver evaluates the gate and its loads, and
    // arcs with the same output edge MERGE their wire delays and load slews to the worse one for
    // that analysis (`GraphDelayCalc::annotateLoadDelays`, `delayGreater(.., min_max)`: larger
    // late, smaller early, delay and slew compared separately). One waveform per driver edge
    // let the last arc evaluated — in either analysis — decide every wire from that driver.
    let drvr_wave: std::cell::RefCell<[Vec<[Vec<crate::dmp::DriverWaveform>; 2]>; 2]> =
        std::cell::RefCell::new([vec![[Vec::new(), Vec::new()]; n], vec![[Vec::new(), Vec::new()]; n]]);
    let mut is_ck = vec![false; n];
    for &i in &ck_node_list {
        is_ck[i] = true;
    }
    // A clock pin has exactly ONE arrival: the time its triggering edge gets there. Early
    // and late may differ only because something DERATES them, never because they picked a
    // different edge. `None` leaves the old extremum behaviour for a clock pin whose cell
    // declares no sequential arc — a gate we cannot attribute an edge to, where guessing
    // one would be worse than the pessimism.
    let mut ck_lane = vec![None::<usize>; n];
    for &(i, l) in &ck_lane_list {
        ck_lane[i] = l;
    }

    // ---- per-lane launch times -------------------------------------------
    // The main pass measures every delay *from the launching clock edge*, so both
    // lanes of a source start at the same time and a flop's Q arrival is already
    // relative to edge 0. A **clock source** is different: it is not a source of
    // relative delay but of the edges themselves, and its rising and falling edges
    // happen at different absolute times. `seed_clk` starts a second pass at those
    // times so a path that runs from a clock port out through the clock network
    // carries the launch edge it actually left on. See `clk_arr` below.
    // The merged passes (every launch from its own edge): per lane, the latest of the two edges'
    // max arrivals and the earliest of their min arrivals.
    let merge_edges = |mm: usize| -> Vec<[f64; 2]> {
        (0..n)
            .map(|i| {
                let (a, b) = (in_seed[0][mm][i], in_seed[1][mm][i]);
                std::array::from_fn(|l| match (a[l].is_finite(), b[l].is_finite()) {
                    (true, true) if mm == 1 => a[l].max(b[l]),
                    (true, true) => a[l].min(b[l]),
                    (true, false) => a[l],
                    (false, true) => b[l],
                    (false, false) => f64::NEG_INFINITY,
                })
            })
            .collect()
    };
    let seed_main: Vec<[f64; 2]> = merge_edges(1);
    let seed_early: Vec<[f64; 2]> = merge_edges(0);
    let period_of_src = |name: &str| -> f64 {
        job.clocks
            .iter()
            .find(|(_, s, _)| s == name)
            .map(|(_, _, p)| *p)
            .unwrap_or(period)
    };
    let mut seed_clk = vec![[f64::NEG_INFINITY; 2]; n]; // unreached unless a clock source
    let mut have_clock_launch = false;
    for (idx, name) in &input_ports {
        if clock_srcs.contains(name) {
            // the clock's own edges ([`clock_edge_time`]): by default the rise at 0, the fall
            // half a period later
            let clk = job.clocks.iter().find(|(_, s, _)| s == name).map(|(c, _, _)| c.as_str());
            let p = period_of_src(name);
            seed_clk[*idx] = [clock_edge_time(job, clk, 0, p), clock_edge_time(job, clk, 1, p)];
            have_clock_launch = true;
        }
    }

    // ---- forward propagation, reusable over a net-delay table ------------
    // Rise and fall propagate as **separate lanes** (0=rise, 1=fall): a cell arc
    // maps input→output edges by its unateness (negative_unate inverts: out-rise
    // comes from in-fall; positive_unate keeps; non_unate takes the worst of both),
    // and uses the matching cell_rise/cell_fall + transition table. This avoids the
    // pessimism of `max(rise,fall)` at every stage — an inverter chain alternates
    // edges, so the true path is sum-of-alternating, not sum-of-max. `late=true`
    // keeps the max edge, `late=false` the min. Per lane we also carry nominal
    // arrival, cell-stage depth, and variance (AOCV/POCV). On return we collapse
    // each node to its worst lane so every downstream consumer is unchanged.
    let input_slew = job.input_slew;
    // The thresholds a pin is timed with (see `node_th`).
    let th_of = |v: usize| node_th.get(&v).copied().unwrap_or(lib.thresholds);
    // The DRIVER's library thresholds per output edge (lane 0 = rise), as `DmpAlg::init` reads
    // them (`drvr_library->outputThreshold(rf)` ...).
    let dmp_vth = |v: usize, ol: usize| {
        let th = th_of(v);
        crate::dmp::Vth {
            vth: if ol == 0 { th.output_rise } else { th.output_fall },
            vl: if ol == 0 { th.slew_lower_rise } else { th.slew_lower_fall },
            vh: if ol == 0 { th.slew_upper_rise } else { th.slew_upper_fall },
            slew_derate: th.slew_derate,
        }
    };
    // `nd` = per-arc net delay, `ns` = per-arc degraded sink slew (0 = keep driver slew).
    // `seed_l` gives each source node its per-lane start time; `block_seq` stops
    // propagation at a flop's clock pin, which is what keeps the clock-launched pass
    // (below) free of paths a flop launched.
    let eff_clocks: Vec<(String, f64)> = if job.clocks.is_empty() {
        vec![(job.clock_port.clone(), period)]
    } else {
        job.clocks
            .iter()
            .map(|(_, src, per)| (src.clone(), *per))
            .collect()
    };
    let mut clock_src: HashMap<usize, f64> = HashMap::new();
    for (src, per) in &eff_clocks {
        let node = match split_inst_pin(src) {
            Some((inst, pin)) => key2idx.get(&pin_key(inst, pin)).copied(),
            None => key2idx.get(&port_key(src)).copied(),
        };
        if let Some(nd) = node {
            clock_src.insert(nd, *per);
        }
    }
    // ---- is this register's clock pin reached by a DECLARED clock? ------
    // OpenSTA seeds clock-tagged paths only at pins the SDC names as a clock source
    // (`Sdc::isLeafPinClock` gating `seedClkArrivals` in `Search::seedArrival`) and
    // propagates that tag forward; everything else starts as a DATA arrival via
    // `seedInputArrival`. `VisitPathEnds::visitCheckEnd` then builds a setup / hold /
    // recovery / removal `PathEndCheck` only for a target clock path whose
    // `isClock()` is true — with none it leaves `check_clked` false and falls to
    // `visitCheckEndUnclked`, which emits nothing but an explicit set_max_delay /
    // set_min_delay. So a register clocked by a clock the SDC never declares carries
    // NO check at all; it is an unconstrained endpoint, not a passing one.
    //
    // This walk is that tag propagation: forward from every declared clock source,
    // stopping AT a CK pin exactly as the clock-launched pass does (`block_seq`
    // above) — a CK->Q arc starts a new launch and does not carry the clock onward.
    //
    // Measured on fft_top, whose SDC declares clk_i but not pclk_i: 194 of 730 CK
    // pins arrive at ~12.8 ns because pclk_i's 12 ns `set_input_delay` propagates
    // through the clock buffers and was then used as a CAPTURE CLOCK ARRIVAL. That
    // put WHS at -1.0682 where OpenSTA and sign-off both say +0.88, and drove 599
    // false delay-cell insertions into a design that is already hold-clean.
    let mut clock_reached = vec![false; n];
    {
        // a clock source held at a constant launches no clock (`SearchPred0::searchFrom`)
        let mut stack: Vec<usize> = clock_src.keys().copied().filter(|&s| !const_driver[s]).collect();
        for &s in &stack {
            clock_reached[s] = true;
        }
        while let Some(u) = stack.pop() {
            if is_ck[u] {
                continue; // the clock edge stops here; CK->Q is a new launch
            }
            for e in out_edges[u].iter().filter(|e| e.searched() && !e.latch_dq) {
                if !clock_reached[e.to] {
                    clock_reached[e.to] = true;
                    stack.push(e.to);
                }
            }
        }
    }
    // ---- the clock EDGE each clock-network pin's transition carries ------------------------------
    // The reference's clock path keeps the clock edge it was seeded with (`ClkInfo::clkEdge`; a
    // clock source's rising transition carries the rise edge, its falling the fall) through every
    // wire and combinational arc (`Search::thruClkTag` / `thruClkInfo` leave it), while the pin's
    // own transition follows each arc's sense: through an inverting gate a register's rising clock
    // pin is reached by the clock's FALLING edge, so a rising-edge register there launches and
    // captures on the fall. `src_mask[v][lane]`: the source edges (bit 0 rise, bit 1 fall) whose
    // path reaches `v`'s transition `lane`, along the arcs the arrivals take (`feeds`, the
    // constants' lanes), stopping at a register clock pin as `clock_reached` does.
    let mut src_mask = vec![[0u8; 2]; n];
    {
        let mut stack: Vec<usize> = clock_src.keys().copied().filter(|&s| !const_driver[s]).collect();
        for &s in &stack {
            src_mask[s] = [1, 2];
        }
        while let Some(u) = stack.pop() {
            if is_ck[u] {
                continue;
            }
            for e in out_edges[u].iter().filter(|e| e.searched() && !e.latch_dq && clock_reached[e.to]) {
                let mut m = src_mask[e.to];
                for il in 0..2 {
                    for ol in 0..2 {
                        let thru = match &e.kind {
                            EdgeKind::Net(_) => il == ol && e.lanes[il][ol],
                            EdgeKind::Cell(arc) => {
                                arc.clk_edge.is_none() && arc.out_edges.drives(ol == 0) && arc.feeds(il == 0, ol == 0) && e.lanes[il][ol]
                            }
                        };
                        if thru {
                            m[ol] |= src_mask[u][il];
                        }
                    }
                }
                if m != src_mask[e.to] {
                    src_mask[e.to] = m;
                    stack.push(e.to);
                }
            }
        }
    }
    // The clock edge (0 rise, 1 fall) that pin `v`'s transition `lane` belongs to: the one source
    // edge that reaches it, else (no clock, or both edges through a non-unate arc) the transition.
    let src_edge = |v: Option<usize>, lane: usize| -> usize {
        match v.map(|v| src_mask[v][lane]) {
            Some(1) => 0,
            Some(2) => 1,
            _ => lane,
        }
    };
    // A rising-edge register reached by the clock's fall launches on the fall (see `fall_launch`).
    let fall_launch = fall_launch || ck_lane_list.iter().any(|&(i, l)| l.is_some_and(|l| src_edge(Some(i), l) == 1));
    // IDEAL clock networks (an SDC clock with no `set_propagated_clock`): every pin from the
    // source up to the register clock pins arrives at the source's own time — no network delay —
    // with the clock transition as its slew, so a register's CK->Q is looked up at that slew. The
    // reference's rule; a propagated clock is timed through its network as before.
    let mut ideal_slew: Vec<Option<[f64; 2]>> = vec![None; n];
    {
        let mut stack: Vec<usize> = Vec::new();
        for (src, sl) in &job.ideal_clocks {
            let node = match split_inst_pin(src) {
                Some((inst, pin)) => key2idx.get(&pin_key(inst, pin)).copied(),
                None => key2idx.get(&port_key(src)).copied(),
            };
            if let Some(nd) = node.filter(|&nd| !const_driver[nd]) {
                ideal_slew[nd] = Some(*sl);
                stack.push(nd);
            }
        }
        while let Some(u) = stack.pop() {
            if is_ck[u] {
                continue;
            }
            for e in out_edges[u].iter().filter(|e| e.searched() && !e.latch_dq) {
                if ideal_slew[e.to].is_none() {
                    ideal_slew[e.to] = ideal_slew[u];
                    stack.push(e.to);
                }
            }
        }
    }
    // Guarded on having RESOLVED at least one declared clock source. With none we
    // cannot tell a missing `create_clock` from a clock port this model failed to
    // locate, and declining every check would turn that model gap into a silently
    // clean design — the exact failure this rule exists to prevent.
    let clock_declared = !clock_src.is_empty();
    let unclocked_ck = |ck: Option<usize>| -> bool {
        clock_declared && ck.is_some_and(|c| !clock_reached[c])
    };
    // A DATA arrival entering the clock network — a clock gate's enable, from a latch or a
    // register, into its AND gate — is not a clock arrival there. The reference propagates it as
    // a data path, and a data path on a register's clock pin launches nothing (the CK->Q rule
    // above); only the clock's own path sets the clock pin's arrival and slew. Merged with it here,
    // the gate's enable path became the gated registers' clock latency: an ASAP7 latch clock gate
    // put 46 ps on its registers' launch and on their hold requirement. Such an edge sets the
    // slews after it (the delay calculation merges every arc's, whatever its path) and no
    // arrival. The incremental fast path has no slew-only arc, so a design with one leaves it off.
    // (The clock gating check the reference infers at the gate is not made here.)
    let data_into_clock = |u: usize, v: usize| -> bool { clock_declared && !clock_reached[u] && clock_reached[v] };
    // (and the fast path takes a register's clock edge from its library edge alone)
    let simple_ctx = simple_ctx
        && !fall_launch
        && !out_edges.iter().enumerate().any(|(u, es)| es.iter().any(|e| data_into_clock(u, e.to)));

    // ---- inferred clock gating checks ----------------------------------------------------------
    // The reference infers a setup and a hold check at a gate's enable input wherever a data
    // path gates a clock (`GatedClk::isGatedClkEnable`, `VisitPathEnds::visitGatedClkEnd`, on by
    // default): an input that is not a clock, with a combinational arc to an output that is in
    // the clock network, whose function is an AND or an OR gating a clock input of the same
    // instance by it (`clk_gating_active_one`), that clock input carrying the clock to a register
    // clock pin downstream. The check is made against the clock at that clock input: an AND's
    // enable (active value one) must settle before the clock rises there (setup) and hold until
    // it falls (hold); an OR's the opposite edges. Margin zero (`set_clock_gating_check` is not
    // applied here); the reference reports these in their own path group, `gated clock`.
    // Made here as a register check of zero margin, against the clock pin's arrival at that edge.
    let mut gating_end: std::collections::HashSet<usize> = std::collections::HashSet::new();
    if clock_declared {
        // a clock pin that carries the clock on to a register clock pin
        let reaches_ck = |c: usize| -> bool {
            let mut seen = std::collections::HashSet::new();
            let mut stack = vec![c];
            while let Some(u) = stack.pop() {
                if is_ck[u] {
                    return true;
                }
                for e in out_edges[u].iter().filter(|e| e.searched() && !e.latch_dq && clock_reached[e.to]) {
                    if seen.insert(e.to) {
                        stack.push(e.to);
                    }
                }
            }
            false
        };
        let zero = |edge: ClkEdge, clk: &str| Constraint {
            rise: crate::liberty::Table { values: vec![vec![0.0]], ..Default::default() },
            fall: crate::liberty::Table { values: vec![vec![0.0]], ..Default::default() },
            clk_edge: edge,
            related_pin: clk.to_string(),
            ..Default::default()
        };
        for inst in &nl.insts {
            let Some(cell) = lib.cell(&inst.cell) else { continue };
            for (en, _) in &inst.conns {
                let Some(&ei) = key2idx.get(&pin_key(&inst.name, en)) else { continue };
                if clock_reached[ei] || const_driver.get(ei).copied().unwrap_or(false) || cell.pins.get(en).map(|p| p.direction) != Some(Dir::In) {
                    continue;
                }
                // the enable's combinational arcs, to an output in the clock network
                let found = cell.pins.iter().filter(|(_, p)| p.direction == Dir::Out).find_map(|(out, p)| {
                    let oi = *key2idx.get(&pin_key(&inst.name, out))?;
                    if !clock_reached[oi] || !p.arcs.iter().any(|a| a.related_pin == *en && a.clk_edge.is_none()) {
                        return None;
                    }
                    let func = vyges_loom::liberty_func::Func::parse_in(p.function.as_deref()?.trim_matches('"'), |q| cell.pins.contains_key(q))?;
                    func.ports().into_iter().filter(|c| c != en).find_map(|c| {
                        let one = clk_gating_active_one(&func, en, c)?;
                        let ci = *key2idx.get(&pin_key(&inst.name, c))?;
                        (clock_reached[ci] && reaches_ck(ci)).then(|| (c.to_string(), one))
                    })
                });
                if let Some((clk, one)) = found {
                    let (setup_edge, hold_edge) = if one { (ClkEdge::Rise, ClkEdge::Fall) } else { (ClkEdge::Fall, ClkEdge::Rise) };
                    let ck_key = Some(pin_key(&inst.name, &clk));
                    is_endpoint[ei] = true;
                    flop_d.push((ei, Cow::Owned(vec![zero(setup_edge, &clk)]), ck_key.clone()));
                    flop_hold.push((ei, Cow::Owned(vec![zero(hold_edge, &clk)]), ck_key));
                    gating_end.insert(ei);
                }
            }
        }
    }

    // Every arc relaxation of a pass that records (see `crpr_cands`), in the pass's order: set to
    // `Some` around the early passes whose arrivals the hold checks read.
    let crpr_rec: std::cell::RefCell<Option<Vec<ArcRec>>> = std::cell::RefCell::new(None);
    let rec_on = std::cell::Cell::new(false);
    #[allow(clippy::type_complexity)]
    // `launch` (lane 0 rise, 1 fall) times only the paths that clock edge launches — a
    // register's CK->Q fires only if its edge is that one — for the reference's per-edge arrivals
    // (each clock edge is its own tag); `fixed_slew` then looks every delay up at the slews the
    // unfiltered pass merged over ALL launches, as the reference's slews are, so a filtered pass
    // moves arrivals only.
    let relax = |nd: &[[f64; 2]],
                 ns: &[[f64; 2]],
                 late: bool,
                 seed_l: &[[f64; 2]],
                 block_seq: bool,
                 launch: Option<usize>,
                 fixed_slew: Option<&[[f64; 2]]>,
                 latch_seed: Option<&HashMap<usize, LatchSeed>>|
     -> (
        Vec<f64>,
        Vec<f64>,
        Vec<Option<usize>>,
        Vec<usize>,
        Vec<[f64; 2]>,
        Vec<[f64; 2]>,
        Vec<[Pred; 2]>,
    ) {
        let init = if late {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        // this pass re-evaluates every arc of its analysis; the clock-launched pass re-times the
        // same arcs and leaves the main pass's waveforms as they are
        let records = !block_seq && launch.is_none() && fixed_slew.is_none();
        if records {
            drvr_wave.borrow_mut()[late as usize].iter_mut().for_each(|w| w.iter_mut().for_each(Vec::clear));
        }
        let mut arr = vec![[init; 2]; n]; // per-lane metric (derated / +-N*sigma)
        let mut arr_nom = vec![[0.0f64; 2]; n];
        let mut var = vec![[0.0f64; 2]; n];
        let mut depth = vec![[0usize; 2]; n];
        let mut slew = vec![[input_slew; 2]; n];
        // each input port its own `set_input_transition`, per edge, this analysis
        for (p, s) in &port_slew {
            slew[*p] = s[late as usize];
        }
        // a port with a driving cell starts from that cell's slew, not the input slew
        for (p, d) in &port_drive {
            slew[*p] = d.slew[late as usize];
        }
        if let Some(f) = fixed_slew {
            slew.copy_from_slice(f);
        }
        // each lane's predecessor AND the predecessor's lane: an inverting arc feeds the other edge,
        // and a reported path has to follow the edge that set the slack, node by node.
        let mut from: Vec<[Pred; 2]> = vec![[None; 2]; n];
        // a driver lane's slew has been set by one of its arcs (merged from then on)
        let mut slew_set = vec![[false; 2]; n];
        // a lane whose slew is computed though the constants stop its arrival: the reference's
        // delay calculation (`DcalcPred`) skips only arcs FROM a held pin, so a deselected mux
        // input or a pin held rising still sets the slews after it
        let mut slew_only = vec![[false; 2]; n];
        let mut indeg_work = indeg.clone();
        let mut order: Vec<usize> = Vec::new();
        // reached by the delay calculation's walk from the roots (see `dis_src`)
        let mut reached = vec![false; n];
        for v in 0..n {
            if indeg_work[v] == 0 {
                // A tie cell drives a LEVEL, not an edge — it is a source of no
                // transition. Leaving it unreached (rather than seeded at t=0) is what
                // makes its whole fanout untimed: the net arc refuses to propagate from a
                // non-finite arrival, every sink then arrives unreached too, and the
                // endpoint guards drop them. Seeding it at 0 instead invents a launch at
                // time zero and manufactures hold violations on wires that cannot switch.
                //
                // It is still pushed, so the topological walk stays consistent and a gate
                // with one constant input still times correctly through its other inputs.
                let untimed = const_driver.get(v).copied().unwrap_or(false)
                    || unconstrained_in.get(v).copied().unwrap_or(false);
                if !untimed {
                    arr[v] = seed_l[v]; // input ports seed with their SDC arrival delay
                    arr_nom[v] = seed_l[v];
                }
                reached[v] = !dis_src[v];
                order.push(v);
            }
        }
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            // unreached: no slew was calculated for it (the graph's initial 0), and the arcs from
            // it are calculated from that 0 for a load the walk reaches through another arc
            if !reached[u] && fixed_slew.is_none() {
                slew[u] = [0.0; 2];
                slew_only[u] = [true; 2];
            }
            // what reaches a latch output through its D (see `LatchSeed`): merged with what its
            // fan-in brought before anything leaves it
            if let Some(sd) = latch_seed.and_then(|m| m.get(&u)).filter(|_| late) {
                for l in 0..2 {
                    if sd.arr[l] > arr[u][l] {
                        arr[u][l] = sd.arr[l];
                        arr_nom[u][l] = sd.arr[l];
                        var[u][l] = 0.0;
                        depth[u][l] = 0;
                        from[u][l] = sd.from[l];
                    }
                }
            }
            for e in &out_edges[u] {
                // Outside the order (see `Edge::latch_dq`), and nothing crosses it here: a latch
                // output's arrivals come through its enable.
                if e.latch_dq {
                    continue;
                }
                let v = e.to;
                if reached[u] {
                    reached[v] = true;
                }
                let mm = late as usize;
                let load_lanes = node_load[v][mm];
                // A flop's CK->Q arc starts a *new* launch, so a pass that is tracking
                // where a clock edge itself gets to must stop here. The indeg bookkeeping
                // below still runs — only the value is withheld, so the topological walk
                // is identical in both passes.
                let blocked = block_seq && is_ck[u];
                match &e.kind {
                    EdgeKind::Net(i) if !blocked => {
                        // interconnect: rise->rise, fall->fall, same delay, no derate.
                        // The sink slew is the transient-degraded value when available
                        // (>0), else the driver slew passes through unchanged.
                        for l in 0..2 {
                            // an ideal clock network: no wire delay, the clock transition
                            let (d, sink_slew) = match ideal_slew[v] {
                                Some(isl) => (0.0, isl[l].max(f64::MIN_POSITIVE)),
                                None => (nd[*i][l], ns[*i][l]),
                            };
                            // No RC model gave this load a slew (0 = the driver's): the reference
                            // still re-expresses the driver's slew, and shifts the wire delay, at
                            // the LOAD's library thresholds (`LumpedCapDelayCalc::makeResult` →
                            // `thresholdAdjust`), float-stored. Never taken for one library.
                            let (d, sink_slew) = match (sink_slew == 0.0 && ideal_slew[v].is_none(), port_drive.get(&u)) {
                                (true, drive) => {
                                    let (th_d, th_l) = (drive.map_or_else(|| th_of(u), |pd| pd.th), th_of(v));
                                    if th_d != th_l {
                                        let (dd, ss) = threshold_adjust(&th_d, &th_l, l == 0, d, slew[u][l]);
                                        (dd as f32 as f64, ss as f32 as f64)
                                    } else {
                                        (d, sink_slew)
                                    }
                                }
                                (false, _) => (d, sink_slew),
                            };
                            let a = arr[u][l];
                            // a pin held rising / falling passes one edge; the other edge's
                            // slew still reaches the load
                            if (!a.is_finite() && slew_only[u][l]) || (a.is_finite() && !e.lanes[l][l]) {
                                if fixed_slew.is_none() && !arr[v][l].is_finite() {
                                    slew[v][l] = if sink_slew > 0.0 { sink_slew } else { slew[u][l] };
                                    slew_only[v][l] = true;
                                }
                                continue;
                            }
                            if !a.is_finite() {
                                continue;
                            }
                            let metric = a + d; // band carries (no new variance)
                            if rec_on.get() {
                                if let Some(r) = crpr_rec.borrow_mut().as_mut() {
                                    r.push(ArcRec { u: u as u32, v: v as u32, il: l as u8, ol: l as u8, d: metric - a });
                                }
                            }
                            let better = if late {
                                metric > arr[v][l]
                            } else {
                                metric < arr[v][l]
                            };
                            if better {
                                arr[v][l] = metric;
                                arr_nom[v][l] = arr_nom[u][l] + d;
                                var[v][l] = var[u][l];
                                depth[v][l] = depth[u][l];
                                if fixed_slew.is_none() {
                                    slew[v][l] = if sink_slew > 0.0 { sink_slew } else { slew[u][l] };
                                }
                                from[v][l] = Some((u, l));
                            }
                        }
                    }
                    EdgeKind::Cell(arc) if !blocked => {
                        for ol in 0..2 {
                            // a one-edge arc (`combinational_rise` / `_fall`) drives only that output edge;
                            // the other has no table, and read anyway it timed as ZERO delay
                            if !arc.out_edges.drives(ol == 0) {
                                continue;
                            }
                            let (dt, st) = if ol == 0 {
                                (&arc.cell_rise, &arc.rise_transition)
                            } else {
                                (&arc.cell_fall, &arc.fall_transition)
                            };
                            for il in 0..2 {
                                // A SEQUENTIAL arc is launched by ONE clock edge, named by
                                // Liberty as `timing_type : rising_edge` / `falling_edge`.
                                // sky130 declares no `timing_sense` on these, so they parse
                                // as non_unate and WITHOUT this guard both clock edges
                                // launch the flop — which is how the falling edge of the
                                // clock reaches the data path on a min-delay pass and makes
                                // the launch look ~0.2 ns early. `ol` still ranges over both:
                                // one clock edge can drive Q either way.
                                if let Some(e) = arc.clk_edge {
                                    if il != usize::from(e == ClkEdge::Fall) {
                                        continue;
                                    }
                                    // a per-edge pass: only its own edge launches
                                    // (the clock's edge, not the pin's: see `src_edge`)
                                    if launch.is_some_and(|le| le != src_edge(Some(u), il)) {
                                        continue;
                                    }
                                    // ⛔ AND A DATA ARRIVAL AT A CLOCK PIN DOES NOT LAUNCH
                                    // THE FLOP. `Search.cc`'s `regClkToQ` branch propagates
                                    // only when `from_tag->isClock()` — its comment is
                                    // literally "Do not propagate paths from input ports
                                    // with default input arrival clk thru CLK->Q edges" —
                                    // and otherwise sets `to_tag = nullptr`, so nothing
                                    // leaves the flop at all.
                                    //
                                    // This is #88's other half. That rule stopped CHECKING
                                    // registers on an undeclared clock; they went on
                                    // LAUNCHING, carrying `pclk_i`'s 12 ns into the data
                                    // cone of registers that ARE declared. Measured on
                                    // fft_top: 83 of 790 setup endpoints out by 8.9–12.6 ns,
                                    // setup rms 3.6447 against hold's 0.033 — and WNS,
                                    // 1.9 % out, showed none of it.
                                    if clock_declared && !clock_reached[u] {
                                        continue;
                                    }
                                }
                                // does input edge `il` drive output edge `ol`?
                                let feeds = arc.feeds(il == 0, ol == 0); // the reference's edge pairs (latch D->Q, tri-state by kind)
                                if !feeds {
                                    continue;
                                }
                                let a_in = arr[u][il];
                                // the constants narrow the pairs an arrival takes
                                // (`searchThruTimingSense`), not the slews (see `slew_only`)
                                // and a data path entering the clock network sets its slews
                                // (the delay calculation merges every arc's) but not its
                                // arrival (see `data_into_clock`)
                                let only_slew = !e.lanes[il][ol] || !a_in.is_finite() || data_into_clock(u, v);
                                if !a_in.is_finite() && !slew_only[u][il] {
                                    continue;
                                }
                                let sin = slew[u][il];
                                // CCS-into-RC: drive the effective capacitance (resistive
                                // shielding), iterating Ceff <-> output transition to a
                                // self-consistent point rather than a single lumped pass.
                                // On the NLDM path this is the reference's own DMP solve
                                // (`crate::dmp`, transcribed from `DmpPi`): the effective
                                // capacitance from a three-equation Newton system, and a
                                // driver slew taken from the resulting WAVEFORM rather than
                                // from the slew table.
                                //
                                // ℹ️ CCS keeps `ceff_iter`. Per OpenSTA issue #73, from the
                                // maintainer: under CCS "the effective capacitance changes
                                // as the output voltage changes" — there is no single Ceff,
                                // and `report_dcalc` is "a placeholder" there. The DMP solve
                                // answers a question CCS does not ask.
                                let ccs_arc = !arc.ccs.is_empty();
                                let load = load_lanes[ol];
                                let dmp = match (ccs_arc, shield[v][mm][ol]) {
                                    (false, Some((c2, rpi, c1))) => {
                                        let gate =
                                            |c: f64| (dt.lookup(sin, c), st.lookup(sin, c));
                                        let t = dmp_vth(v, ol);
                                        let rd =
                                            crate::dmp::gate_model_rd_step(t.vth, &gate, c1, c2, cap_step);
                                        // ⚠️ Rpi is in ohms; the solver works in the library's
                                        // time per capacitance (kΩ for ns and pF, Ω for ps and
                                        // pF) so that rd·C and rpi·C land in its time unit — and
                                        // takes that unit so its `rd < 1e-2` ohms holds in any.
                                        let r = crate::dmp::solve_driver_r(
                                            rd,
                                            c2,
                                            rpi / ohm_per_r,
                                            c1,
                                            t,
                                            &gate,
                                            ohm_per_r,
                                        );
                                        // `VYGES_DEBUG=sta-si:dmp_ceff=1`: the reference's `dmp_ceff` debug
                                        // line per driver edge, in SI, beside the result.
                                        if dmp_trace {
                                            eprintln!(
                                                "dmp|{}|ol={ol}|in_slew={:e}|c2={:e}|rpi={:e}|c1={:e}|rd={:e}|alg={:?}|ceff={:e}|delay={:e}|slew={:e}",
                                                labels[v], sin * lib.units.time_s, c2 * lib.units.cap_f, rpi, c1 * lib.units.cap_f, rd * ohm_per_r,
                                                crate::dmp::select_alg_r(rd, c2, rpi / ohm_per_r, c1, ohm_per_r),
                                                r.ceff * lib.units.cap_f, r.delay * lib.units.time_s, r.slew * lib.units.time_s
                                            );
                                        }
                                        Some(r)
                                    }
                                    _ => None,
                                };
                                let leff = match (&dmp, shield[v][mm][ol]) {
                                    (Some(r), _) => r.ceff,
                                    (None, Some((c2, rpi, c1))) => crate::ccs::ceff_iter(
                                        c2,
                                        c1,
                                        rpi * c1 * r_scale,
                                        |c| st.lookup(sin, c),
                                    ),
                                    (None, None) => load,
                                };
                                if shield[v][mm][ol].is_some() {
                                    ceff_used.borrow_mut()[v][ol] = leff;
                                }
                                if let Some(w) = dmp.as_ref().and_then(|r| r.waveform).filter(|_| records) {
                                    drvr_wave.borrow_mut()[late as usize][v][ol].push(w);
                                }
                                // CCS current-source delay when the arc carries it; else the
                                // DMP result, else a plain NLDM lookup.
                                let (d, sout) = if ccs_arc {
                                    arc.ccs
                                        .delay_slew(ol == 0, sin, leff, 0.3, 0.7)
                                        .unwrap_or((dt.lookup(sin, leff), st.lookup(sin, leff)))
                                } else if let Some(r) = dmp {
                                    (r.delay, r.slew)
                                } else {
                                    (dt.lookup(sin, leff), st.lookup(sin, leff))
                                };
                                // an ideal clock network: no cell delay, the clock transition
                                let (d, sout) = match ideal_slew[v] {
                                    Some(isl) => (0.0, isl[ol]),
                                    None => (d, sout),
                                };
                                let stage = depth[u][il] + 1;
                                let derate = cell_derate(late, stage);
                                let nom = arr_nom[u][il] + d * derate;
                                // per-stage delay sigma: LVF table (slew/load-dependent)
                                // when present, else the global pocv_sigma fraction. The LVF
                                // table is the analysis's own: `std_dev` when given, else the
                                // LATE sigma on a max (setup) pass and the EARLY one on a min
                                // (hold) pass (`GateTableModel::gateDelayPocv`, normal mode).
                                let sigma = if !pocv {
                                    0.0
                                } else {
                                    let lvf = arc.lvf_cell[ol].delay_sigma(late);
                                    if !lvf.values.is_empty() {
                                        lvf.lookup(sin, leff)
                                    } else {
                                        pocv_sigma * d
                                    }
                                };
                                let var_c = var[u][il] + sigma * sigma;
                                let metric = if pocv {
                                    let band = n_sigma * var_c.sqrt();
                                    if late {
                                        nom + band
                                    } else {
                                        nom - band
                                    }
                                } else {
                                    nom
                                };
                                // ⛔ The driver's slew merges over its arcs ON ITS OWN — the
                                // worse for this analysis (`annotateDelaySlew`: `delayGreater(
                                // gate_slew, prev, min_max)`), not the slew of whichever arc
                                // wins the arrival. A fast-input arc can set the arrival while
                                // a slow-input arc sets the slew.
                                if fixed_slew.is_none() && (!slew_set[v][ol] || (late && sout > slew[v][ol]) || (!late && sout < slew[v][ol])) {
                                    slew[v][ol] = sout;
                                    slew_set[v][ol] = true;
                                }
                                if only_slew {
                                    slew_only[v][ol] = true;
                                    continue;
                                }
                                if rec_on.get() {
                                    if let Some(r) = crpr_rec.borrow_mut().as_mut() {
                                        r.push(ArcRec { u: u as u32, v: v as u32, il: il as u8, ol: ol as u8, d: metric - arr[u][il] });
                                    }
                                }
                                let better = if late {
                                    metric > arr[v][ol]
                                } else {
                                    metric < arr[v][ol]
                                };
                                if better {
                                    arr[v][ol] = metric;
                                    arr_nom[v][ol] = nom;
                                    var[v][ol] = var_c;
                                    depth[v][ol] = stage;
                                    from[v][ol] = Some((u, il));
                                }
                            }
                        }
                    }
                    _ => {} // blocked: bookkeeping only, no value propagated
                }
                indeg_work[v] -= 1;
                if indeg_work[v] == 0 {
                    order.push(v);
                }
            }
        }
        // collapse each node to its worst lane (max for late, min for early)
        let pick = |a: [f64; 2]| -> usize {
            if late {
                if a[0] >= a[1] {
                    0
                } else {
                    1
                }
            } else if a[0] <= a[1] {
                0
            } else {
                1
            }
        };
        let mut arrival = vec![init; n];
        let mut slew_c = vec![input_slew; n];
        let mut from_c: Vec<Option<usize>> = vec![None; n];
        for v in 0..n {
            // At a CLOCK pin the lane is not ours to choose by extremum: `create_clock`
            // fixes the launching edge, and a reference timer carries that edge's own
            // arrival (OpenSTA tags clock paths with a `ClockEdge`; `Search::clkPathArrival`
            // returns that path's arrival and never minimises over rise and fall). Picking
            // `min` here made the early pass report the FALLING edge as the launch time.
            let l = ck_lane[v].unwrap_or_else(|| pick(arr[v]));
            arrival[v] = arr[v][l];
            slew_c[v] = slew[v][l];
            from_c[v] = from[v][l].map(|p| p.0);
        }
        // also hand back the per-lane arrays so the incremental fast path can seed a cone
        // recompute from the committed state (only captured by the final passes when asked).
        (arrival, slew_c, from_c, order, arr, slew, from)
    };

    // Per-arc interconnect delay, iterated to convergence. Each net's switching
    // window is slew-derived; an aggressor's Miller cap is added (window-overlap)
    // at the victim's net node, and a **per-pin tree Elmore** turns the RC network
    // into a distinct delay for each driver→sink arc (lumped R·C fallback when the
    // SPEF has no usable tree). Arrivals set the windows and the windows feed back
    // into arrivals, so we iterate until the per-arc delays stabilise.
    let guard = job.xtalk_window;
    let miller = job.miller;
    // Which interconnect model each arc's delay actually came from. A net that silently
    // drops to a coarser model is indistinguishable in the output from one the fine model
    // handled — the delay is just wrong by an amount nothing reports. Recorded here, at
    // the point of the decision, so the answer cannot drift from the code that makes it.
    // Dump it with VYGES_DEBUG=sta-si:vy_rc_model=1 (and VYGES_STA_RC_NET=<net> for one net).
    const RC_NONE: u8 = 0;
    const RC_TRANSIENT: u8 = 1;
    const RC_TREE_ELMORE: u8 = 2;
    const RC_LUMPED: u8 = 3;
    // Per-net receiver capacitance, keyed by SPEF node — what `ReduceToPi::pinCapacitance`
    // contributes to `downstreamCap` upstream. Built once: it does not depend on arrivals.
    // ⚠️ Liberty caps are pF and SPEF node caps are fF, so convert.
    // Per edge (lane 0 rise, 1 fall): each edge's Elmore walks that edge's receiver caps.
    // `[analysis][edge]` like the driver load.
    let net_pin_cap: Vec<[[HashMap<Name, f64>; 2]; 2]> = {
        let e = || [HashMap::new(), HashMap::new()];
        let mut v: Vec<[[HashMap<Name, f64>; 2]; 2]> = (0..nn).map(|_| [e(), e()]).collect();
        if spef.is_some() {
            for a in arcs.iter() {
                if net_rc[a.net_idx].is_none() {
                    continue;
                }
                if a.sink_pin {
                    if let Some(node) = node_of_pin[a.sink] {
                        let c = pin_cap_node.get(&a.sink).copied().unwrap_or([[0.0; 2]; 2]);
                        for mm in 0..2 {
                            for l in 0..2 {
                                *v[a.net_idx][mm][l].entry(node).or_default() += c[mm][l] * ff_per_cap;
                            }
                        }
                    }
                } else if is_out_port[a.sink] {
                    // A top-level output port: its external load at its own node, as the Pi above
                    // takes it (`ReduceToPi::pinCapacitance` → `Sdc::portExtCap`) — the reduction
                    // that gives the Pi also gives each load's Elmore. Without it the wire delay to
                    // every other load of a net that drives a port missed the port's load.
                    for lanes in v[a.net_idx].iter_mut() {
                        for caps in lanes.iter_mut() {
                            *caps.entry(node_of_port[a.sink]).or_default() += job.output_load * ff_per_cap;
                        }
                    }
                }
            }
        }
        v
    };
    let rc_branch = std::cell::RefCell::new(vec![RC_NONE; n_arcs]);
    // Each load's Elmore, kept across passes while its net's reduction inputs stand, as the
    // reference keeps a driver's reduced parasitic until the parasitic or its load changes.
    // Per arc and analysis: the two edges' time constants. Per net and analysis: the coupling
    // the reduction was made with (bits), the one input that moves between passes; the driver
    // node and the receiver caps are fixed for the run.
    #[allow(clippy::type_complexity)]
    let tau_kept: std::cell::RefCell<Vec<[Option<[Option<f64>; 2]>; 2]>> = std::cell::RefCell::new(vec![[None; 2]; n_arcs]);
    let xc_kept: std::cell::RefCell<Vec<[Option<u64>; 2]>> = std::cell::RefCell::new(vec![[None; 2]; nn]);
    // `rc_model: transient` in the job asks for our waveform model in front of the
    // reference one. Default is the reference.
    let transient_first = job.rc_model.eq_ignore_ascii_case("transient");
    // ⛔ PER ARC PER LANE. The reference computes a wire delay and a sink slew inside the
    // RISE or FALL arc evaluation — `loadDelaySlew` is called from `gateDelay` for one
    // `RiseFall` — so both are per edge. Collapsing them to one value per arc forces a
    // single driver waveform onto both edges, which was measured to take data-hold rms
    // from 0.0329 to 0.0537 when the per-sink crossing solve was wired against it.
    // `mm`: the analysis (0 early, 1 late), whose receiver caps the Elmore walks.
    // ⛔ PER NET, as the reference evaluates a driver: its gate, then the wires of the one net it
    // drives at that gate's final slew (`GraphDelayCalc::findDriverArcDelays`, `gateDelay` then
    // `loadDelaySlew` per load). `net_xc` and `wires_of_net` are that per-net step; `compute_wires`
    // is only the loop over every net that the fixed point below calls.
    // Each net's arcs (its driver to each load), in arc order.
    let arcs_of_net: Vec<Vec<usize>> = {
        let mut v = vec![Vec::new(); nn];
        for (k, a) in arcs.iter().enumerate() {
            v[a.net_idx].push(k);
        }
        v
    };
    let win = |x: [f64; 2]| if x[0] > x[1] { x[0] } else { x[1] };
    let slw_of = |x: [f64; 2]| if x[0] > x[1] { x[0] } else { x[1] };
    // Net `i`'s coupling capacitance seen by its RC network, in fF, from every net's driver
    // arrival `sw` and slew `net_slew` (its aggressors' windows: NOT causal in level order).
    let net_xc = |i: usize, sw: &[[f64; 2]], net_slew: &[[f64; 2]]| -> f64 {
        // Per-net coupling capacitance seen by the RC network, in fF.
        //
        // 🔑 BASE, and it was missing: a coupling capacitor is GROUNDED on the net at
        // `couplingCapFactor()` (default 1.0). That is what the reference does on read —
        // `SpefReader::makeCapacitor`'s two-node form calls `incrCap(node, cap *
        // couplingCapFactor())` on each net unless `read_spef -keep_capacitive_coupling`
        // is given, and it is not given by the flow. So the coupling capacitance is part
        // of the ordinary RC network before any crosstalk analysis happens at all.
        //
        // Our window term is the INCREMENT over that base — `(miller - 1.0) * cc` is
        // exactly "the Miller multiplier minus the grounded copy already counted". It was
        // being used as the whole contribution, so a net's coupling capacitance vanished
        // entirely whenever no aggressor window overlapped, which on the early (hold) pass
        // is always. Measured on fft_top's `net55`: 194.4 fF over 512 aggressors against
        // 560.7 fF of wire — 34.7 % of the net's capacitance simply absent.
        // Coupling stays a per-NET aggregate: our Miller term is one number for the net,
        // so the window is taken from the worst lane rather than split per edge.
        let svi = win(sw[i]);
        let mut x: f64 = net_cpl[i].iter().map(|(_, cc)| *cc).sum();
        if svi.is_finite() {
            for &(ai, cc) in &net_cpl[i] {
                let window = (slw_of(net_slew[i]) + slw_of(net_slew[ai])) / 2.0 + guard;
                if win(sw[ai]).is_finite() && (win(sw[ai]) - svi).abs() <= window {
                    x += (miller - 1.0).max(0.0) * cc;
                }
            }
        }
        x
    };
    // Net `i`'s wire edges for analysis `mm` (0 early, 1 late), from its coupling `xc_i`
    // (`net_xc`) and its driver's slew `slew_i`, per lane: each of its arcs' (net delay,
    // degraded sink slew) written to `out_d[k]` / `out_s[k]`; slew 0 means "keep driver slew".
    // Reads the driver waveforms its gate arcs left in `drvr_wave`, so call it after them.
    let wires_of_net = |i: usize, mm: usize, xc_i: f64, slew_i: [f64; 2], out_d: &mut [[f64; 2]], out_s: &mut [[f64; 2]]| {
        // waveform-into-RC: a transient response per net (driver-slew-aware), memoized
        // so each sink reads the same simulation. Only built when it is actually selected.
        let tr_i: Option<HashMap<Name, (f64, f64)>> = (|| {
            if !transient_first {
                return None;
            }
            let rc = net_rc[i]?;
            net_drv_pin[i].then_some(())?;
            let dn = node_of_pin[net_drv[i]?]?;
            // the solve is in ns: the driver slew in, each delay and slew out
            let map = rc.transient(dn, slw_of(slew_i) * ns_per_time, xc_i, lib.thresholds)?;
            Some(map.into_iter().map(|(n, (d, s))| (n, (d * time_per_ns, s * time_per_ns))).collect())
        })();
        // Each net's reduced Elmore, per edge, made once and read by every load, as the
        // reference reduces a driver's parasitic once (`ReduceToPiElmore`, one `setElmore` per
        // load pin) and each load's arc reads its own (`findElmore`). Made on the first load
        // that asks, from the same inputs every load of the net would give it: the driver node,
        // this pass's coupling and the edge's receiver caps. Held by node (`elmore_by_node`).
        type NetElmore = Option<Vec<(Name, f64)>>;
        let elmore_i: [std::cell::OnceCell<NetElmore>; 2] = Default::default();
        // a net whose coupling moved since its loads' Elmore was kept: reduce it again
        {
            let mut xk = xc_kept.borrow_mut();
            let b = xc_i.to_bits();
            if xk[i][mm] != Some(b) {
                let mut tk = tau_kept.borrow_mut();
                for &k in &arcs_of_net[i] {
                    tk[k][mm] = None;
                }
            }
            xk[i][mm] = Some(b);
        }
        for &k in &arcs_of_net[i] {
            let a = &arcs[k];
            let (d, s) = (|| -> ([f64; 2], [f64; 2]) {
                let i = a.net_idx;
                let mark = |b: u8| rc_branch.borrow_mut()[k] = b;
                let Some(rc) = net_rc[i] else {
                    mark(RC_NONE);
                    return ([0.0, 0.0], [0.0, 0.0]); // no parasitics -> ideal interconnect
                };
                // ---- BASE MODEL: per-sink Elmore -----------------------------------
                // OpenSTA's default arc delay calculator is `dmp_ceff_elmore`
                // (`Sta.cc:426`), and its per-sink half takes the sink's ELMORE TIME
                // CONSTANT and produces a wire delay and a degraded slew.
                //
                // ⚠️ WHAT FOLLOWS IS AN APPROXIMATION, AND THE REFERENCE IS NOT ONE LAW.
                // `DmpCeffDelayCalc::setCeffAlgorithm` picks per driver from Rd, Rpi, c1
                // and c2, and the two branches do DIFFERENT things:
                //   · `DmpCap::loadDelaySlew` — `delay = elmore` (the RAW time constant,
                //     no conversion) and `slew = drvr_slew` (NO degradation at all);
                //   · `DmpAlg::loadDelaySlew` (DmpPi / DmpZeroC2) — put a pole at
                //     `1/elmore` on the DMP driver waveform and solve for the actual
                //     threshold crossings.
                // We can select neither yet: the branch test needs the THREE-element Pi
                // (c2, rpi, c1) from `reduceToPi` plus `gateModelRd`, and our `pi_reduce`
                // yields only (C1, tau). Measured on `fft_top`'s `net55` the reference
                // took the CROSSING SOLVE — its sink slew 1.6616 differs from its driver
                // slew 1.1048, which `DmpCap` could not produce.
                //
                // So this uses the closed form from `DelayCalcBase::dspfWireDelaySlew`,
                // which upstream applies on the DSPF/input-port path:
                //
                //     wire_delay = -tau * ln(1 - Vth)
                //     load_slew  = drvr_slew + tau * ln((1 - Vl) / (1 - Vh)) / slew_derate
                //
                // ⚠️ Returning `tau` itself with no slew degradation — what we did before
                // — is exactly `DmpCap`, so it was right for THAT branch and wrong for the
                // other. Measured A/B on fft_top with everything else equal, the two are a
                // wash (WNS 6.7073 vs 6.6983, WHS 0.8286 vs 0.8262 against 6.85/0.8821), so
                // the per-sink law is now a second-order term: the dominant one is the
                // DRIVER SLEW, 0.3036 against the reference's 1.1048.
                //
                // ⚠️ DIVERGENCE, deliberate: upstream evaluates this per rise/fall with
                // that edge's thresholds, and our `compute` is per ARC rather than per lane,
                // so the rise thresholds are used for both. sky130 declares them symmetric
                // (0.2/0.8/0.5), so it costs nothing here — but it would on a library that
                // does not.
                // A port end is its own node, named by the port: an input port drives its net
                // (with its driving cell's waveform, else the closed form) and an output port is
                // a load with its own Elmore, as an instance pin is.
                {
                    let dnode = if net_drv_pin[i] { net_drv[i].and_then(|d| node_of_pin[d]) } else { net_drv[i].map(|d| node_of_port[d]) };
                    let snode = if a.sink_pin { node_of_pin[a.sink] } else { Some(node_of_port[a.sink]) };
                    if let (Some(dt), Some(st)) = (dnode, snode) {
                        let kept = tau_kept.borrow()[k][mm];
                        let taus = kept.unwrap_or_else(|| {
                            let t = [0, 1].map(|l| elmore_i[l].get_or_init(|| rc.elmore(dt, xc_i, |k| net_pin_cap[i][mm][l].get(&k).copied()).map(elmore_by_node)).as_ref().and_then(|dl| elmore_at(dl, st)));
                            tau_kept.borrow_mut()[k][mm] = Some(t);
                            t
                        });
                        if let [Some(_), Some(_)] = taus {
                            {
                                mark(RC_TREE_ELMORE);
                                // ⛔ TYPES (`cpp-to-rust-numeric-reference.md` §2 — narrowing
                                // is the rounding people miss). The reduced Elmore is STORED
                                // and READ as float: `setElmore(Parasitic*, const Pin*,
                                // float)` and `findElmore(..., float &elmore, ...)`. So the
                                // double the DFS accumulated is rounded to f32 before any
                                // consumer sees it.
                                let taus = taus.map(|t| (t.unwrap() * time_per_ns) as f32 as f64);
                                // PER EDGE: each lane has its own driver slew and its own
                                // thresholds, exactly as the reference evaluates one arc.
                                // The arithmetic is double — `1.0` in `log(1.0 - vth)`
                                // promotes the float thresholds — but both results land in
                                // `ArcDelay`/`Slew`, and `Delay.hh` has `typedef float
                                // Delay`, which is a SECOND narrowing.
                                // The closed form reads the LOAD's library thresholds
                                // (`dspfWireDelaySlew`, `thresholdLibrary(load_pin)`); a port
                                // takes the first library's. The driver's are for the
                                // conversion after (`thresholdAdjust`).
                                let th = th_of(a.sink);
                                let th_d = net_drv[i].map_or(lib.thresholds, |dn| port_drive.get(&dn).map_or_else(|| th_of(dn), |pd| pd.th));
                                let mut dl = [0.0f64; 2];
                                let mut sl = [0.0f64; 2];
                                // The reference's own per-sink model, when this edge's
                                // driver produced a waveform: `DmpAlg::loadDelaySlew` puts
                                // a third pole at 1/tau on it and solves for the actual
                                // Vth/Vl/Vh crossings. Its degenerate branch is the closed
                                // form's degenerate branch, so the fallback below is only
                                // for an edge that has not been evaluated yet.
                                let waves = drvr_wave.borrow();
                                for l in 0..2 {
                                    // this edge's Elmore: its own receiver caps
                                    let tau = taus[l];
                                    // every arc's waveform onto this edge, merged to the worse
                                    // for this analysis (a port: its one drive waveform)
                                    let ws: Vec<crate::dmp::DriverWaveform> = match net_drv[i] {
                                        Some(dn) => match port_drive.get(&dn) {
                                            Some(pd) => pd.wave[mm][l].into_iter().collect(),
                                            None => waves[mm][dn][l].clone(),
                                        },
                                        None => Vec::new(),
                                    };
                                    if !ws.is_empty() {
                                        let worse = |a: f64, b: f64| if mm == 1 { a.max(b) } else { a.min(b) };
                                        let mut merged: Option<(f64, f64)> = None;
                                        for w in &ws {
                                            let (d, sv) = w.load_delay_slew(tau);
                                            let (d, sv) = (d as f32 as f64, sv as f32 as f64);
                                            merged = Some(merged.map_or((d, sv), |(md, ms)| (worse(md, d), worse(ms, sv))));
                                        }
                                        let (d, sv) = merged.expect("non-empty");
                                        dl[l] = d;
                                        sl[l] = sv;
                                        continue;
                                    }
                                    let drvr_slew = slew_i[l];
                                    // degenerate case, verbatim from
                                    // `DmpAlg::loadDelaySlew`: an Elmore small against the
                                    // driver slew IS the delay, and the slew is undegraded.
                                    if tau == 0.0 || tau < drvr_slew * 1e-3 {
                                        dl[l] = tau;
                                        sl[l] = drvr_slew;
                                        continue;
                                    }
                                    let (vth, vl, vh) = if l == 0 {
                                        (th.input_rise, th.slew_lower_rise, th.slew_upper_rise)
                                    } else {
                                        (th.input_fall, th.slew_lower_fall, th.slew_upper_fall)
                                    };
                                    let d = (-tau * (1.0 - vth).ln()) as f32 as f64;
                                    let sv = (drvr_slew
                                        + tau * ((1.0 - vl) / (1.0 - vh)).ln() / th.slew_derate)
                                        as f32
                                        as f64;
                                    // upstream's two guards
                                    dl[l] = if d < 0.0 { tau } else { d };
                                    sl[l] = sv.max(drvr_slew);
                                }
                                // Driver and load from libraries whose thresholds differ: the
                                // reference re-expresses both at the load's. ⚠️ It does so on
                                // the double before the float store; here on the stored float
                                // (at most an f32 ulp apart). Never taken for one library.
                                if th_d != th {
                                    for l in 0..2 {
                                        let (d, s) = threshold_adjust(&th_d, &th, l == 0, dl[l], sl[l]);
                                        dl[l] = d as f32 as f64;
                                        sl[l] = s as f32 as f64;
                                    }
                                }
                                return (dl, sl);
                            }
                        }
                    }
                }
                // ---- ADDITION, opt-in: our waveform-into-RC transient ----------------
                // Asked for with `rc_model: transient`. It is an extension of the model
                // above, not a replacement, and it must not run unless requested — it took
                // precedence silently and displaced the reference model entirely.
                if transient_first {
                    if let (Some(map), true) = (&tr_i, a.sink_pin) {
                        if let Some(sn) = node_of_pin[a.sink] {
                            if let Some(&(delay, slew)) = map.get(&sn) {
                                mark(RC_TRANSIENT);
                                // one transient per net, so both edges share it
                                return ([delay, delay], [slew, slew]);
                            }
                        }
                    }
                }
                // last resort: lumped Elmore (R·C) + lumped crosstalk (R·xtalk-cap)
                // ⚠️ This branch also returns slew 0, i.e. "keep the driver slew" — so a net
                // that lands here degrades no slew at all, and every downstream delay is
                // looked up at too sharp a transition.
                mark(RC_LUMPED);
                let d = net_res[i] * net_cap[i] * fs_to_time + net_res[i] * xc_i * fs_to_time;
                ([d, d], [0.0, 0.0])
            })();
            out_d[k] = d;
            out_s[k] = s;
        }
    };
    let compute_wires = |sw: &[[f64; 2]], net_slew: &[[f64; 2]], mm: usize| -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
        let mut d = vec![[0.0; 2]; n_arcs];
        let mut s = vec![[0.0; 2]; n_arcs];
        for i in 0..nn {
            wires_of_net(i, mm, net_xc(i, sw, net_slew), net_slew[i], &mut d, &mut s);
        }
        (d, s)
    };

    // every wire edge from a port with a driving cell also carries the cell's load-dependent
    // delay, `gate - intrinsic` (`annotateLoadDelays`)
    // net `i`'s share for analysis `mm`, per lane, added to each of its wire edges; None for a
    // net no driving-cell port drives
    let port_extra = |i: usize, mm: usize| -> Option<[f64; 2]> { net_drv[i].and_then(|dn| port_drive.get(&dn)).map(|pd| pd.extra[mm]) };
    let compute = |sw: &[[f64; 2]], net_slew: &[[f64; 2]], mm: usize| -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
        let (mut d, s) = compute_wires(sw, net_slew, mm);
        if !port_drive.is_empty() {
            for (dk, a) in d.iter_mut().zip(arcs.iter()) {
                if let Some(extra) = port_extra(a.net_idx, mm) {
                    for (x, e) in dk.iter_mut().zip(extra) {
                        *x += e;
                    }
                }
            }
        }
        (d, s)
    };

    drop(stage_graph);
    let stage_delays = crate::debug::stage("findDelays");
    const MAX_SI_ITERS: usize = 20;
    const SI_TOL: f64 = 1e-9; // ns — per-arc delay change below which we stop
    let neg = vec![[f64::NEG_INFINITY; 2]; nn];
    let zero = vec![[0.0f64; 2]; nn];
    // A STARTING POINT for the iteration below, nothing more: no windows and no slews yet
    // exist, so this is a degenerate operating point and its delays are systematically
    // small. ⚠️ It is not a usable answer — see `early_d`/`early_s` after the loop.
    let (nom_d, nom_s) = compute(&neg, &zero, 1);
    let mut arc_d = nom_d.clone();
    let mut arc_s = nom_s.clone();
    // The converged per-net driver slew, kept past the loop: the EARLY interconnect has to
    // be evaluated at the same operating point as the late one.
    let mut conv_net_slew = zero.clone();
    let mut cycle_checked = false;
    for _ in 0..MAX_SI_ITERS {
        let (_a, _s, _f, ord, arr, slw, _fl) = relax(&arc_d, &arc_s, true, &seed_main, false, None, None, None);
        if !cycle_checked {
            if ord.len() != n {
                // name one cycle the order could not pass: its pins, in order
                let mut done = vec![false; n];
                for &v in &ord {
                    done[v] = true;
                }
                let adj: Vec<Vec<usize>> = out_edges.iter().map(|es| es.iter().filter(|e| !e.latch_dq).map(|e| e.to).collect()).collect();
                let cycle = find_cycle(&adj, &done).unwrap_or_default();
                return Err(StaError::CombinationalLoop(cycle.into_iter().map(|v| labels[v].clone()).collect()));
            }
            cycle_checked = true;
        }
        // per-lane driver arrival and slew — `relax` hands the per-lane arrays back
        let sw: Vec<[f64; 2]> = (0..nn)
            .map(|i| net_drv[i].map(|d| arr[d]).unwrap_or([f64::NEG_INFINITY; 2]))
            .collect();
        let net_slew: Vec<[f64; 2]> = (0..nn)
            .map(|i| net_drv[i].map(|d| slw[d]).unwrap_or([0.0; 2]))
            .collect();
        let (nd, ns) = compute(&sw, &net_slew, 1);
        let delta = (0..n_arcs)
            .map(|k| (nd[k][0] - arc_d[k][0]).abs().max((nd[k][1] - arc_d[k][1]).abs()))
            .fold(0.0, f64::max);
        conv_net_slew = net_slew;
        arc_d = nd;
        arc_s = ns;
        if delta < SI_TOL {
            break;
        }
    }
    // Say which interconnect model the design actually got. Opt-in: it is a diagnostic,
    // and the counts are only meaningful once the loop above has converged.
    if crate::debug::on(crate::debug::RC_MODEL) {
        let b = rc_branch.borrow();
        let mut tally = [0usize; 4];
        for &x in b.iter() {
            tally[x as usize] += 1;
        }
        // Does the DRIVER get an effective capacitance, or the total? Upstream always
        // characterises the gate against the Pi model from the reduction
        // (`DmpCeffDelayCalc::gateDelay` -> `piModel` -> `setCeffAlgorithm` ->
        // `gateDelaySlew`), and the driver slew that comes out is what anchors every
        // sink's `loadDelaySlew`. A driver with no Pi here falls back to the TOTAL load,
        // which double-counts capacitance the wire resistance shields.
        let drivers: usize = nets.values().filter(|nt| nt.driver.is_some()).count();
        let with_pi: usize = nets
            .values()
            .filter_map(|nt| nt.driver)
            .filter(|d| shield[*d][1].iter().any(Option::is_some))
            .count();
        eprintln!(
            "rc-trace: driver Pi (Ceff) available on {with_pi} of {drivers} driven net(s) — \
             the rest use the TOTAL load"
        );
        eprintln!(
            "rc-trace: {} interconnect arc(s) — transient {} · tree-elmore {} · lumped {} · \
             no-parasitics {}",
            b.len(),
            tally[RC_TRANSIENT as usize],
            tally[RC_TREE_ELMORE as usize],
            tally[RC_LUMPED as usize],
            tally[RC_NONE as usize]
        );
        // per-net, worst first: a net whose arcs did NOT reach the transient model is the
        // one whose per-sink delays are flattened.
        let mut per_net: HashMap<usize, [usize; 4]> = HashMap::new();
        for (k, a) in arcs.iter().enumerate() {
            per_net.entry(a.net_idx).or_insert([0; 4])[b[k] as usize] += 1;
        }
        let want = std::env::var("VYGES_STA_RC_NET").ok();
        let mut rows: Vec<(usize, [usize; 4])> = per_net.into_iter().collect();
        rows.sort_by_key(|(_, c)| {
            std::cmp::Reverse(c[RC_TREE_ELMORE as usize] + c[RC_LUMPED as usize])
        });
        for (i, c) in rows.iter().take(10) {
            let fan: usize = c.iter().sum();
            eprintln!(
                "rc-trace:   net {:<28} fanout {:>4} — transient {} · tree-elmore {} · \
                 lumped {} · none {}",
                net_order[*i],
                fan,
                c[RC_TRANSIENT as usize],
                c[RC_TREE_ELMORE as usize],
                c[RC_LUMPED as usize],
                c[RC_NONE as usize]
            );
        }
        if let Some(w) = want {
            match rows.iter().find(|(i, _)| net_order[*i] == w) {
                Some((i, c)) => eprintln!(
                    "rc-trace: ASKED net {} — fanout {} · transient {} · tree-elmore {} · \
                     lumped {} · none {} · res {:.4} ohm · cap {:.4} fF",
                    net_order[*i],
                    c.iter().sum::<usize>(),
                    c[RC_TRANSIENT as usize],
                    c[RC_TREE_ELMORE as usize],
                    c[RC_LUMPED as usize],
                    c[RC_NONE as usize],
                    net_res[*i],
                    net_cap[*i]
                ),
                // (coupling total reported on the next line)
                None => eprintln!("rc-trace: ASKED net {w} — not found"),
            }
            // Coupling capacitance is reported separately because the reference GROUNDS it
            // by default (`SpefReader::makeCapacitor` two-node form, at
            // `couplingCapFactor()`, unless `read_spef -keep_capacitive_coupling`), whereas
            // our `xc` only adds `(miller-1)*Cc` for window-overlapping aggressors. If those
            // two differ, the RC network itself differs before any delay model runs.
            // Our driver Pi, to diff against `report_dcalc`'s `Pi model C2=.. Rpi=.. C1=..`.
            if let Some((nm, nt)) = nets.iter().find(|(nm, _)| {
                std::env::var("VYGES_STA_RC_NET").ok().as_deref() == Some(nm.as_str())
            }) {
                if let Some(d) = nt.driver {
                    match shield[d][1][0] {
                        Some((c2, rpi, c1)) => eprintln!(
                            "rc-trace: ASKED net {nm} DRIVER Pi — ours C2={c2:.4} \
                             Rpi={:.4}k C1={c1:.4} (total {:.4}) · Ceff rise={:.4}                              fall={:.4} · load {:.4} pF",
                            rpi / ohm_per_r,
                            c2 + c1,
                            ceff_used.borrow()[d][0],
                            ceff_used.borrow()[d][1],
                            node_load[d][1][0]
                        ),
                        None => eprintln!(
                            "rc-trace: ASKED net {nm} DRIVER — NO Pi; gate sees the TOTAL \
                             load {:.4} pF",
                            node_load[d][1][0]
                        ),
                    }
                }
            }
            if let Some(i) = rows.iter().map(|(i, _)| *i).find(|i| {
                std::env::var("VYGES_STA_RC_NET").ok().as_deref() == Some(net_order[*i])
            }) {
                let cc: f64 = net_cpl[i].iter().map(|(_, c)| *c).sum();
                eprintln!(
                    "rc-trace: ASKED net {} — coupling {:.4} fF over {} aggressor(s), \
                     ground/wire cap {:.4} fF ⇒ grounding coupling would add {:.1} %",
                    net_order[i],
                    cc,
                    net_cpl[i].len(),
                    net_cap[i],
                    100.0 * cc / net_cap[i].max(1e-12)
                );
            }
        }
    }

    // The EARLY (min-delay) interconnect: no coupling windows — a hold path is scored
    // without the aggressor pessimism the late path carries — but at the CONVERGED
    // OPERATING POINT, exactly like the late one.
    //
    // ⛔ This used to reuse `nom_d`/`nom_s`, the loop's zero-slew SEED. That is not
    // "no crosstalk", it is "no slew": every early delay came out systematically small,
    // and disabling coupling entirely (`miller: 1.0`) did not move the answer by a
    // picosecond, which is what proved the slew and not the coupling was responsible.
    // OpenSTA never does this — it varies min against max by derating and by a coupling
    // cap factor per analysis point, and always evaluates at the real operating point.
    // Measured on fft_top: WHS 0.6607 -> ~0.878 against OpenSTA's 0.8821.
    //
    // ⛔ AND IT NEEDS ITS OWN FIXED POINT, for the same reason the late one does. A net
    // hands its sinks a DEGRADED slew, and that slew is what the next cell's delay is
    // looked up at — so seeding the early interconnect from the LATE driver slews carries
    // late slews into early delays. Measured on `_11950_/A0`: sink slew 1.0786, which is
    // the LATE slew at its driver `_12893_/Q`, where the early slew there is 0.4032. That
    // one pin's cell delay came out 0.211 ns long and the endpoint read 0.277 ns
    // OPTIMISTIC — the direction that misses a real hold violation.
    let (early_d, early_s) = {
        let (mut d, mut sl) = compute(&neg, &conv_net_slew, 0); // seeded from the late point
        for _ in 0..MAX_SI_ITERS {
            let (_a, _s2, _f, _o, _ar, slw, _fl) = relax(&d, &sl, false, &seed_early, false, None, None, None);
            let e_net_slew: Vec<[f64; 2]> = (0..nn)
                .map(|i| net_drv[i].map(|dv| slw[dv]).unwrap_or([0.0; 2]))
                .collect();
            let (nd, ns) = compute(&neg, &e_net_slew, 0);
            let delta = (0..n_arcs)
                .map(|k| (nd[k][0] - d[k][0]).abs().max((nd[k][1] - d[k][1]).abs()))
                .fold(0.0, f64::max);
            d = nd;
            sl = ns;
            if delta < SI_TOL {
                break;
            }
        }
        (d, sl)
    };
    drop(stage_delays);
    let stage_arrivals = crate::debug::stage("findArrivals");
    // final late propagation consistent with the converged per-arc delays, and the
    // early (min-delay) propagation used for hold and for early clock arrivals.
    // CRPR candidates are tracked only where the merged arrival can mislead: a CRPR credit that
    // differs between launches, without per-stage (AOCV) or statistical (POCV) delays, which do
    // not add along a path. On the setup side, not where a latch passes data on (its outputs'
    // arrivals are re-propagated from its window, see `latch_seeds`).
    let track_cands = job.crpr && !pocv && !aocv;
    let track_setup = track_cands && latch_d.is_empty();
    let rec_pass = |on: bool| {
        rec_on.set(on);
        *crpr_rec.borrow_mut() = on.then(Vec::new);
    };
    let mut late_recs: Vec<Vec<ArcRec>> = Vec::new();
    rec_pass(track_setup && !fall_launch);
    let (arrival, slew, from, order_late, late_arr, late_slew, late_from) =
        relax(&arc_d, &arc_s, true, &seed_main, false, None, None, None);
    if let Some(r) = crpr_rec.borrow_mut().take() {
        late_recs.push(r);
    }
    rec_pass(false);
    rec_pass(track_cands && !fall_launch);
    let (arr_min, slew_min, from_min, _ord_min, early_arr, early_slew, early_from) =
        relax(&early_d, &early_s, false, &seed_early, false, None, None, None);
    let mut early_recs: Vec<Vec<ArcRec>> = Vec::new();
    if let Some(r) = crpr_rec.borrow_mut().take() {
        early_recs.push(r);
    }
    rec_pass(false);

    // ---- per launching edge -----------------------------------------------
    // The reference keeps one arrival per clock EDGE that launched it (a tag per `ClockEdge`) and
    // checks each against the capture edge its cycle accounting pairs with that launch. The passes
    // above merge every launch into one arrival measured from its own edge, which is exact while
    // every register launches on the same edge. Where one launches on the fall, each edge is timed
    // alone — its own launches only (inputs launch on the rise), at the slews merged over all of
    // them — and every endpoint is checked once per launching edge.
    // each edge's own launches: its input delays and, on both, the clock sources
    let edge_seed = |e: usize, mm: usize| -> Vec<[f64; 2]> {
        (0..n).map(|i| if clock_src.contains_key(&i) { [0.0; 2] } else { in_seed[e][mm][i] }).collect()
    };
    let edge_runs: Vec<EdgeRun> = if fall_launch {
        [0, 1]
            .into_iter()
            .map(|edge| {
                let (sd, sd_early) = (edge_seed(edge, 1), edge_seed(edge, 0));
                rec_pass(track_setup);
                let (_, _, from, _, late_arr, _, late_from) = relax(&arc_d, &arc_s, true, &sd, false, Some(edge), Some(&late_slew), None);
                if let Some(r) = crpr_rec.borrow_mut().take() {
                    late_recs.push(r);
                }
                rec_pass(track_cands);
                let (_, _, from_min, _, early_arr, _, early_from) =
                    relax(&early_d, &early_s, false, &sd_early, false, Some(edge), Some(&early_slew), None);
                if let Some(r) = crpr_rec.borrow_mut().take() {
                    early_recs.push(r);
                }
                rec_pass(false);
                EdgeRun { edge, late_arr, late_from, from, early_arr, early_from, from_min }
            })
            .collect()
    } else {
        Vec::new()
    };

    // Clock-launched arrivals: the same late propagation, but started at the clock
    // source's own edge times and stopped at every flop CK pin. A clock port driving
    // an output port (a clock forwarded off-chip) is a real timed path, and it is one
    // the main pass cannot see: there the falling edge leaves at half a period, so its
    // arrival is that much later than a delay measured from the launch edge. On a block
    // with a forwarded clock this is routinely the critical path.
    let (clk_arr, clk_slew, clk_from, _clk_ord, _, _, _) = if have_clock_launch {
        relax(&arc_d, &arc_s, true, &seed_clk, true, None, None, None)
    } else {
        (
            vec![f64::NEG_INFINITY; n],
            vec![input_slew; n],
            vec![None; n],
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        )
    };

    drop(stage_arrivals);
    let _stage_path_ends = crate::debug::stage("findPathEnds");
    // ---- clock paths + CRPR ---------------------------------------------
    // Proper OCV uses opposite clock corners on launch vs capture (setup: late
    // launch / early capture; hold: early launch / late capture). The launch and
    // capture clock paths share a segment from the root to their common point;
    // deriving that shared segment two ways is unphysical pessimism, so CRPR credits
    // back its OCV spread (late arrival − early arrival at the common point).
    let ck_node = |k: &Option<String>| -> Option<usize> {
        k.as_deref().and_then(|s| key2idx.get(s)).copied()
    };
    // clock-network path from a CK pin back to the root, using clock-tree topology
    let path_to_root = |start: usize| -> Vec<usize> {
        let mut p = vec![start];
        let mut v = start;
        while let Some(u) = from[v] {
            p.push(u);
            v = u;
            if p.len() > n {
                break; // safety against a pathological cycle
            }
        }
        p
    };
    // deepest node shared by both clock paths (least common ancestor toward root)
    let common_point = |a: usize, b: usize| -> Option<usize> {
        let pb: HashMap<usize, ()> = path_to_root(b).into_iter().map(|x| (x, ())).collect();
        path_to_root(a).into_iter().find(|x| pb.contains_key(x))
    };
    // first clock pin reached walking the data path backward = the launch flop's CK
    let launch_ck = |endpoint: usize, pred: &[Option<usize>]| -> Option<usize> {
        let mut v = endpoint;
        while let Some(u) = pred[v] {
            if is_ck[u] {
                return Some(u);
            }
            v = u;
        }
        None
    };
    let crpr_on = job.crpr;
    // CRPR credit for a (launch CK, capture CK) pair: the OCV spread at their
    // common clock node, i.e. late − early arrival there (>= 0).
    // The edge a clock path carries at node `p`: from its clock pin `ck`, arriving on `lane`, back
    // along that analysis's per-edge predecessors.
    let lane_at = |from: &[[Pred; 2]], ck: usize, lane: usize, p: usize| -> Option<usize> {
        let (mut v, mut l) = (ck, lane);
        loop {
            if v == p {
                return Some(l);
            }
            (v, l) = from[v][l]?;
        }
    };
    // CRPR as the reference finds it (`CheckCrpr::findCrpr` / `findCrpr1`, `same_pin` mode): at the
    // pin the launch and capture clock paths share, each path's own spread — its arrival against
    // the OPPOSITE analysis's arrival of the SAME edge there — and the smaller of the two. `setup`:
    // the launch clock path is the late one and the capture the early, else the reverse. One
    // corner, no derate: the two analyses agree edge for edge, and the credit is 0. Taken from the
    // nodes' collapsed arrivals instead (the latest edge late, the earliest early) it was the
    // rise/fall spread of the clock tree, a credit the reference never gives.
    let crpr_credit = |lck: usize, cck: usize, cl: usize, setup: bool| -> f64 {
        if !crpr_on {
            return 0.0;
        }
        let Some(p) = common_point(lck, cck) else { return 0.0 };
        let (lf, cf) = if setup { (&late_from, &early_from) } else { (&early_from, &late_from) };
        let ll = ck_lane[lck].unwrap_or(0);
        let (Some(ls), Some(cs)) = (lane_at(lf, lck, ll, p), lane_at(cf, cck, cl, p)) else { return 0.0 };
        let spread = |l: usize| ((late_arr[p][l] as f32) - (early_arr[p][l] as f32)).abs() as f64;
        spread(ls).min(spread(cs))
    };

    // ---- multi-clock: which clock reaches each flop, and the launch→capture
    // edge relation between two clock domains ------------------------------
    // Map each clock's source node to its period (a single-clock job synthesizes
    // one entry from clock_port/period_ns). A generated/divided clock is just an
    // entry whose source is an internal pin.
    // Say what was skipped. Leaving these endpoints out makes them vanish from
    // WNS/WHS, and an incomplete SDC must not read as a clean design, so the count
    // is on stderr — the same argument as the unconstrained-input-port note above.
    if clock_declared {
        let unclocked: Vec<usize> = (0..n).filter(|&i| is_ck[i] && !clock_reached[i]).collect();
        if !unclocked.is_empty() {
            let total = is_ck.iter().filter(|&&b| b).count();
            let head: Vec<&str> = unclocked.iter().take(5).map(|&i| labels[i].as_str()).collect();
            vyges_events::emit(
                &vyges_events::Event::new(
                    "vyges-sta-si",
                    vyges_events::Severity::Warn,
                    format!(
                        "{} of {total} register clock pin(s) are not reached by a declared clock and carry no setup/hold/recovery/removal check ({}{}). Add create_clock for the missing clock.",
                        unclocked.len(),
                        head.join(", "),
                        if unclocked.len() > head.len() { ", \u{2026}" } else { "" }
                    ),
                )
                .with_code("STA-UNCLOCKED-REGISTER")
                .with_objects(head.iter().map(|p| format!("pin:{p}")).collect()),
            );
        }
    }
    // period of the clock reaching a CK pin = nearest clock source toward the root
    let clock_period_of = |ck: usize| -> f64 {
        for node in path_to_root(ck) {
            if let Some(&p) = clock_src.get(&node) {
                return p;
            }
        }
        period // fallback: the primary period
    };
    // asynchronous clock groups (set_clock_groups -asynchronous): map each clock
    // SOURCE node to its group index; a path whose launch and capture resolve to
    // different groups is cut — no setup OR hold check. CDC synchronizer paths are
    // intentionally unrelated, so they must not show cross-domain violations.
    let group_of_name: HashMap<&str, usize> = job
        .async_groups
        .iter()
        .enumerate()
        .flat_map(|(gi, g)| g.iter().map(move |n| (n.as_str(), gi)))
        .collect();
    let mut clock_group_node: HashMap<usize, usize> = HashMap::new();
    if !group_of_name.is_empty() {
        for (name, src, _per) in job.clocks.iter() {
            let Some(&gi) = group_of_name.get(name.as_str()) else {
                continue;
            };
            let node = match split_inst_pin(src) {
                Some((inst, pin)) => key2idx.get(&pin_key(inst, pin)).copied(),
                None => key2idx.get(&port_key(src)).copied(),
            };
            if let Some(nd) = node {
                clock_group_node.insert(nd, gi);
            }
        }
    }
    let clock_group_of = |ck: usize| -> Option<usize> {
        path_to_root(ck)
            .into_iter()
            .find_map(|n| clock_group_node.get(&n).copied())
    };
    // From an SDC, clocks are separated PER COMMAND, by the reference's rule — see
    // `sdc::clocks_unrelated`; each clock's source node is mapped to its NAME for that. Groups
    // from a job file keep the one-list rule.
    let clock_name_node: HashMap<usize, &str> = job
        .clocks
        .iter()
        .filter_map(|(name, src, _)| {
            let node = match split_inst_pin(src) {
                Some((inst, pin)) => key2idx.get(&pin_key(inst, pin)).copied(),
                None => key2idx.get(&port_key(src)).copied(),
            };
            node.map(|nd| (nd, name.as_str()))
        })
        .collect();
    let clock_name_of = |ck: usize| -> Option<&str> { path_to_root(ck).into_iter().find_map(|n| clock_name_node.get(&n).copied()) };
    let is_async = |lck: Option<usize>, cap: Option<usize>| -> bool {
        if !job.clock_groups.is_empty() {
            return match (lck.and_then(clock_name_of), cap.and_then(clock_name_of)) {
                (Some(a), Some(b)) => crate::sdc::clocks_unrelated(&job.clock_groups, a, b),
                _ => false,
            };
        }
        match (lck.and_then(clock_group_of), cap.and_then(clock_group_of)) {
            (Some(a), Some(b)) => a != b,
            _ => false,
        }
    };
    // The same separation between two clocks by NAME — an I/O delay's clock, which may be
    // virtual and so has no source node to map.
    let is_async_named = |a: Option<&str>, b: Option<&str>| -> bool {
        let (Some(a), Some(b)) = (a, b) else { return false };
        if !job.clock_groups.is_empty() {
            return crate::sdc::clocks_unrelated(&job.clock_groups, a, b);
        }
        match (group_of_name.get(a), group_of_name.get(b)) {
            (Some(x), Some(y)) => x != y,
            _ => false,
        }
    };
    // a clock's period by name (a virtual clock's too); the primary period for none
    let period_named = |c: &str| job.clocks.iter().find(|k| k.0 == c).map_or(period, |k| k.2);
    // The clock every I/O delay is relative to here (the job keeps no per-delay clock), and the
    // path group an endpoint is reported in: `asynchronous` for recovery and removal, else the
    // capture clock's name (`PathGroups::pathGroup`).
    let primary_clock = job.clocks.first().map_or(job.clock_port.as_str(), |c| c.0.as_str());
    let path_group = |v: usize, cap: Option<usize>| -> String {
        if async_end.contains(&v) {
            "asynchronous".to_string()
        } else if gating_end.contains(&v) {
            "gated clock".to_string()
        } else {
            cap.and_then(clock_name_of).unwrap_or(primary_clock).to_string()
        }
    };
    // (setup, hold) launch→capture relation between a launch edge (`ll`: lane 0 rise at t=0,
    // 1 fall at half the period) of a clock of period `pl` and a capture edge (`cl`) of period
    // `pc`, as the reference's cycle accounting finds it (`cycle_accting`): setup is the time
    // from the launch to the capture edge it must meet, hold the capture edge's time RELATIVE
    // to the launch (≤ 0). Same clock, same edge → (pc, 0); a falling launch captured on the
    // rise → (pc/2, −pc/2).
    //
    // The edge times are the clocks' own (`ClockEdge::time`, the waveform `create_clock
    // -waveform` or `create_generated_clock` gave it, [`clock_edge_time`]); a clock named by none
    // (a job file's default) rises at 0 and falls at half its period.
    let edge_relation = |pl: f64, lclk: Option<&str>, ll: usize, pc: f64, cclk: Option<&str>, cl: usize| -> (f64, f64) {
        let (setup, hold) = cycle_accting(pl, clock_edge_time(job, lclk, ll, pl), pc, clock_edge_time(job, cclk, cl, pc));
        (setup, -hold)
    };
    // The hold relation of check end `v`: a clock gating hold is in its setup check's cycle
    // (`cycle_accting_gated`), every other hold `edge_relation`'s.
    let hold_relation = |v: usize, pl: f64, lclk: Option<&str>, ll: usize, pc: f64, cclk: Option<&str>, cl: usize| -> f64 {
        if gating_end.contains(&v) {
            let ct = |e: usize| clock_edge_time(job, cclk, e, pc);
            -cycle_accting_gated(pl, clock_edge_time(job, lclk, ll, pl), pc, ct(cl), ct(1 - cl)).2
        } else {
            edge_relation(pl, lclk, ll, pc, cclk, cl).1
        }
    };

    // worst (max) constraint over a pin's groups, interpolated at the operating
    // clock + data transitions — matches how delay arcs are looked up.

    // Timing exceptions, matched BY OBJECT KIND (`crate::exc`): a clock against the path's
    // launching / capturing clock, an instance or port against its start / end object. Matched
    // on names alone, `-from [get_clocks a]` compared the clock's name with register names and
    // never applied.
    let inst_of = |node: usize| instance_of(&labels[node]).to_string();
    let exc_clocks: Vec<&str> = job.clocks.iter().map(|c| c.0.as_str()).collect();
    let exc_matchers = crate::exc::matchers(&job.exceptions, &exc_clocks);
    // A path end as an exception sees it: a register by its instance and the clock reaching its
    // clock pin; a port (`port`) by its name and the clock its I/O delay is relative to here.
    struct End {
        name: String,
        clock: Option<String>,
        port: bool,
    }
    let reg_end = |ck: Option<usize>, node: usize| End { name: inst_of(node), clock: ck.and_then(clock_name_of).map(str::to_string), port: false };
    // The start of the path into `endpoint` over `pred`: its launching register (`lck`), else
    // the input port it starts at, timed against the primary clock. Without the port's name a
    // `-from [get_ports din]` matched no path at all.
    let mut is_in_port = vec![false; n];
    for (i, _) in &input_ports {
        is_in_port[*i] = true;
    }
    // the input port a path into `endpoint` over `pred` starts at, when it starts at one
    let start_port = |endpoint: usize, pred: &[Option<usize>]| -> Option<usize> {
        let mut v = endpoint;
        while let Some(u) = pred[v] {
            v = u;
        }
        is_in_port[v].then_some(v)
    };
    // the clock an input port's delays on launching edge `edge` are relative to, if one is named
    let in_clock_at = |port: Option<usize>, edge: usize| -> Option<&str> { port.and_then(|p| in_clock.get(&p)?[edge].as_deref()) };
    // the launch period of the path from an input port on edge `edge`: its delay's clock's
    let in_period = |endpoint: usize, pred: &[Option<usize>], edge: usize| in_clock_at(start_port(endpoint, pred), edge).map(period_named);
    let launch_end = |lck: Option<usize>, endpoint: usize, pred: &[Option<usize>], edge: usize| match lck {
        Some(ck) => reg_end(Some(ck), ck),
        None => {
            let p = start_port(endpoint, pred);
            let name = p.map(|v| labels[v].clone()).unwrap_or_default();
            End { name, clock: Some(in_clock_at(p, edge).unwrap_or(primary_clock).to_string()), port: true }
        }
    };
    fn point(e: &End) -> crate::exc::PathPoint<'_> {
        crate::exc::PathPoint { name: e.name.as_str(), clock: e.clock.as_deref(), port: e.port }
    }
    // How far the check (`hold`) on the path `from` -> `to` moves its capture edge, `None` when
    // a false path drops it — by the reference's exception priority, not file order.
    let exc_shift = |hold: bool, from: &End, to: &End, pl: f64, pc: f64| {
        crate::exc::check_shift(&exc_matchers, hold, &point(from), &point(to), pl, pc)
    };
    let mut excluded_setup = vec![false; n]; // false-path endpoints (skip setup)
    let mut check_arr = vec![f64::NAN; n]; // see `Timing::check_arr`
    // the reference's rule, where the job asks for it: an output with no output delay ends no path
    if job.unconstrained_outputs {
        for (idx, name) in &output_ports {
            if !job.output_delay_declared_for(name) {
                excluded_setup[*idx] = true;
            }
        }
    }
    // An output port's delays, one per clock edge (`OutputDelay`), resolved once.
    let out_delays: HashMap<usize, Vec<crate::job::PortDelay>> =
        output_ports.iter().map(|(i, name)| (*i, job.output_delays_for(name))).collect();
    // An output port's check per data edge, for the path from `from` launched on edge `edge`
    // (launch period `pl`), setup (`max`) or hold: over its delays, the tightest requirement
    // `(required, capture clock, capture edge, relation)` — the reference checks each delay's
    // clock edge as its own path end (`PathEndOutputDelay`), and the worst of them is the slack.
    // The relation is the launch edge to that clock edge's capture (`CycleAccting`), moved by any
    // exception; a false path, or clocks in separate groups, leave that delay unchecked. `None`
    // on a data edge no delay sets.
    type OutCheck = [Option<(f64, String, usize, f64)>; 2];
    let out_check = |idx: usize, from: &End, edge: usize, pl: f64, max: bool| -> OutCheck {
        let mut best: OutCheck = [None, None];
        let name = labels[idx].as_str();
        for d in out_delays.get(&idx).map(Vec::as_slice).unwrap_or_default() {
            let clk = d.clock.clone().unwrap_or_else(|| primary_clock.to_string());
            // clocks in separate groups: no check (a delay with no clock names none)
            if d.clock.is_some() && is_async_named(from.clock.as_deref(), Some(&clk)) {
                continue;
            }
            let cl = usize::from(d.clock_fall);
            // a delay with no clock is relative to the job's primary period
            let pc = d.clock.as_deref().map_or(period, period_named);
            let to = End { name: name.to_string(), clock: Some(clk.clone()), port: true };
            let Some(shift) = exc_shift(!max, from, &to, pl, pc) else { continue };
            // a delay with no clock (a job file's default) keeps this timer's rule: setup from a
            // launch of the primary period itself, hold from the launch clock's
            let pl_setup = if d.clock.is_none() { period } else { pl };
            let lclk = d.clock.as_ref().and(from.clock.as_deref());
            let setup_rel = edge_relation(pl_setup, lclk, edge, pc, d.clock.as_deref(), cl).0;
            let hold_rel = edge_relation(pl, lclk, edge, pc, d.clock.as_deref(), cl).1;
            let rel = if max { setup_rel } else { hold_rel } + shift;
            for (l, slot) in best.iter_mut().enumerate() {
                let Some(od) = d.value[usize::from(max)][l] else { continue };
                let req = if max { rel - od - unc(Some(&clk), true) } else { rel + unc(Some(&clk), false) - od };
                if slot.as_ref().is_none_or(|b| if max { req < b.0 } else { req > b.0 }) {
                    *slot = Some((req, clk.clone(), cl, rel));
                }
            }
        }
        best
    };
    // the launch period of the path into `v` over `pred` on edge `edge`: its register's clock,
    // else its input delay's clock, else `default`
    let launch_period = |lck: Option<usize>, v: usize, pred: &[Option<usize>], edge: usize, default: f64| {
        lck.map(clock_period_of).or_else(|| in_period(v, pred, edge)).unwrap_or(default)
    };
    // The clock slew a check's margin is looked up at: the OTHER analysis's slew at the clock pin,
    // the early one for a setup (recovery) check and the late one for hold (removal). The
    // reference computes check arcs with the clock pin's slew of `Scene::checkClkSlewMinMax`,
    // which under its default on-chip-variation analysis is the opposite of the check's own min /
    // max; its report shows it, the setup check's capture clock path printing the min slews. Where
    // the two analyses load a clock pin alike (no capacitance ranges, no derate) they agree.
    let check_clk_slew = |cap: Option<usize>, cl: usize, setup: bool| -> f64 {
        cap.map(|i| if setup { early_slew[i][cl] } else { late_slew[i][cl] }).unwrap_or(input_slew)
    };
    // A latch opens on the edge before the one its setup check is against (`cl`): the enable
    // edge, a pulse width earlier (`CycleAccting`, latch setup role: the disable edge is the first
    // after the launch, the enable the one before it). Its time as the check sees it carries the
    // enable edge's own early latency and CRPR and the setup uncertainty (`Latches::latchRequired`
    // / `latchBorrowInfo`). `rel_close`: the launch-to-disable relation, exceptions included.
    // Returns (opening time, enable edge time from the launch, open latency, open CRPR).
    // `set_max_time_borrow`: the borrow limit of a latch data pin, as the reference looks it up
    // (`Sdc::latchBorrowLimit`): the data pin's, else the enable pin's, else the latch
    // instance's, else the enable clock's; the last command naming an object wins. A limit
    // replaces the window's own maximum (pulse width less latency, CRPR and setup differences),
    // larger or smaller.
    let (pin_borrow, inst_borrow, clk_borrow) = {
        let mut pins: HashMap<usize, f64> = HashMap::new();
        let mut insts: HashMap<String, f64> = HashMap::new();
        // (clock name pattern, limit), matched against the enable's clock when looked up
        let mut clks: Vec<(String, f64)> = Vec::new();
        let inst_names: Vec<&str> = nl.insts.iter().map(|i| i.name.as_str()).collect();
        let inst_pins: Vec<(usize, &str)> = nl.insts.iter().enumerate().flat_map(|(k, i)| i.conns.iter().map(move |(p, _)| (k, p.as_str()))).collect();
        let mut warn = Vec::new();
        for c in &job.max_time_borrow {
            let Some(limit) = c.number(0) else {
                warn.push("set_max_time_borrow: no limit; not applied".to_string());
                continue;
            };
            let Some(crate::sdc::KeptArg::Objects(o)) = c.args.get(1) else {
                warn.push(format!("set_max_time_borrow {limit}: objects not named by get_pins / get_cells / get_clocks; not applied"));
                continue;
            };
            for name in &o.names {
                match o.accessor.as_str() {
                    "get_pins" => {
                        for j in vyges_loom::sdc::pins_matching(name, &inst_names, &inst_pins) {
                            if let Some(&i) = key2idx.get(&pin_key(inst_names[inst_pins[j].0], inst_pins[j].1)) {
                                pins.insert(i, limit);
                            }
                        }
                    }
                    "get_cells" => {
                        for i in inst_names.iter().filter(|i| vyges_loom::sdc::object_matches(name, i)) {
                            insts.insert(i.to_string(), limit);
                        }
                    }
                    "get_clocks" => clks.push((name.clone(), limit)),
                    a => warn.push(format!("set_max_time_borrow {limit} [{a} {name}]: not applied")),
                }
            }
        }
        for m in &warn {
            vyges_events::emit(&vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, m.clone()).with_code("STA-SDC-NOT-APPLIED"));
        }
        (pins, insts, clks)
    };
    let borrow_limit = |d: usize, en: Option<usize>, clk: Option<&str>| -> Option<f64> {
        pin_borrow
            .get(&d)
            .or_else(|| en.and_then(|e| pin_borrow.get(&e)))
            .or_else(|| inst_borrow.get(instance_of(&labels[d])))
            .copied()
            // the enable's clock, named as its path group names it
            .or_else(|| {
                let c = clk.unwrap_or(primary_clock);
                clk_borrow.iter().rev().find(|(p, _)| vyges_loom::sdc::object_matches(p, c)).map(|(_, l)| *l)
            })
    };
    let latch_opening = |cap: Option<usize>, lck: Option<usize>, cclk: Option<&str>, pc: f64, cl: usize, rel_close: f64| -> (f64, f64, f64, f64) {
        let el = 1 - cl;
        let high = clock_edge_time(job, cclk, 1, pc) - clock_edge_time(job, cclk, 0, pc);
        // the opening's clock edge (`src_edge`); `el` is the enable pin's transition
        let pulse_width = if src_edge(cap, el) == 0 { high } else { pc - high };
        let open_latency = cap.map(|i| early_arr[i][el]).filter(|a| a.is_finite()).unwrap_or(0.0);
        let open_crpr = match (lck, cap) {
            (Some(l), Some(c)) => crpr_credit(l, c, el, true),
            _ => 0.0,
        };
        let rel_open = rel_close - pulse_width;
        (open_latency + rel_open + open_crpr - unc(cclk, true), rel_open, open_latency, open_crpr)
    };
    // ---- latch D->Q: what passes through a transparent latch -------------------------------------
    // The reference's arrival search postpones every latch output and, after each pass, takes
    // its D->Q arc (`Search::findAllArrivals` -> `ArrivalVisitor` latch branch,
    // `Latches::latchOutArrival`), re-pending a latch whose D moved, until nothing does. Only on
    // the max analysis. The latch's enable state decides: data arriving while the latch is open
    // (it borrows, `latchRequired`) passes to Q at the time it is required — its own arrival, or
    // the opening plus the maximum borrow — plus the D->Q delay; it then carries the ENABLE
    // clock edge's tag, so downstream it is launched by the latch's enable edge. Data that
    // arrives before the opening borrows nothing and leaves Q at the enable's own G->Q time.
    //
    // Here: the latch window of each latch data pin on each launching edge's pass, the arrival
    // that leaves its Q in the enable edge's frame, and the late passes again with those seeds,
    // until no seed moves. The delays and slews are the converged ones (`fixed_slew`): the
    // reference merges no D->Q slew into Q (`annotateDelaySlew` skips the latch D->Q role) and
    // computes the D->Q delay at D's slew after the rest (`findLatchEdgeDelays`).
    // Does the path to `v`'s edge `l` on `run` leave a latch through its D->Q arc (`seeds`: what
    // its pass seeded the latch outputs with)? Its first launching pin is then the latch's enable
    // and the latch output's arrival is the seed's.
    //
    // ⛔ SUCH A PATH HAS NO CRPR. The reference hands the data passing a latch the enable's clock
    // information with the enable path as its CRPR clock path (`latchOutArrival`), and that
    // enable path is the target clock's, the EARLY one; a setup check downstream compares it with
    // its early capture path, and the pessimism between two early paths is nothing. Measured on
    // an ASAP7 latch chain: open_crpr 3.386 ps on the enable's own launch, 0 through D->Q.
    let via_latch = |run: &EdgeView, seeds: &[HashMap<usize, LatchSeed>], v: usize, l: usize| -> bool {
        let Some(sd) = seeds.get(run.edge).filter(|m| !m.is_empty()) else { return false };
        let (mut x, mut xl) = (v, l);
        while let Some((u, ul)) = run.late_from[x][xl] {
            if is_ck[u] {
                return sd.get(&x).is_some_and(|s| s.from[xl] == Some((u, ul)) && s.arr[xl] == run.late_arr[x][xl]);
            }
            (x, xl) = (u, ul);
        }
        false
    };
    // (latch window per data edge, as the search clips a borrow: (opening, closing, enable edge
    // time from the launch, enable lane))
    let latch_window = |run: &EdgeView, seeds: &[HashMap<usize, LatchSeed>], idx: usize, setup: &[Constraint], ck: &Option<String>| -> [Option<(f64, f64, f64, usize)>; 2] {
        let mut out = [None; 2];
        let cap = ck_node(ck);
        if unclocked_ck(cap) {
            return out;
        }
        let pc = cap.map(clock_period_of).unwrap_or(period);
        let lck = launch_ck(idx, run.from);
        let pl = launch_period(lck, idx, run.from, run.edge, pc);
        let (from_end, to_end) = (launch_end(lck, idx, run.from, run.edge), reg_end(cap, idx));
        let Some(shift) = exc_shift(false, &from_end, &to_end, pl, pc) else {
            return out;
        };
        let in_clk = lck.map_or_else(|| in_clock_at(start_port(idx, run.from), run.edge), |_| None);
        if is_async(lck, cap) || is_async_named(in_clk, to_end.clock.as_deref()) {
            return out;
        }
        for (l, o) in out.iter_mut().enumerate() {
            let mut best = f64::INFINITY;
            // the launching clock pin CRPR is taken against: none through a latch's D->Q
            let crpr_lck = lck.filter(|_| !via_latch(run, seeds, idx, l));
            for c in setup.iter().filter(|c| checks_edge(c, l == 0)) {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                let crpr = match (crpr_lck, cap) {
                    (Some(a), Some(b)) => crpr_credit(a, b, cl, true),
                    _ => 0.0,
                };
                let rel_close = edge_relation(pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)).0 + shift;
                // ⛔ The setup time the SEARCH clips a latch's borrow with is not the one its check
                // reports. The search takes the check arc's delay on the enable path's analysis
                // (`Latches::latchSetupMargin`: the disable path's own delay-calculation point,
                // the early one for a max path), so it is looked up at D's EARLY slew and that
                // analysis's check clock slew, the late one; the check itself
                // (`PathEndCheck::margin`) is at the data path's, D's late slew. 0.6 ps apart on
                // an ASAP7 latch, and the downstream register's slack moves with it.
                let margin = c.eval_edge(l == 0, check_clk_slew(cap, cl, false), early_slew[idx][l]);
                let close = cap_early + rel_close + crpr - unc(to_end.clock.as_deref(), true) - margin;
                if close < best {
                    best = close;
                    let (open, rel_open, _, _) = latch_opening(cap, crpr_lck, to_end.clock.as_deref(), pc, cl, rel_close);
                    let close = borrow_limit(idx, cap, to_end.clock.as_deref()).map_or(close, |lim| open + lim);
                    *o = Some((open, close, rel_open, src_edge(cap, 1 - cl)));
                }
            }
        }
        out
    };
    // (data pin, its setup checks, its enable pin key, Q, the D->Q arc, the lanes it passes)
    type LatchArc<'a> = (usize, &'a [Constraint], &'a Option<String>, usize, &'a Arc, [[bool; 2]; 2]);
    let latch_arcs: Vec<LatchArc<'_>> = flop_d
        .iter()
        .filter(|(idx, _, _)| latch_d.contains(idx))
        .flat_map(|(idx, setup, ck)| {
            out_edges[*idx].iter().filter(|e| e.latch_dq).filter_map(move |e| match &e.kind {
                EdgeKind::Cell(arc) => Some((*idx, &**setup, ck, e.to, *arc, e.lanes)),
                EdgeKind::Net(_) => None,
            })
        })
        .collect();
    // The D->Q delay (late) from data edge `il` to Q edge `ol`, at D's slew and Q's load.
    // ⚠️ Lumped load, or the Ceff Q's own arcs were solved at under SPEF: the reference reduces
    // the D->Q arc's load on its own.
    let dq_delay = |d: usize, q: usize, arc: &Arc, il: usize, ol: usize| -> f64 {
        let (dt, st) = if ol == 0 { (&arc.cell_rise, &arc.rise_transition) } else { (&arc.cell_fall, &arc.fall_transition) };
        let sin = late_slew[d][il];
        let load = if shield[q][1][ol].is_some() { ceff_used.borrow()[q][ol] } else { node_load[q][1][ol] };
        let dly = if arc.ccs.is_empty() {
            dt.lookup(sin, load)
        } else {
            arc.ccs.delay_slew(ol == 0, sin, load, 0.3, 0.7).map_or_else(|| dt.lookup(sin, load), |r| r.0)
        };
        let _ = st;
        dly * cell_derate(true, 1)
    };
    // the seeds each launching edge's pass gives the latch outputs it launches, from every pass's
    // data at their D pins
    let latch_seeds = |views: &[EdgeView], prev: &[HashMap<usize, LatchSeed>]| -> Vec<HashMap<usize, LatchSeed>> {
        let mut seeds: Vec<HashMap<usize, LatchSeed>> = vec![HashMap::new(); 2];
        for run in views {
            for &(d, setup, ck, q, arc, lanes) in &latch_arcs {
                let Some(en) = ck_node(ck) else { continue };
                for (il, w) in latch_window(run, prev, d, setup, ck).into_iter().enumerate() {
                    let Some((open, close, rel_open, el)) = w else { continue };
                    let a = run.late_arr[d][il];
                    if !a.is_finite() || a <= open {
                        continue; // closed when it arrives, or never: Q is its enable's
                    }
                    let required = open + (a - open).min(close - open);
                    for ol in 0..2 {
                        if !arc.out_edges.drives(ol == 0) || !arc.feeds(il == 0, ol == 0) || !lanes[il][ol] {
                            continue;
                        }
                        let qa = required - rel_open + dq_delay(d, q, arc, il, ol);
                        let sd = seeds[el].entry(q).or_insert(LatchSeed { arr: [f64::NEG_INFINITY; 2], from: [None; 2] });
                        if qa > sd.arr[ol] {
                            sd.arr[ol] = qa;
                            sd.from[ol] = Some((en, el));
                        }
                    }
                }
            }
        }
        seeds
    };
    let mut latch_iters = 0usize;
    let (arrival, from, order_late, late_arr, late_from, edge_runs, latch_seeds_final) = {
        // the converged passes stay borrowed by the clock-path closures above (their clock pins are
        // the same in every pass: nothing crosses into the clock network from a latch output)
        let (mut arrival, mut from, mut order_late, mut late_arr, mut late_from, mut edge_runs) =
            (arrival.clone(), from.clone(), order_late, late_arr.clone(), late_from.clone(), edge_runs);
        let mut prev: Vec<HashMap<usize, LatchSeed>> = vec![HashMap::new(); 2];
        const MAX_LATCH_PASSES: usize = 64;
        while !latch_arcs.is_empty() && latch_iters < MAX_LATCH_PASSES {
            let views: Vec<EdgeView> = if edge_runs.is_empty() {
                vec![EdgeView { edge: 0, late_arr: &late_arr, late_from: &late_from, from: &from, early_arr: &early_arr, early_from: &early_from, from_min: &from_min }]
            } else {
                edge_runs.iter().map(EdgeRun::view).collect()
            };
            let seeds = latch_seeds(&views, &prev);
            drop(views);
            let same = seeds.iter().zip(&prev).all(|(a, b)| {
                a.len() == b.len()
                    && a.iter().all(|(k, x)| b.get(k).is_some_and(|y| x.from == y.from && (0..2).all(|l| x.arr[l] == y.arr[l] || (x.arr[l] - y.arr[l]).abs() < 1e-9)))
            });
            if same {
                break;
            }
            latch_iters += 1;
            // the merged pass: every edge's launches, each from its own edge
            let mut merged = seeds[0].clone();
            for (q, sd) in &seeds[1] {
                let m = merged.entry(*q).or_insert(*sd);
                for l in 0..2 {
                    if sd.arr[l] > m.arr[l] {
                        m.arr[l] = sd.arr[l];
                        m.from[l] = sd.from[l];
                    }
                }
            }
            let (a, _s, f, o, la, _ls, lf) = relax(&arc_d, &arc_s, true, &seed_main, false, None, Some(&late_slew), Some(&merged));
            (arrival, from, order_late, late_arr, late_from) = (a, f, o, la, lf);
            for er in edge_runs.iter_mut() {
                let sd = edge_seed(er.edge, 1);
                let (_, _, f, _, la, _, lf) = relax(&arc_d, &arc_s, true, &sd, false, Some(er.edge), Some(&late_slew), Some(&seeds[er.edge]));
                (er.from, er.late_arr, er.late_from) = (f, la, lf);
            }
            prev = seeds;
        }
        if latch_iters == MAX_LATCH_PASSES {
            vyges_events::emit(&vyges_events::Event::new(
                "vyges-sta-si",
                vyges_events::Severity::Warn,
                format!("latch arrivals did not settle in {MAX_LATCH_PASSES} passes; the last pass's are used"),
            ).with_code("STA-LATCH-PASSES"));
        }
        (arrival, from, order_late, late_arr, late_from, edge_runs, prev)
    };
    // every launching edge as a view: the one merged pass when all launch on the rise
    let runs: Vec<EdgeView> = if edge_runs.is_empty() {
        vec![EdgeView { edge: 0, late_arr: &late_arr, late_from: &late_from, from: &from, early_arr: &early_arr, early_from: &early_from, from_min: &from_min }]
    } else {
        edge_runs.iter().map(EdgeRun::view).collect()
    };
    // Required time PER LAUNCHING EDGE AND DATA EDGE: `req_run[r][v][l]` against `runs[r]`'s
    // arrivals; NaN = no check from that edge. `endpoint_req` keeps the tightest so the query API
    // still answers with one number; the scoring loop below uses them all.
    let mut req_run = vec![vec![[f64::NAN; 2]; n]; runs.len()];
    // An output port: scored per data edge like a register, so its reported path is the edge
    // that set the slack, followed lane by lane. Each of its delays is a check of its own clock
    // and edge; an exception reaches it as a register's.
    // `out_setup_clock[(v, r)]`: the clock of the delay that set each data edge's requirement.
    let mut out_setup_clock: HashMap<(usize, usize), [Option<String>; 2]> = HashMap::new();
    for (idx, _) in &output_ports {
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(*idx, run.from);
            let pl = launch_period(lck, *idx, run.from, run.edge, period);
            let chk = out_check(*idx, &launch_end(lck, *idx, run.from, run.edge), run.edge, pl, true);
            for (l, c) in chk.iter().enumerate() {
                if let Some((req, _, _, _)) = c {
                    req_run[r][*idx][l] = *req;
                }
            }
            out_setup_clock.insert((*idx, r), chk.map(|c| c.map(|c| c.1)));
        }
        // the collapsed requirement: the rising launch's (the merged arrival is measured from
        // it), else any launch's
        let tightest = |q: &[[f64; 2]]| q[*idx].iter().copied().filter(|x| !x.is_nan()).fold(f64::INFINITY, f64::min);
        let t = match tightest(&req_run[0]) {
            t if t.is_finite() => t,
            _ => req_run.iter().map(|q| tightest(q)).fold(f64::INFINITY, f64::min),
        };
        if t.is_finite() {
            endpoint_req[*idx] = t;
        } else {
            excluded_setup[*idx] = true;
        }
    }

    // The launches a pin keeps where CRPR tells them apart (see `crpr_cands`, and the hold side
    // below): each launch's clock pin spread (`CheckCrpr::maxCrpr`) prunes the others.
    let start_spread = |k: usize| -> f64 {
        if !is_ck[k] {
            return 0.0;
        }
        let l = ck_lane[k].unwrap_or(0);
        ((late_arr[k][l] as f32) - (early_arr[k][l] as f32)).abs() as f64
    };
    let any_spread = crpr_on && (0..n).any(|k| is_ck[k] && start_spread(k) > 0.0);
    let skip_from = |u: usize| clock_declared && clock_reached[u] && !is_ck[u];
    // where an edge's predecessors lead back to: the first clock pin, else the start
    let chain_start = |from: &[[Pred; 2]], v: usize, l: usize| -> Option<usize> {
        let (mut x, mut xl) = (v, l);
        while let Some((u, ul)) = from[x][xl] {
            if is_ck[u] {
                return Some(u);
            }
            (x, xl) = (u, ul);
        }
        Some(x)
    };
    // ⛔ SETUP, PER LAUNCH WHERE CRPR TELLS THEM APART, the mirror of the hold side: the merged
    // late arrival is the LATEST launch's, scored with its CRPR credit; an earlier launch whose
    // clock path shares less with the capture's gets less credit and can be the worse setup. A
    // register's own Q->D loop is the common case: its credit is its whole clock path.
    let mut setup_keep = vec![false; n];
    for (idx, _, _) in &flop_d {
        setup_keep[*idx] = !latch_d.contains(idx);
    }
    let setup_cands: Vec<HashMap<usize, [Vec<Cand>; 2]>> = if any_spread && track_setup && late_recs.len() == runs.len() {
        runs.iter()
            .zip(&late_recs)
            .map(|(run, recs)| crpr_cands(recs, run.late_arr, &is_ck, &skip_from, &start_spread, true, &setup_keep, None).0)
            .collect()
    } else {
        Vec::new()
    };
    let simple_ctx = simple_ctx && setup_cands.iter().all(|m| m.values().all(|c| c.iter().all(|s| s.len() < 2)));
    // per (run, end, data edge) where launches were told apart: (slack, arrival, launch pin)
    let mut setup_cand_slack: HashMap<(usize, usize, usize), (f64, f64, usize)> = HashMap::new();
    // a setup end whose worst launch its edge's predecessors do not reach: (run, data edge, launch)
    let mut setup_pick: HashMap<usize, (usize, usize, usize)> = HashMap::new();
    // A latch data pin's check also needs the time the latch OPENS, per launching edge and data
    // edge (`latch_open[(r, v)][l]`, NaN = none): `req_run` holds the time it closes.
    let mut latch_open: HashMap<(usize, usize), [f64; 2]> = HashMap::new();
    // setup capture uses the EARLY clock; CRPR adds back the shared-path pessimism;
    // the capture window is the launch→capture edge relation (one period intra-domain),
    // shifted out by a multicycle exception (false paths drop the endpoint).
    for (idx, setup, ck) in &flop_d {
        let cap = ck_node(ck);
        // No declared clock reaches this capture register, so it has no reference to
        // be checked against: OpenSTA builds no PathEndCheck here at all. Scoring it
        // against the data arrival that happens to sit on its CK pin invents a check.
        // Covers RECOVERY too — `flop_d` carries both setup and recovery constraints.
        if unclocked_ck(cap) {
            excluded_setup[*idx] = true;
            continue;
        }
        let pc = cap.map(clock_period_of).unwrap_or(period);
        let mut checked = false;
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(*idx, run.from);
            let pl = launch_period(lck, *idx, run.from, run.edge, pc);
            // A `-hold` multicycle or false path does not reach the SETUP check, and `-start`
            // counts the launch clock's cycles where the default `-end` counts the capture clock's.
            let (from_end, to_end) = (launch_end(lck, *idx, run.from, run.edge), reg_end(cap, *idx));
            let Some(shift) = exc_shift(false, &from_end, &to_end, pl, pc) else {
                continue;
            };
            let in_clk = lck.map_or_else(|| in_clock_at(start_port(*idx, run.from), run.edge), |_| None);
            if is_async(lck, cap) || is_async_named(in_clk, to_end.clock.as_deref()) {
                continue; // async clock groups: no setup check
            }
            checked = true;
            // Each check at ITS capture edge (`setup_rising` / `_falling`): that edge's clock
            // arrival and slew, and the relation from this run's launch edge to it.
            let base_of = |c: &Constraint, via: bool| -> (f64, f64) {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                let ck_slew = check_clk_slew(cap, cl, true);
                let crpr = match (lck.filter(|_| !via), cap) {
                    (Some(l), Some(c)) => crpr_credit(l, c, cl, true),
                    _ => 0.0,
                };
                (cap_early + edge_relation(pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)).0 + shift + crpr - unc(to_end.clock.as_deref(), true), ck_slew)
            };
            let open_of = |c: &Constraint, via: bool| -> f64 {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let cclk = to_end.clock.as_deref();
                let rel_close = edge_relation(pl, from_end.clock.as_deref(), run.edge, pc, cclk, src_edge(cap, cl)).0 + shift;
                latch_opening(cap, lck.filter(|_| !via), cclk, pc, cl, rel_close).0
            };
            let is_latch = latch_d.contains(idx);
            // ⛔ PER DATA EDGE, for the same reason as hold: a check exists for one edge, so
            // the latest arrival must meet THAT edge's setup constraint, not the worse of the
            // two.
            let mut open = [f64::NAN; 2];
            for (l, req) in req_run[r][*idx].iter_mut().enumerate() {
                *req = f64::INFINITY;
                // a path through a latch's D->Q carries no CRPR (see `via_latch`)
                let via = via_latch(run, &latch_seeds_final, *idx, l);
                for c in setup.iter().filter(|c| checks_edge(c, l == 0)) {
                    let (base, ck_slew) = base_of(c, via);
                    let q = base - c.eval_edge(l == 0, ck_slew, late_slew[*idx][l]);
                    if q < *req {
                        *req = q;
                        if is_latch {
                            open[l] = open_of(c, via);
                        }
                    }
                }
            }
            // each launch this end keeps on each data edge, checked with its own CRPR credit, period
            // and exceptions; the latest one's arrival as the pass found it
            if let Some(cands) = setup_cands.get(r).and_then(|m| m.get(idx)).filter(|_| !is_latch) {
                for (l, set) in cands.iter().enumerate() {
                    let a_m = run.late_arr[*idx][l];
                    if set.is_empty() || !a_m.is_finite() {
                        continue;
                    }
                    let last = set.iter().map(|e| e.0).fold(f64::NEG_INFINITY, f64::max);
                    let mut best: Option<(f64, f64, usize)> = None;
                    for &(a, k) in set {
                        let k = k as usize;
                        let a = if a == last { a_m } else { a };
                        let (lck_c, sport) = if is_ck[k] { (Some(k), None) } else { (None, is_in_port[k].then_some(k)) };
                        let pl_c = lck_c.map(clock_period_of).or_else(|| in_clock_at(sport, run.edge).map(period_named)).unwrap_or(pc);
                        let from_c = match lck_c {
                            Some(c) => reg_end(Some(c), c),
                            None => End {
                                name: sport.map(|v| labels[v].clone()).unwrap_or_default(),
                                clock: Some(in_clock_at(sport, run.edge).unwrap_or(primary_clock).to_string()),
                                port: true,
                            },
                        };
                        let Some(shift_c) = exc_shift(false, &from_c, &to_end, pl_c, pc) else {
                            continue;
                        };
                        let in_clk_c = lck_c.map_or_else(|| in_clock_at(sport, run.edge), |_| None);
                        if is_async(lck_c, cap) || is_async_named(in_clk_c, to_end.clock.as_deref()) {
                            continue;
                        }
                        for c in setup.iter().filter(|c| checks_edge(c, l == 0)) {
                            let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                            let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                            let ck_slew = check_clk_slew(cap, cl, true);
                            let crpr = match (lck_c, cap) {
                                (Some(lc), Some(cc)) => crpr_credit(lc, cc, cl, true),
                                _ => 0.0,
                            };
                            let rel = edge_relation(pl_c, from_c.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)).0 + shift_c;
                            let q = cap_early + rel + crpr - unc(to_end.clock.as_deref(), true) - c.eval_edge(l == 0, ck_slew, late_slew[*idx][l]);
                            if best.is_none_or(|b| q - a < b.0) {
                                best = Some((q - a, a, k));
                            }
                        }
                    }
                    if let Some(b) = best {
                        setup_cand_slack.insert((r, *idx, l), b);
                    }
                }
            }
            if is_latch {
                // a borrow limit closes the window at the opening plus the limit
                if let Some(lim) = borrow_limit(*idx, cap, to_end.clock.as_deref()) {
                    for (req, o) in req_run[r][*idx].iter_mut().zip(open) {
                        if !o.is_nan() {
                            *req = o + lim;
                        }
                    }
                }
                latch_open.insert((r, *idx), open);
            }
            // capture the clock-side constants so the fast path can re-derive required time
            // from the (possibly changed) data slew without re-walking the clock network.
            if simple_ctx {
                if let Some((base, ck_slew)) = setup.first().map(|c| base_of(c, false)) {
                    inc_setup.push(SetupRec {
                        idx: *idx,
                        base,
                        cons: setup.to_vec(),
                        ck_slew,
                        launch_ck: lck,
                    });
                }
            }
        }
        if !checked {
            excluded_setup[*idx] = true;
            continue;
        }
        endpoint_req[*idx] = req_run.iter().flat_map(|q| q[*idx]).filter(|x| !x.is_nan()).fold(f64::INFINITY, f64::min);
    }

    // ---- data checks (`set_data_check`, see `crate::data_check`) -------------------------------
    // Timed where the reference's terms are this timer's: one launching edge of one clock (the
    // from-pin's and the to-pin's paths then launch from the same edge, and the setup check is
    // zero-cycle on it) and no exceptions (which the reference matches to the check's end). Any
    // other case is refused, said, and not applied.
    let data_checks: Vec<(usize, usize, crate::data_check::Margins)> = {
        let mut w = Vec::new();
        // `[get_pins …]` is found as the reference's SDC network finds pins (loom `pins_matching`:
        // a `*` / `?` pattern, brackets escaped only when nothing matches as written); the key
        // names ONE pin (Error 128), so a pattern matching several, or none, is not applied.
        let inst_names: Vec<&str> = nl.insts.iter().map(|i| i.name.as_str()).collect();
        let inst_pins: Vec<(usize, &str)> = nl.insts.iter().enumerate().flat_map(|(k, i)| i.conns.iter().map(move |(p, _)| (k, p.as_str()))).collect();
        let node_of = |(acc, name): &(String, String)| -> Option<usize> {
            let port = key2idx.get(&port_key(name)).copied();
            let pin = split_inst_pin(name).and_then(|(i, p)| key2idx.get(&pin_key(i, p)).copied());
            match acc.as_str() {
                "get_ports" => port,
                "get_pins" => match vyges_loom::sdc::pins_matching(name, &inst_names, &inst_pins).as_slice() {
                    &[j] => key2idx.get(&pin_key(inst_names[inst_pins[j].0], inst_pins[j].1)).copied(),
                    _ => None,
                },
                _ => port.or(pin),
            }
        };
        let dcs = crate::data_check::data_checks(&job.data_check_cmds, &mut w, &node_of);
        let in_terms = runs.len() == 1 && job.clocks.len() <= 1 && job.exceptions.is_empty();
        let mut out = Vec::new();
        for d in dcs {
            if !in_terms {
                w.push(format!("{} -> {}: a data check is timed here for one launching edge of one clock and no exceptions; not applied", d.from.1, d.to.1));
                continue;
            }
            out.push((d.nodes.0, d.nodes.1, d.margins));
        }
        for m in &w {
            vyges_events::emit(
                &vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, format!("data check: {m}"))
                    .with_code("STA-DATA-CHECK"),
            );
        }
        out
    };
    // setup: zero-cycle, against the from-pin's EARLY arrival (`CycleAccting`, `PathEndDataCheck::
    // requiredTimeNoCrpr` with the data clock path at the target min/max)
    for (f, t, m) in &data_checks {
        let mut any = false;
        for tl in 0..2 {
            let q = (0..2)
                .filter_map(|fl| m[0][fl][tl].map(|mg| runs[0].early_arr[*f][fl] - unc(launch_ck(*f, runs[0].from).and_then(clock_name_of), true) - mg))
                .filter(|q| q.is_finite())
                .fold(f64::INFINITY, f64::min);
            if q.is_finite() {
                let cur = &mut req_run[0][*t][tl];
                *cur = if cur.is_nan() { q } else { cur.min(q) };
                any = true;
            }
        }
        if any {
            is_endpoint[*t] = true;
            endpoint_req[*t] = req_run[0][*t].iter().copied().filter(|x| !x.is_nan()).fold(f64::INFINITY, f64::min);
        }
    }

    // ---- setup slack + worst path ---------------------------------------
    // Each endpoint's required time is fixed (period at outputs, period - setup at
    // flop D), so slack = required - latest arrival; no backward pass needed.
    let mut wns = f64::INFINITY;
    let mut tns = 0.0;
    let mut worst = None;
    let mut worst_is_clk_launched = false;
    // the launching edge's run and the data edge whose check set the worst slack
    let mut worst_at: Option<(usize, usize)> = None;
    let mut endpoints = 0;
    let mut latch_borrow: Vec<(usize, f64)> = Vec::new();
    // each path group's worst setup end, for the report: (slack, endpoint, run, data edge)
    let setup_end: HashMap<usize, usize> = flop_d.iter().enumerate().map(|(e, (idx, _, _))| (*idx, e)).collect();
    let mut setup_group_worst: BTreeMap<String, (f64, usize, usize, usize)> = BTreeMap::new();
    // DIAGNOSTIC: `VYGES_STA_PROBE_END=<pin>` also reports that end's own setup and hold path, in
    // a path group of its own (`probe`), for a path-by-path comparison with the reference's
    // `report_checks -to`. Unset, nothing changes.
    let probe_end: Option<String> = std::env::var("VYGES_STA_PROBE_END").ok();
    let is_probe = |v: usize| probe_end.as_deref() == Some(labels[v].as_str());
    for v in 0..n {
        if !is_endpoint[v] || excluded_setup[v] {
            continue; // not an endpoint, or false-path-excluded
        }
        // Worst of the two launch models: a delay measured from the launch edge
        // (flops, input ports) and an absolute time carried from a clock source's own
        // edge. Whichever arrives later is the one this endpoint has to meet.
        let main_a = arrival[v];
        let clk_a = clk_arr[v];
        let (arr_v, clk_launched) = match (main_a.is_finite(), clk_a.is_finite()) {
            (true, true) if clk_a > main_a => (clk_a, true),
            (true, _) => (main_a, false),
            (false, true) => (clk_a, true),
            (false, false) => continue, // unreached by either
        };
        endpoints += 1;
        // per launching edge and data edge where we have it: each one's own arrival against
        // its own required. The clock-launched arrival is collapsed (it carries an absolute
        // edge time, not a per-lane delay), so it is scored against the tightest required.
        let mut slack = f64::INFINITY;
        let mut slack_at = None;
        // a latch end's: its borrow, of the check that set the slack
        let mut borrow = None;
        if !clk_launched {
            for (r, run) in runs.iter().enumerate() {
                let open = latch_open.get(&(r, v));
                for (l, (&q, &a)) in req_run[r][v].iter().zip(&run.late_arr[v]).enumerate() {
                    if q.is_nan() || !a.is_finite() {
                        continue;
                    }
                    let (s, b) = match open.map(|o| o[l]).filter(|o| !o.is_nan()) {
                        Some(o) => {
                            let (s, b) = latch_setup(o, q, a);
                            (s, Some(b))
                        }
                        // a launch told apart from the merged one by its CRPR credit: its own check
                        None => (setup_cand_slack.get(&(r, v, l)).map_or(q - a, |c| c.0), None),
                    };
                    // equal slacks of two latch checks: the larger borrow is the worse end
                    // (`PathEnd::cmpSlack`)
                    if s < slack || (s == slack && b.is_some_and(|b| borrow.is_some_and(|w| b > w))) {
                        slack = s;
                        slack_at = Some((r, l));
                        borrow = b;
                    }
                }
            }
        }
        if let Some(b) = borrow {
            latch_borrow.push((v, b));
        }
        // ⛔ No launching edge's check reached it: every path into it on the edge that carries
        // its arrival is cut (a false path), and no other edge's launch arrives. The reference has
        // no path end here (`VisitPathEnds`: no tag survives); scored with the merged arrival it
        // brought the cut path back as the worst slack.
        if !clk_launched && !slack.is_finite() {
            excluded_setup[v] = true;
            endpoints -= 1;
            continue;
        }
        if !slack.is_finite() {
            slack = endpoint_req[v] - arr_v;
        }
        // ⛔ The checked slack is per launching edge AND data edge; no one required time pairs
        // with the merged arrival (the latest over edges and launches), so the slack is folded
        // back into it, as the incremental path folds its per-edge slack — the query API's
        // `required − arrival` is then this endpoint's checked slack, the one the report and WNS
        // read. Folded only for several launching edges or a latch end, the query answered the
        // tightest required (one data edge) less the latest arrival (the other): an ECO consumer
        // ranked a pessimistic slack the report never showed.
        if let Some((r, l)) = slack_at {
            check_arr[v] = runs[r].late_arr[v][l];
            if let Some(&(_, a, k)) = setup_cand_slack.get(&(r, v, l)) {
                check_arr[v] = a;
                // the report follows this edge's own predecessors: traced only for a launch they
                // do not reach
                if Some(k) != chain_start(runs[r].late_from, v, l) {
                    setup_pick.insert(v, (r, l, k));
                }
            }
        }
        if slack_at.is_some() {
            endpoint_req[v] = arr_v + slack;
        }
        if let Some((r, l)) = slack_at {
            // an output's group is the clock of the delay that set its slack
            let out_group = out_setup_clock.get(&(v, r)).and_then(|c| c[l].clone());
            let g = out_group.unwrap_or_else(|| path_group(v, setup_end.get(&v).and_then(|&e| ck_node(&flop_d[e].2))));
            if is_probe(v) {
                setup_group_worst.insert("probe".to_string(), (slack, v, r, l));
            }
            if setup_group_worst.get(&g).is_none_or(|w| slack < w.0) {
                setup_group_worst.insert(g, (slack, v, r, l));
            }
        }
        if slack < 0.0 {
            tns += slack;
        }
        if slack < wns {
            wns = slack;
            worst = Some(v);
            worst_at = slack_at;
            worst_is_clk_launched = clk_launched;
        }
    }

    let mut worst_path = Vec::new();
    let worst_endpoint = match (worst, worst_at) {
        // scored per edge: that launching edge's and data edge's own path
        (Some(v), Some((r, l))) => {
            worst_path = crate::inc::lane_path(&labels, runs[r].late_arr, &late_slew, runs[r].late_from, v, l);
            labels[v].clone()
        }
        (Some(mut v), None) => {
            let end_label = labels[v].clone();
            // trace through the pass that produced the reported slack, so the printed
            // path is the one the number came from
            let (pred, arr_of, slew_of) = if worst_is_clk_launched {
                (&clk_from, &clk_arr, &clk_slew)
            } else {
                (&from, &arrival, &slew)
            };
            let mut chain = vec![v];
            while let Some(u) = pred[v] {
                chain.push(u);
                v = u;
            }
            chain.reverse();
            for idx in chain {
                worst_path.push(PathNode {
                    label: labels[idx].clone(),
                    arrival: arr_of[idx],
                    slew: slew_of[idx],
                });
            }
            end_label
        }
        (None, _) => String::new(),
    };

    // ---- path-based analysis (optional) ---------------------------------
    // Re-time the worst path and its 1-exchange fan-in alternatives with strictly
    // path-local slew (flat late derate; AOCV-depth not re-applied). GBA merges each
    // node's worst slew over all its arcs, so a path can carry a slew it never had; each
    // path re-timed with its own slews is what that pessimism costs.
    let pba_wns = if job.pba {
        let mut preds: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (u, edges) in out_edges.iter().enumerate() {
            for e in edges.iter().filter(|e| e.searched() && !e.latch_dq && !data_into_clock(u, e.to)) {
                preds[e.to].push(u);
            }
        }
        let cell_dly = |arc: &Arc, sin: f64, vnode: usize| -> (f64, f64) {
            // the collapsed path takes the worse edge, each edge at its own load
            let leff_of = |lane: usize| match shield[vnode][1][lane] {
                Some((c2, rpi, c1)) => crate::ccs::ceff_iter(c2, c1, rpi * c1 * r_scale, |c| {
                    arc.rise_transition
                        .lookup(sin, c)
                        .max(arc.fall_transition.lookup(sin, c))
                }),
                None => node_load[vnode][1][lane],
            };
            let leff = leff_of(0).max(leff_of(1));
            let (d, s) = if !arc.ccs.is_empty() {
                match (
                    arc.ccs.delay_slew(true, sin, leff, 0.3, 0.7),
                    arc.ccs.delay_slew(false, sin, leff, 0.3, 0.7),
                ) {
                    (Some(a), Some(b)) => (a.0.max(b.0), a.1.max(b.1)),
                    (Some(a), None) => a,
                    (None, Some(b)) => b,
                    _ => (0.0, sin),
                }
            } else {
                (
                    arc.cell_rise
                        .lookup(sin, leff)
                        .max(arc.cell_fall.lookup(sin, leff)),
                    arc.rise_transition
                        .lookup(sin, leff)
                        .max(arc.fall_transition.lookup(sin, leff)),
                )
            };
            (d * late_derate, s)
        };
        let retime = |path: &[usize]| -> f64 {
            let mut t = 0.0;
            let mut s = input_slew;
            for w in path.windows(2) {
                let (a, b) = (w[0], w[1]);
                let Some(e) = out_edges[a].iter().find(|e| e.to == b) else {
                    return f64::NEG_INFINITY;
                };
                match &e.kind {
                    EdgeKind::Net(i) => {
                        // ⚠️ The path-based walk carries a COLLAPSED path with no edge, so
                        // it takes the worse lane. Identical to the old single-valued
                        // behaviour wherever the two lanes agree.
                        t += arc_d[*i][0].max(arc_d[*i][1]);
                        let sk = arc_s[*i][0].max(arc_s[*i][1]);
                        if sk > 0.0 {
                            s = sk;
                        }
                    }
                    EdgeKind::Cell(arc) => {
                        let (d, so) = cell_dly(arc, s, b);
                        t += d;
                        s = so;
                    }
                }
            }
            t
        };
        // GBA-best prefix to a node (source-first)
        let prefix = |start: usize| -> Vec<usize> {
            let mut c = vec![start];
            let mut v = start;
            while let Some(u) = from[v] {
                c.push(u);
                v = u;
            }
            c.reverse();
            c
        };
        worst.map(|end| {
            let gba_path = prefix(end);
            let mut worst_arr = retime(&gba_path);
            for wi in 1..gba_path.len() {
                let node = gba_path[wi];
                let on_path = gba_path[wi - 1];
                for &alt in &preds[node] {
                    if alt == on_path {
                        continue;
                    }
                    let alt_path: Vec<usize> = prefix(alt)
                        .iter()
                        .chain(gba_path[wi..].iter())
                        .copied()
                        .collect();
                    worst_arr = worst_arr.max(retime(&alt_path));
                }
            }
            endpoint_req[end] - worst_arr
        })
    } else {
        None
    };

    // ---- hold (early / min-delay) path ----------------------------------
    // Earliest data arrival (min-corner cells + nominal no-crosstalk interconnect,
    // including the early launch clock) must clear the capture edge + hold. The
    // capture clock uses the LATE insertion delay (pessimistic for hold); CRPR adds
    // back the shared clock-path spread. The early forward pass was run above.
    let mut whs = f64::INFINITY;
    let mut ths = 0.0;
    let mut worst_hold = None;
    let mut hold_endpoints = 0;
    let mut hold_slacks: Vec<(usize, f64)> = Vec::new();
    // each path group's worst hold end, for the report: (slack, endpoint, run, data edge)
    let hold_end: HashMap<usize, usize> = flop_hold.iter().enumerate().map(|(e, (idx, _, _))| (*idx, e)).collect();
    let mut hold_group_worst: BTreeMap<String, (f64, usize, usize, usize)> = BTreeMap::new();
    // ⛔ PER LAUNCH WHERE CRPR TELLS THEM APART. The merged early arrival is the earliest launch's,
    // and its CRPR credit that launch's: a later launch whose clock path shares less with the
    // capture's gets less credit and can be the worse hold. The reference keeps a path per CRPR
    // clock pin (pruned, `pruneCrprArrivals`) and checks each; so does this, from the early
    // passes' recorded relaxations (`crpr_cands`). Measured on CoralNPU: a register's own
    // Q->D loop arrived first with 40.7 ps of credit, a neighbour 9.3 ps later with 25.7: the
    // reference's 47.237 against 52.923 here.
    let mut hold_keep = vec![false; n];
    for (idx, _, _) in &flop_hold {
        hold_keep[*idx] = true;
    }
    let hold_cands: Vec<HashMap<usize, [Vec<Cand>; 2]>> = if any_spread && early_recs.len() == runs.len() {
        runs.iter()
            .zip(&early_recs)
            .map(|(run, recs)| crpr_cands(recs, run.early_arr, &is_ck, &skip_from, &start_spread, false, &hold_keep, None).0)
            .collect()
    } else {
        Vec::new()
    };
    // the incremental fast path carries one launch per end
    let simple_ctx = simple_ctx && hold_cands.iter().all(|m| m.values().all(|c| c.iter().all(|s| s.len() < 2)));
    // a hold end whose worst launch is not its merged arrival's: (run, data edge, launch pin)
    let mut hold_pick: HashMap<usize, (usize, usize, usize)> = HashMap::new();
    for (idx, hold, ck) in &flop_hold {
        let idx = *idx;
        if arr_min[idx] == f64::INFINITY {
            continue; // unreached
        }
        let cap = ck_node(ck);
        // Same rule on the hold side; `flop_hold` carries both hold and REMOVAL.
        if unclocked_ck(cap) {
            continue;
        }
        let pc = cap.map(clock_period_of).unwrap_or(period);
        // ⛔ PER LAUNCHING EDGE AND DATA EDGE. A check exists for ONE data edge at a time —
        // `VisitPathEnds::visitCheckEnd` matches `check_arc->toEdge()` against the path's
        // `end_rf` — so each edge's own arrival is scored against that edge's own
        // constraint, and the worst of them is the endpoint's slack.
        //
        // Taking `min` over the lanes for the arrival and `max` over rise/fall for the
        // constraint scores a pairing no check has. It shows up on ASYNC release paths:
        // for an active-low reset the release is the RISING edge, and the reference's min
        // path is `^` at every node, while the minimum over lanes is the falling (assert)
        // edge. Measured on fft_top: all 132 async endpoints pessimistic, mean −0.1414.
        let mut slack = f64::INFINITY;
        let mut slack_at = (0, 0);
        let mut pick: Option<usize> = None;
        let mut checked = false;
        for (r, run) in runs.iter().enumerate() {
            let m_lck = launch_ck(idx, run.from_min);
            let m_port = if m_lck.is_none() { start_port(idx, run.from_min) } else { None };
            let cands = hold_cands.get(r).and_then(|m| m.get(&idx));
            let mut reached = false;
            for l in 0..2 {
                let a_m = run.early_arr[idx][l];
                if !a_m.is_finite() {
                    continue;
                }
                // each launch on THIS edge: (arrival, its clock pin, else its input port); the
                // earliest one's arrival as the pass found it (the sums agree but for rounding)
                let list: Vec<(f64, Option<usize>, Option<usize>)> = match cands.map(|c| &c[l]).filter(|c| !c.is_empty()) {
                    Some(c) => {
                        let first = c.iter().map(|e| e.0).fold(f64::INFINITY, f64::min);
                        c.iter()
                            .map(|&(a, k)| {
                                let k = k as usize;
                                let a = if a == first { a_m } else { a };
                                if is_ck[k] { (a, Some(k), None) } else { (a, None, is_in_port[k].then_some(k)) }
                            })
                            .collect()
                    }
                    None => vec![(a_m, m_lck, m_port)],
                };
                for (a, lck, sport) in list {
                    let pl = lck.map(clock_period_of).or_else(|| in_clock_at(sport, run.edge).map(period_named)).unwrap_or(pc);
                    // apply exceptions: false path drops the check, multicycle shifts the hold
                    // capture back by (cycles − 1).
                    let from_end = match lck {
                        Some(c) => reg_end(Some(c), c),
                        None => End {
                            name: sport.map(|v| labels[v].clone()).unwrap_or_default(),
                            clock: Some(in_clock_at(sport, run.edge).unwrap_or(primary_clock).to_string()),
                            port: true,
                        },
                    };
                    let to_end = reg_end(cap, idx);
                    let Some(shift) = exc_shift(true, &from_end, &to_end, pl, pc) else {
                        continue;
                    };
                    let in_clk = lck.map_or_else(|| in_clock_at(sport, run.edge), |_| None);
                    if is_async(lck, cap) || is_async_named(in_clk, to_end.clock.as_deref()) {
                        continue; // async clock groups: no hold check
                    }
                    reached = true;
                    // Each check at ITS capture edge: that edge's late clock arrival and early
                    // slew, and the hold relation from this run's launch edge to it.
                    for c in hold.iter().filter(|c| checks_edge(c, l == 0)) {
                        let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                        let crpr = match (lck, cap) {
                            (Some(lc), Some(cc)) => crpr_credit(lc, cc, cl, false),
                            _ => 0.0,
                        };
                        let cap_late = cap.map(|i| late_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                        let ck_slew = check_clk_slew(cap, cl, false);
                        let hold_rel = hold_relation(idx, pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)) + shift;
                        let base = crpr - cap_late - hold_rel - unc(to_end.clock.as_deref(), false);
                        let sl = a + base - c.eval_edge(l == 0, ck_slew, early_slew[idx][l]);
                        if sl < slack {
                            slack = sl;
                            slack_at = (r, l);
                            // the report follows this edge's own predecessors: traced only for
                            // a launch they do not reach
                            pick = lck.or(sport).filter(|k| Some(*k) != chain_start(run.early_from, idx, l));
                        }
                    }
                }
            }
            checked |= reached;
            if reached && simple_ctx {
                let pl = launch_period(m_lck, idx, run.from_min, run.edge, pc);
                let (from_end, to_end) = (launch_end(m_lck, idx, run.from_min, run.edge), reg_end(cap, idx));
                if let (Some(shift), Some(c)) = (exc_shift(true, &from_end, &to_end, pl, pc), hold.first()) {
                    let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                    let crpr = match (m_lck, cap) {
                        (Some(lc), Some(cc)) => crpr_credit(lc, cc, cl, false),
                        _ => 0.0,
                    };
                    let cap_late = cap.map(|i| late_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                    let ck_slew = check_clk_slew(cap, cl, false);
                    let hold_rel = hold_relation(idx, pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)) + shift;
                    inc_hold.push(HoldRec {
                        idx,
                        base: crpr - cap_late - hold_rel - unc(to_end.clock.as_deref(), false),
                        cons: hold.to_vec(),
                        ck_slew,
                        launch_ck: m_lck,
                        port: false,
                    });
                }
            }
        }
        if let Some(k) = pick {
            hold_pick.insert(idx, (slack_at.0, slack_at.1, k));
        }
        if !checked || !slack.is_finite() {
            continue; // no edge reached this pin after all
        }
        hold_endpoints += 1;
        hold_slacks.push((idx, slack));
        if slack < 0.0 {
            ths += slack;
        }
        if slack < whs {
            whs = slack;
            worst_hold = Some((idx, slack_at));
        }
        let g = path_group(idx, hold_end.get(&idx).and_then(|&e| ck_node(&flop_hold[e].2)));
        if is_probe(idx) {
            hold_group_worst.insert("probe".to_string(), (slack, idx, slack_at.0, slack_at.1));
        }
        if hold_group_worst.get(&g).is_none_or(|w| slack < w.0) {
            hold_group_worst.insert(g, (slack, idx, slack_at.0, slack_at.1));
        }
    }
    // data checks, hold: the to-pin's data from the NEXT launch (`src_cycle + 1`) against the
    // from-pin's LATE arrival plus the hold uncertainty and margin
    for (f, t, m) in &data_checks {
        if hold_end.contains_key(t) {
            vyges_events::emit(
                &vyges_events::Event::new("vyges-sta-si", vyges_events::Severity::Warn, format!("data check: hold at {}, a register data pin, is not modelled; not applied", labels[*t]))
                    .with_code("STA-DATA-CHECK"),
            );
            continue;
        }
        let mut slack = f64::INFINITY;
        let mut slack_at = (0, 0);
        for (tl, &a) in runs[0].early_arr[*t].iter().enumerate() {
            if !a.is_finite() {
                continue;
            }
            for (fl, by_to) in m[1].iter().enumerate() {
                let (Some(mg), late) = (by_to[tl], runs[0].late_arr[*f][fl]) else { continue };
                if late.is_finite() {
                    let sl = a + period - (late + unc(launch_ck(*f, runs[0].from).and_then(clock_name_of), false) + mg);
                    if sl < slack {
                        slack = sl;
                        slack_at = (0, tl);
                    }
                }
            }
        }
        if !slack.is_finite() {
            continue;
        }
        hold_endpoints += 1;
        hold_slacks.push((*t, slack));
        if slack < 0.0 {
            ths += slack;
        }
        if slack < whs {
            whs = slack;
            worst_hold = Some((*t, slack_at));
        }
        let g = path_group(*t, None);
        if hold_group_worst.get(&g).is_none_or(|w| slack < w.0) {
            hold_group_worst.insert(g, (slack, *t, slack_at.0, slack_at.1));
        }
    }
    // An output port's HOLD check. `set_output_delay` constrains both analyses (the reference's
    // `PathEndOutputDelay`): the data must not arrive before the capture edge less the
    // external delay — required = hold edge − output delay (+ hold uncertainty), per edge.
    // Without it no output was ever hold-checked: the reference's min report on a design
    // whose only endpoint is an output port says `0.2196 slack`, this timer said nothing.
    for (idx, name) in &output_ports {
        let idx = *idx;
        if !arr_min[idx].is_finite() || !job.output_delay_declared_for(name) {
            continue; // unreached, or unconstrained
        }
        let mut slack = f64::INFINITY;
        let mut slack_at = (0, 0);
        let mut group = primary_clock.to_string();
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(idx, run.from_min);
            let pl = launch_period(lck, idx, run.from_min, run.edge, period);
            let chk = out_check(idx, &launch_end(lck, idx, run.from_min, run.edge), run.edge, pl, false);
            let mut reached = false;
            for (l, &a) in run.early_arr[idx].iter().enumerate() {
                let Some((req, clk, _, _)) = &chk[l] else { continue };
                if a.is_finite() {
                    reached = true;
                    if a - req < slack {
                        slack = a - req;
                        slack_at = (r, l);
                        group = clk.clone();
                    }
                }
            }
            if reached && simple_ctx {
                let base = chk.iter().flatten().map(|c| -c.0).fold(f64::INFINITY, f64::min);
                inc_hold.push(HoldRec { idx, base, cons: Vec::new(), ck_slew: input_slew, launch_ck: lck, port: true });
            }
        }
        if !slack.is_finite() {
            continue;
        }
        hold_endpoints += 1;
        hold_slacks.push((idx, slack));
        if slack < 0.0 {
            ths += slack;
        }
        if slack < whs {
            whs = slack;
            worst_hold = Some((idx, slack_at));
        }
        if hold_group_worst.get(&group).is_none_or(|w| slack < w.0) {
            hold_group_worst.insert(group, (slack, idx, slack_at.0, slack_at.1));
        }
    }
    // The path of the edge whose check set the slack — not the earliest lane at each node,
    // which can be the other edge's path printed beside this one's slack.
    let (worst_hold_endpoint, worst_hold_path) = match worst_hold {
        Some((v, (r, l))) => (labels[v].clone(), crate::inc::lane_path(&labels, runs[r].early_arr, &early_slew, runs[r].early_from, v, l)),
        None => (String::new(), Vec::new()),
    };

    // ---- reported paths ----------------------------------------------------
    // Each path group's worst end as the reference's full `report_checks` prints it. Every term
    // is the one its check was scored with (the expressions above); the slack is asserted equal.
    let check_paths: Vec<CheckPath> = {
        let inst_cell: HashMap<&str, &str> = nl.insts.iter().map(|i| (i.name.as_str(), i.cell.as_str())).collect();
        let in_port: std::collections::HashSet<usize> = input_ports.iter().map(|(i, _)| *i).collect();
        let out_port: std::collections::HashSet<usize> = output_ports.iter().map(|(i, _)| *i).collect();
        let driver: std::collections::HashSet<usize> = nets.values().filter_map(|n| n.driver).collect();
        // `pin (cell)`, or a port's `name (in)` / `name (out)`
        let desc = |u: usize| -> String {
            if in_port.contains(&u) {
                format!("{} (in)", labels[u])
            } else if out_port.contains(&u) {
                format!("{} (out)", labels[u])
            } else {
                let inst = instance_of(&labels[u]);
                format!("{} ({})", labels[u], inst_cell.get(inst).copied().unwrap_or("?"))
            }
        };
        let clock_line = |name: &str, rise: bool, time: f64| -> ClockLine {
            let src = job.clocks.iter().find(|c| c.0 == name).map_or(job.clock_port.as_str(), |c| c.1.as_str());
            let ideal = job.ideal_clocks.get(src);
            // a virtual clock (no source) is ideal
            let propagated = !src.is_empty() && ideal.is_none();
            ClockLine { name: name.to_string(), rise, time, slew: ideal.map_or(0.0, |s| s[usize::from(!rise)]), propagated }
        };
        // a register's description from the clock-to-output arcs of its clock pin: the one on
        // `edge` if it has one (`ReportPath::clkRegLatchDesc`)
        let reg_desc = |ck: usize, edge: Option<usize>| -> &'static str {
            let label = &labels[ck];
            let (inst, pin) = label.rsplit_once('/').unwrap_or((label.as_str(), ""));
            let edges: Vec<usize> = inst_cell
                .get(inst)
                .and_then(|c| lib.cell(c))
                .map(|c| {
                    c.pins
                        .values()
                        .flat_map(|p| p.arcs.iter())
                        .filter(|a| a.related_pin == pin)
                        .filter_map(|a| a.clk_edge.map(|e| usize::from(e == ClkEdge::Fall)))
                        .collect()
                })
                .unwrap_or_default();
            let e = edge.filter(|e| edges.contains(e)).or(edges.first().copied()).or(edge).unwrap_or(0);
            // a latch is named by the level that opens it: the edge its enable launches on
            let latch = inst_cell.get(inst).and_then(|c| lib.cell(c)).and_then(|c| c.seq_group.as_ref()).is_some_and(|g| g.latch);
            match (latch, e == 1) {
                (true, false) => "positive level-sensitive latch",
                (true, true) => "negative level-sensitive latch",
                (false, true) => "falling edge-triggered flip-flop",
                (false, false) => "rising edge-triggered flip-flop",
            }
        };
        let make = |max: bool, v: usize, run: &EdgeView, l: usize, group: &str| -> Option<CheckPath> {
            let (arr, from, slews, mm) =
                if max { (run.late_arr, run.late_from, &late_slew, 1) } else { (run.early_arr, run.early_from, &early_slew, 0) };
            let mut chain = vec![(v, l)];
            let (mut x, mut xl) = (v, l);
            while let Some((u, ul)) = from[x][xl] {
                chain.push((u, ul));
                (x, xl) = (u, ul);
            }
            chain.reverse();
            // the launching register's clock pin, else the input port the path starts at
            let k = chain.iter().position(|&(u, _)| is_ck[u]);
            let start = k.unwrap_or(0);
            let (s0, s0l) = chain[start];
            if k.is_none() && !in_port.contains(&s0) {
                return None; // a clock used as data: not reported here
            }
            // the launching clock pin as the check was scored with it (`launch_ck` over the merged
            // predecessors), which on a design with several clocks need not be the one this
            // edge's path reaches
            let pred = if max { run.from } else { run.from_min };
            let lck = launch_ck(v, pred);
            // an input's launching clock is its delay's, on this run's edge
            let src_name = lck.and_then(clock_name_of).or_else(|| in_clock_at(start_port(v, pred), run.edge)).unwrap_or(primary_clock);
            // the launch clock's period; with no launching clock pin, the input delay's clock's,
            // else the period the check defaults to: the capture clock's for a register, the
            // job's for an output
            let pl_or = |default: f64| launch_period(lck, v, pred, run.edge, default);
            let pl = pl_or(period);
            let launch_edge = if k.is_some() { src_edge(Some(s0), s0l) } else { run.edge };
            let launch = clock_edge_time(job, Some(src_name), launch_edge, pl);
            let src_clock = clock_line(src_name, launch_edge == 0, launch);
            let (startpoint, start_reason) = match k {
                Some(_) => (instance_of(&labels[s0]).to_string(), format!("{} clocked by {src_name}", reg_desc(s0, Some(s0l)))),
                None => (labels[s0].clone(), format!("input port clocked by {src_name}")),
            };
            let src_network_delay = if k.is_some() { arr[s0][s0l] } else { 0.0 };
            let input_delay = if k.is_none() { Some((arr[s0][s0l], s0l == 0)) } else { None };
            let lines: Vec<PathLine> = chain[start..]
                .iter()
                .enumerate()
                .map(|(i, &(u, ul))| {
                    let clk_start = k.is_some() && i == 0;
                    let slew = if clk_start && !src_clock.propagated { src_clock.slew } else { slews[u][ul] };
                    PathLine {
                        desc: desc(u),
                        rise: ul == 0,
                        time: launch + arr[u][ul],
                        slew,
                        cap: driver.contains(&u).then(|| node_load[u][mm][ul]),
                        clk_start,
                    }
                })
                .collect();
            let arrival = launch + arr[v][l];
            let from_end = launch_end(lck, v, pred, run.edge);
            if out_port.contains(&v) {
                // an output port, captured by the clock edge of the delay that set the slack
                let (req, clk, cl, rel) = out_check(v, &from_end, run.edge, pl, max)[l].clone()?;
                let (tgt_time, required, uncertainty) =
                    (launch + rel, launch + req, if max { -unc(Some(&clk), true) } else { unc(Some(&clk), false) });
                return Some(CheckPath {
                    max,
                    group: group.to_string(),
                    startpoint,
                    start_reason,
                    endpoint: labels[v].clone(),
                    end_reason: format!("output port clocked by {clk}"),
                    src_clock,
                    src_network_delay,
                    input_delay,
                    lines,
                    arrival,
                    tgt_clock: clock_line(&clk, cl == 0, tgt_time),
                    tgt_network_delay: 0.0,
                    uncertainty,
                    crpr: 0.0,
                    tgt_pin: None,
                    margin_what: "output external delay".to_string(),
                    required,
                    slack: if max { required - arrival } else { arrival - required },
                });
            }
            // a register's check, its terms as scored: the constraint that set the slack
            let (cons, ck) = if max {
                let e = setup_end.get(&v)?;
                (&flop_d[*e].1, &flop_d[*e].2)
            } else {
                let e = hold_end.get(&v)?;
                (&flop_hold[*e].1, &flop_hold[*e].2)
            };
            let cap = ck_node(ck);
            let pc = cap.map(clock_period_of).unwrap_or(period);
            let pl = pl_or(pc);
            let to_end = reg_end(cap, v);
            let shift = exc_shift(!max, &from_end, &to_end, pl, pc).unwrap_or(0.0);
            // (required, capture edge, capture clock arrival, relation) of each constraint;
            // the tightest one set the slack
            // a path through a latch's D->Q carries no CRPR (see `via_latch`)
            let crpr_lck = lck.filter(|_| !(max && via_latch(run, &latch_seeds_final, v, l)));
            let terms = cons.iter().filter(|c| checks_edge(c, l == 0)).map(|c| {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let crpr = match (crpr_lck, cap) {
                    (Some(a), Some(c)) => crpr_credit(a, c, cl, max),
                    _ => 0.0,
                };
                if max {
                    let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                    let ck_slew = check_clk_slew(cap, cl, true);
                    let rel = edge_relation(pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)).0 + shift;
                    let req = cap_early + rel + crpr - unc(to_end.clock.as_deref(), true) - c.eval_edge(l == 0, ck_slew, late_slew[v][l]);
                    (req, cl, cap_early, rel, crpr)
                } else {
                    let cap_late = cap.map(|i| late_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                    let ck_slew = check_clk_slew(cap, cl, false);
                    let rel = hold_relation(v, pl, from_end.clock.as_deref(), run.edge, pc, to_end.clock.as_deref(), src_edge(cap, cl)) + shift;
                    let req = cap_late + rel + unc(to_end.clock.as_deref(), false) - crpr + c.eval_edge(l == 0, ck_slew, early_slew[v][l]);
                    (req, cl, cap_late, rel, crpr)
                }
            });
            let (req, cl, cap_arr, rel, crpr) = if max {
                terms.min_by(|a, b| a.0.total_cmp(&b.0))?
            } else {
                terms.max_by(|a, b| a.0.total_cmp(&b.0))?
            };
            let tgt_name = cap.and_then(clock_name_of).unwrap_or(primary_clock);
            // A latch data pin's setup check (`PathEndLatchCheck`): its target is the edge that
            // opens the latch, with that edge's latency and CRPR, and the time borrowed in place
            // of the setup time; required is the opening plus the borrow.
            if max && latch_d.contains(&v) {
                let (open, rel_open, open_latency, open_crpr) = latch_opening(cap, crpr_lck, to_end.clock.as_deref(), pc, cl, rel);
                let req = borrow_limit(v, cap, to_end.clock.as_deref()).map_or(req, |lim| open + lim);
                let (slack, borrow) = latch_setup(open, req, arrival - launch);
                return Some(CheckPath {
                    max,
                    group: group.to_string(),
                    startpoint,
                    start_reason,
                    endpoint: instance_of(&labels[v]).to_string(),
                    // open on the rise: a positive latch
                    end_reason: format!("{} level-sensitive latch clocked by {tgt_name}", if cl == 1 { "positive" } else { "negative" }),
                    src_clock,
                    src_network_delay,
                    input_delay,
                    lines,
                    arrival,
                    tgt_clock: clock_line(tgt_name, src_edge(cap, 1 - cl) == 0, launch + rel_open),
                    tgt_network_delay: open_latency,
                    uncertainty: -unc(Some(tgt_name), true),
                    crpr: open_crpr,
                    tgt_pin: cap.map(|c| (desc(c), cl == 1)),
                    margin_what: "time borrowed from endpoint".to_string(),
                    required: launch + open + borrow,
                    slack,
                });
            }
            let is_async = async_end.contains(&v);
            let check = match (max, is_async) {
                (true, false) => "setup",
                (true, true) => "recovery",
                (false, false) => "hold",
                (false, true) => "removal",
            };
            let end_reason = if is_async {
                format!("{check} check against {}-edge clock {tgt_name}", if cl == 1 { "falling" } else { "rising" })
            } else {
                format!("{} clocked by {tgt_name}", cap.map_or("rising edge-triggered flip-flop", |c| reg_desc(c, Some(cl))))
            };
            Some(CheckPath {
                max,
                group: group.to_string(),
                startpoint,
                start_reason,
                endpoint: instance_of(&labels[v]).to_string(),
                end_reason,
                src_clock,
                src_network_delay,
                input_delay,
                lines,
                arrival,
                tgt_clock: clock_line(tgt_name, src_edge(cap, cl) == 0, launch + rel),
                tgt_network_delay: cap_arr,
                uncertainty: if max { -unc(Some(tgt_name), true) } else { unc(Some(tgt_name), false) },
                crpr: if max { crpr } else { -crpr },
                tgt_pin: cap.map(|c| (desc(c), cl == 0)),
                margin_what: format!("library {check} time"),
                required: launch + req,
                slack: if max { launch + req - arrival } else { arrival - launch - req },
            })
        };
        // the reference's group order, per analysis: `asynchronous`, then the clocks by name
        let ordered = |m: &BTreeMap<String, (f64, usize, usize, usize)>| -> Vec<(String, (f64, usize, usize, usize))> {
            let mut v: Vec<_> = m.iter().map(|(g, w)| (g.clone(), *w)).collect();
            v.sort_by_key(|(g, _)| g != "asynchronous");
            v
        };
        let mut out = Vec::new();
        for (max, worst) in [(true, &setup_group_worst), (false, &hold_group_worst)] {
            for (g, (slack, v, r, l)) in ordered(worst) {
                // a hold end whose worst launch is not the merged arrival's: that launch's own
                // path, as it survived the pruning (`crpr_cands` traced)
                let pick = if max { setup_pick.get(&v) } else { hold_pick.get(&v) };
                let traced = pick.filter(|p| p.0 == r && p.1 == l).map(|&(_, _, k)| {
                    let recs = if max { &late_recs[r] } else { &early_recs[r] };
                    let arr = if max { runs[r].late_arr } else { runs[r].early_arr };
                    let (_, tr) = crpr_cands(recs, arr, &is_ck, &skip_from, &start_spread, max, &vec![false; n], Some(k as u32));
                    let (ta, tp) = tr.unwrap();
                    let fm: Vec<Option<usize>> = (0..n)
                        .map(|x| {
                            let b = if (ta[x][0] <= ta[x][1]) != max { 0 } else { 1 };
                            tp[x][b].map(|(u, _)| u)
                        })
                        .collect();
                    (ta, tp, fm)
                });
                let view = match (&traced, max) {
                    (Some((ta, tp, fm)), true) => EdgeView { edge: runs[r].edge, late_arr: ta, late_from: tp, from: fm, early_arr: runs[r].early_arr, early_from: runs[r].early_from, from_min: runs[r].from_min },
                    (Some((ta, tp, fm)), false) => EdgeView { edge: runs[r].edge, late_arr: runs[r].late_arr, late_from: runs[r].late_from, from: runs[r].from, early_arr: ta, early_from: tp, from_min: fm },
                    (None, _) => runs[r],
                };
                if let Some(p) = make(max, v, &view, l, &g) {
                    debug_assert!((p.slack - slack).abs() < 1e-9, "{g}: reported {} scored {slack}", p.slack);
                    out.push(p);
                }
            }
        }
        out
    };

    let report = TimingReport {
        wns: if endpoints == 0 { f64::INFINITY } else { wns },
        tns,
        endpoints,
        worst_endpoint,
        worst_path,
        whs: if hold_endpoints == 0 {
            f64::INFINITY
        } else {
            whs
        },
        ths,
        hold_endpoints,
        worst_hold_endpoint,
        worst_hold_path,
        hold_slacks,
        pba_wns,
        units: lib.units,
        check_paths,
        latch_borrow,
    };
    // Retain a per-pin snapshot for the Timer's query API (Phase 1). Cloned, not moved:
    // closures above still hold immutable borrows of these arrays through the return.
    // the query API reports one load per driver: the larger edge's
    let node_load_max: Vec<f64> = node_load.iter().map(|l| l[1][0].max(l[1][1])).collect();
    let mut timing = Timing::new(
        &labels,
        &is_endpoint,
        &excluded_setup,
        &arrival,
        &clk_arr,
        &slew,
        &arr_min,
        &slew_min,
        &node_load_max,
        &endpoint_req,
        &late_slew,
        &node_load.iter().map(|l| l[1]).collect::<Vec<[f64; 2]>>(),
    );
    timing.check_arr = check_arr;

    // Capture the persistent incremental graph (simple context only). Built once here; the
    // optimizer's `update()` then recomputes only the cone of a cell swap against it.
    let inc = if simple_ctx {
        let mut succ: Vec<Vec<usize>> = vec![Vec::new(); n];
        let mut in_edges: Vec<Vec<InEdge>> = (0..n).map(|_| Vec::new()).collect();
        for (u, edges) in out_edges.iter().enumerate() {
            for e in edges {
                succ[u].push(e.to);
                in_edges[e.to].push(match &e.kind {
                    EdgeKind::Net(_) => InEdge::Net { from: u },
                    // the incremental graph outlives the build: it keeps its own copy
                    EdgeKind::Cell(arc) => InEdge::Cell {
                        from: u,
                        arc: Box::new((*arc).clone()),
                    },
                });
            }
        }
        let mut topo_pos = vec![0usize; n];
        for (pos, &node) in order_late.iter().enumerate() {
            topo_pos[node] = pos;
        }
        let mut net_driver: HashMap<String, usize> = HashMap::new();
        for (name, net) in &nets {
            if let Some(d) = net.driver {
                net_driver.insert(name.clone(), d);
            }
        }
        let topo = IncTopo {
            n,
            label2idx: labels
                .iter()
                .enumerate()
                .map(|(i, l)| (l.clone(), i))
                .collect(),
            labels: labels.clone(),
            succ,
            in_edges,
            topo_pos,
            is_ck: is_ck.clone(),
            is_endpoint: is_endpoint.clone(),
            excluded_setup: excluded_setup.clone(),
            net_driver,
            setup_recs: inc_setup,
            hold_recs: inc_hold,
            late_derate,
            early_derate,
            input_slew,
        };
        let state = IncState {
            node_load: node_load.clone(),
            late: Lanes {
                arr: late_arr,
                slew: late_slew,
                from: late_from,
            },
            early: Lanes {
                arr: early_arr,
                slew: early_slew,
                from: early_from,
            },
            arrival: arrival.clone(),
            slew: slew.clone(),
            from: from.clone(),
            arr_min: arr_min.clone(),
            slew_min: slew_min.clone(),
            from_min: from_min.clone(),
            endpoint_req: endpoint_req.clone(),
            clk_arrival: clk_arr.clone(),
            overrides: HashMap::new(),
        };
        Some(IncGraph { topo, state })
    } else {
        None
    };
    Ok((report, timing, inc))
}

#[cfg(test)]
mod per_edge_load {
    use super::*;

    /// The parasitics' units in the library's: the literals a ns, pF library always used, to the
    /// bit; a ps, fF library keeps femtofarads, the same kΩ, and Ω·fF in ps.
    /// `DelayCalcBase::thresholdAdjust`, measured on `examples/thresholds` (the reference's
    /// `report_dcalc`): a 20/80, 50 % driver into a 10/90, 40 % load turns a 0.075445 load slew
    /// into 0.100593 (x 0.8/0.6), and shifts the wire delay by slew x (0.4 - 0.5) / 0.6 — ADDED on
    /// a rising edge, SUBTRACTED on a falling one. Equal thresholds: the identity.
    #[test]
    fn a_load_from_another_library_is_measured_at_its_thresholds() {
        let lib = |lo: f64, hi: f64, vin: f64, vout: f64| crate::liberty::Thresholds {
            slew_lower_rise: lo, slew_lower_fall: lo, slew_upper_rise: hi, slew_upper_fall: hi,
            input_rise: vin, input_fall: vin, output_rise: vout, output_fall: vout,
            ..crate::liberty::Thresholds::default()
        };
        let (a, b) = (lib(0.2, 0.8, 0.5, 0.5), lib(0.1, 0.9, 0.4, 0.6));
        let (d, s) = threshold_adjust(&a, &b, true, 0.0, 0.075445);
        assert!((s - 0.100593).abs() < 1e-6, "{s}");
        assert!((d - (-0.075445 * 0.1 / 0.6)).abs() < 1e-7, "{d}");
        let (d, _) = threshold_adjust(&a, &b, false, 0.0, 0.075445);
        assert!(d > 0.0, "a falling edge shifts the other way: {d}");
        assert_eq!(threshold_adjust(&a, &a, true, 0.0123, 0.0456), (0.0123, 0.0456));
    }

    #[test]
    fn spef_scales_are_exact_powers_of_ten() {
        use crate::liberty::LibUnits;
        assert_eq!(spef_scales(LibUnits::default()), (1000.0, 1000.0, 1e-6));
        let ps = LibUnits { time_s: 1e-12, cap_f: 1e-15, ..LibUnits::default() };
        assert_eq!(spef_scales(ps), (1.0, 1000.0, 1e-3));
        assert_eq!(1.0 / spef_scales(LibUnits::default()).1, 1e-3);
        assert_eq!(1.0 / spef_scales(LibUnits::default()).0, 1e-3);
    }

    /// Reference rule: the default delay calculator loads a net with each pin's Liberty
    /// capacitance (`Sdc::pinCapacitance` in `ReduceToPi`), per edge and analysis, even where the
    /// pin also carries a CCS `receiver_capacitance` model. The reference's reg1_asap7 test: u2/B
    /// loads u1/Y's rise with 0.526025 fF (rise range maximum), not the receiver model's 0.3034.
    #[test]
    fn a_receiver_model_does_not_replace_the_pin_capacitance() {
        let lib = crate::liberty::Lib::parse(
            r#"library(t) { capacitive_load_unit (1, pf);
  lu_table_template (r) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 ("0.05"); index_2 ("0.005"); }
  cell (R) { pin(A) { direction : input; capacitance : 0.0019; rise_capacitance : 0.0021; fall_capacitance : 0.0017;
                       rise_capacitance_range (0.0015, 0.0021); fall_capacitance_range (0.0011, 0.0017);
      receiver_capacitance () {
        receiver_capacitance1_rise (r) { values ("0.0030"); } receiver_capacitance2_rise (r) { values ("0.0030"); }
        receiver_capacitance1_fall (r) { values ("0.0030"); } receiver_capacitance2_fall (r) { values ("0.0030"); } } }
    pin(Y) { direction : output; } } }"#,
        )
        .unwrap();
        let a = &lib.cell("R").unwrap().pins["A"];
        assert!(a.recv.as_ref().is_some_and(|r| !r.is_empty()), "the receiver model is read");
        let c = pin_load_caps(a);
        let r = |x: f64| (x * 1e7).round() / 1e7;
        assert_eq!([r(c[0][0]), r(c[0][1]), r(c[1][0]), r(c[1][1])], [0.0015, 0.0011, 0.0021, 0.0017]);
    }

    /// Reference rule: a receiver loads its driver's rising edge with its rise capacitance and
    /// the falling edge with its fall capacitance, `capacitance` only where those are absent.
    /// An early (hold) analysis takes each range's minimum, a late (setup) one its maximum.
    /// Measured: icsprout55's two-flop example times setup to the reference's last digit with
    /// this (ss 1.680693, tt 1.824064), 6 ps off with the single `capacitance`.
    #[test]
    fn a_receiver_loads_each_edge_with_that_edges_capacitance() {
        let lib = crate::liberty::Lib::parse(
            r#"library(t) { capacitive_load_unit (1, pf);
  cell (B) { pin(A) { direction : input; capacitance : 0.0020; rise_capacitance : 0.0022; fall_capacitance : 0.0018;
                       rise_capacitance_range (0.0016, 0.0022); fall_capacitance_range (0.0012, 0.0018); }
             pin(C) { direction : input; capacitance : 0.0030; } }
}"#,
        )
        .unwrap();
        let c = &lib.cells["B"];
        // Liberty values are read as f32, as the reference reads them. [early min, late max][rise, fall]
        let f = |v: f32| v as f64;
        assert_eq!(pin_load_caps(&c.pins["A"]), [[f(0.0016), f(0.0012)], [f(0.0022), f(0.0018)]]);
        assert_eq!(pin_load_caps(&c.pins["C"]), [[f(0.0030); 2]; 2]);
    }
}

#[cfg(test)]
mod latch_check {
    use super::latch_setup;

    /// `Latches::latchRequired`'s three cases, opening at 1 and closing at 3 (maximum borrow 2):
    /// data before the opening meets it with nothing borrowed; data in the window borrows and
    /// meets its check exactly; data past it borrows the maximum and fails by the rest.
    #[test]
    fn the_latch_check_borrows_within_the_window() {
        assert_eq!(latch_setup(1.0, 3.0, 0.5), (0.5, 0.0));
        assert_eq!(latch_setup(1.0, 3.0, 1.0), (0.0, 0.0));
        assert_eq!(latch_setup(1.0, 3.0, 2.5), (0.0, 1.5));
        assert_eq!(latch_setup(1.0, 3.0, 3.0), (0.0, 2.0));
        assert_eq!(latch_setup(1.0, 3.0, 4.0), (-1.0, 2.0));
    }
}

#[cfg(test)]
mod loop_naming {
    use super::*;

    /// The cycle named is the cycle alone: a node leading into it, and one after it, are left
    /// out, and the nodes the order DID pass are never entered.
    #[test]
    fn the_named_cycle_is_only_the_cycle() {
        // 0 -> 1 -> 2 -> 1 (the cycle), 2 -> 3 (after it); 4 -> 0 passed by the order
        let adj = vec![vec![1], vec![2], vec![1, 3], vec![], vec![0]];
        let done = [false, false, false, false, true];
        assert_eq!(find_cycle(&adj, &done), Some(vec![1, 2]));
        // nothing left out, or nothing cyclic among what was: no cycle to name
        assert_eq!(find_cycle(&adj, &[true; 5]), None);
        assert_eq!(find_cycle(&[vec![1], vec![]], &[false, false]), None);
    }

    #[test]
    fn the_refusal_lists_the_pins_and_closes_the_cycle() {
        let e = StaError::CombinationalLoop(vec!["g1/A".into(), "g1/Y".into(), "g2/A".into(), "g2/Y".into()]);
        let m = e.to_string();
        assert!(m.contains("through 4 pin(s): g1/A -> g1/Y -> g2/A -> g2/Y -> g1/A"), "{m}");
    }
}

/// The reference's clock gating function rule (`GatedClk::isClkGatingFunc`) and its gated clock
/// hold cycle (`CycleAccting::findDelays`).
#[cfg(test)]
mod clock_gating {
    use super::{clk_gating_active_one, cycle_accting_gated};
    use vyges_loom::liberty_func::Func;

    fn f(s: &str) -> Func {
        Func::parse_in(s, |_| true).unwrap()
    }

    /// AND gates with the enable at one, OR at zero; outer negations are stripped (a NAND is an
    /// AND, a NOR an OR); a negated clock operand flips the value; a function that is neither,
    /// or an enable the operands do not name, is no clock gate.
    #[test]
    fn gate_function() {
        assert_eq!(clk_gating_active_one(&f("A & B"), "B", "A"), Some(true));
        assert_eq!(clk_gating_active_one(&f("A | B"), "B", "A"), Some(false));
        assert_eq!(clk_gating_active_one(&f("!(A & B)"), "A", "B"), Some(true));
        assert_eq!(clk_gating_active_one(&f("!(A | B)"), "B", "A"), Some(false));
        assert_eq!(clk_gating_active_one(&f("!A & B"), "B", "A"), Some(false));
        assert_eq!(clk_gating_active_one(&f("A & B & C"), "C", "A"), Some(true));
        assert_eq!(clk_gating_active_one(&f("A ^ B"), "B", "A"), None);
        assert_eq!(clk_gating_active_one(&f("(A & B) | C"), "B", "A"), None);
        assert_eq!(clk_gating_active_one(&f("A & B"), "C", "A"), None);
    }

    /// The gated clock hold check is in the setup check's cycle: launched on the rise (0, period
    /// 10) against the fall (5), opened by the rise at 0: `src - tgt` = -5 (an ordinary hold
    /// pairs it with the fall before, +5). Launched on the fall (5) against the fall: 0. An OR's
    /// hold edge is the rise, opened by the fall after it: no rise at or after 0 has its fall at
    /// or before a launch at 0, so the search pairs the next launch (10) with the rise at 0: 10.
    #[test]
    fn gated_hold_cycle() {
        assert_eq!(cycle_accting_gated(10.0, 0.0, 10.0, 5.0, 0.0), (5.0, 5.0, -5.0));
        assert_eq!(cycle_accting_gated(10.0, 5.0, 10.0, 5.0, 0.0).2, 0.0);
        assert_eq!(cycle_accting_gated(10.0, 0.0, 10.0, 0.0, 5.0).2, 10.0);
    }
}


#[cfg(test)]
mod elmore_by_node_rule {
    use super::{elmore_at, elmore_by_node};
    use std::collections::HashMap;

    /// The rule: a net's Elmore is looked up by node exactly as collecting the walk into a map
    /// gave it, so a node the walk lists twice reads its LATER value, and a node the walk never
    /// reached reads none.
    #[test]
    fn a_node_listed_twice_reads_its_later_value_as_the_map_did() {
        let walk = vec![(7, 0.5), (3, 0.25), (7, 0.75), (11, 1.0), (3, 0.125)];
        let by_node = elmore_by_node(walk.clone());
        let map: HashMap<u32, f64> = walk.into_iter().collect();
        for n in [3, 7, 11] {
            assert_eq!(elmore_at(&by_node, n), map.get(&n).copied(), "node {n}");
        }
        assert_eq!(elmore_at(&by_node, 7), Some(0.75));
        assert_eq!(elmore_at(&by_node, 5), None);
        assert_eq!(by_node.len(), 3);
    }
}

#[cfg(test)]
mod checks_of_rule {
    use super::{checks_of, Constraint};
    use std::borrow::Cow;

    fn check(related: &str) -> Constraint {
        Constraint { related_pin: related.to_string(), ..Default::default() }
    }

    /// The rule: a register's check arcs are the library cell's own (the reference shares one
    /// arc set across every instance of a cell), so with nothing disabled the instance holds the
    /// library's list itself, not a copy; a `set_disable_timing` that removes a check leaves the
    /// rest, in library order.
    #[test]
    fn all_kept_is_the_library_list_itself_and_a_disable_keeps_the_rest_in_order() {
        let lib = vec![check("CK"), check("SE"), check("CK2")];
        let all = checks_of(&lib, |_| true);
        assert!(matches!(all, Cow::Borrowed(s) if std::ptr::eq(s, lib.as_slice())), "nothing disabled: the library list, borrowed");
        let some = checks_of(&lib, |c| c.related_pin != "SE");
        let kept: Vec<&str> = some.iter().map(|c| c.related_pin.as_str()).collect();
        assert_eq!(kept, ["CK", "CK2"]);
        assert!(checks_of(&lib, |_| false).is_empty());
    }
}

