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
//! (per-stage sigma — from LVF `ocv_sigma_*` tables when present, else a global
//! fraction — N-sigma band growing as sqrt(depth)) when configured. Multiple
//! clocks (incl. generated) are supported — cross-domain paths use the tightest
//! launch→capture edge relation — and false-path / multicycle **exceptions** drop or
//! shift paths. With `pba` on, a path-based pass re-times the critical path and its
//! fan-in alternatives (path-local slew) to catch non-greedy worst paths. Pure std
//! — unit-tested.

use std::collections::{BTreeMap, HashMap};

use crate::inc::{HoldRec, InEdge, IncGraph, IncState, IncTopo, Lanes, SetupRec};
use crate::job::{ExcKind, StaJob};
use crate::liberty::{Arc, Constraint, Dir, Lib};
use crate::netlist::Netlist;
use crate::spef::Spef;

#[derive(Debug)]
pub enum StaError {
    Parse(String),
    Io(String),
    UnknownCell(String),
    CombinationalLoop,
    /// Reserved: SI/crosstalk analysis was requested but isn't modeled in v0.
    SiNotModeled,
}

impl std::fmt::Display for StaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StaError::Parse(m) => write!(f, "parse error: {m}"),
            StaError::Io(m) => write!(f, "io error: {m}"),
            StaError::UnknownCell(c) => write!(f, "cell not in any .lib: {c}"),
            StaError::CombinationalLoop => {
                write!(f, "combinational loop (sequential timing not modeled in v0)")
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
fn is_power_pin(pin: &str) -> bool {
    let p = pin.trim_start_matches('\\').to_ascii_uppercase();
    const RAILS: &[&str] = &[
        "VPWR", "VGND", "VPB", "VNB", // sky130 std cell
        "VDD", "VSS", "VDDA", "VSSA", "VCC", "VEE", "VPP", "GND", "VNEG",
        "VCCD", "VCCD1", "VCCD2", "VSSD", "VSSD1", "VSSD2",
        "VDDIO", "VSSIO", "VDDPST", "VSSPST", "KAPWR", "VSWITCH", "VCCHIB",
        "VNW", "VPW", "VNWELL", "VPWELL", "VWELL", "VSUBS", // wells / substrate
        "VBP", "VBN", "VBB", "VBG", "VB", "VBODY",          // body bias
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

#[derive(Debug, Clone)]
pub struct PathNode {
    pub label: String,
    pub arrival: f64,
    pub slew: f64,
}

#[derive(Debug, Clone)]
pub struct TimingReport {
    pub wns: f64,           // setup worst negative slack (ns); >0 means met
    pub tns: f64,           // setup total negative slack over endpoints (ns)
    pub endpoints: usize,
    pub worst_endpoint: String,
    pub worst_path: Vec<PathNode>,
    // hold (early / min-delay path) — only meaningful when there are flop endpoints
    pub whs: f64,           // worst hold slack (ns); >0 means met
    pub ths: f64,           // total hold negative slack (ns)
    pub hold_endpoints: usize,
    pub worst_hold_endpoint: String,
    pub worst_hold_path: Vec<PathNode>,
    // every hold endpoint and its hold slack (node index, slack ns) — the per-endpoint
    // list a hold-fix ECO ranks candidate delay insertions from. Consistent with whs/ths.
    pub hold_slacks: Vec<(PinId, f64)>,
    // path-based analysis: worst setup slack after re-timing critical paths with
    // path-local slews (Some only when `pba` is enabled). Catches non-greedy worst
    // paths the graph-based max misses.
    pub pba_wns: Option<f64>,
}

enum EdgeKind {
    Net(usize), // net index — delay looked up from the per-net delay table
    Cell(Box<Arc>),
}

struct Edge {
    to: usize,
    kind: EdgeKind,
}

struct Net {
    driver: Option<usize>,
    sinks: Vec<usize>,
    load: f64,
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
    slew: Vec<f64>,            // setup-corner output slew, ns
    arr_min: Vec<f64>,         // earliest (hold) arrival, ns
    node_load: Vec<f64>,       // driver-node capacitive load, pF
    endpoint_req: Vec<f64>,    // required time (meaningful at setup endpoints)
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
        slew: &[f64],
        arr_min: &[f64],
        node_load: &[f64],
        endpoint_req: &[f64],
    ) -> Timing {
        Timing {
            label2idx: labels.iter().enumerate().map(|(i, l)| (l.clone(), i)).collect(),
            labels: labels.to_vec(),
            is_endpoint: is_endpoint.to_vec(),
            excluded_setup: excluded_setup.to_vec(),
            arrival: arrival.to_vec(),
            slew: slew.to_vec(),
            arr_min: arr_min.to_vec(),
            node_load: node_load.to_vec(),
            endpoint_req: endpoint_req.to_vec(),
        }
    }
}

/// A netlist edit an optimizer stages on a [`Timer`]. (v0: cell swap — resize / Vt-swap;
/// buffer insertion and removal land with the topology-aware recompute.)
pub enum Move {
    /// Replace an instance's library cell with another (same logic function): the resize /
    /// Vt-swap move. `inst` is the instance name; `cell` the new library cell name.
    Resize { inst: String, cell: String },
}

/// An opaque saved state for speculative apply/undo: `checkpoint()` captures it, `restore()`
/// rolls back to it (a swap of the cached state + working netlist; the incremental graph
/// state, when present, rolls back with it so speculation stays cheap).
pub struct Checkpoint {
    nl: Netlist,
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
    spef: Option<Spef>,
    dirty: bool, // a mutation is staged but not yet folded into the cached report
    // incremental fast path (Some only in the simple timing context — see `inc`).
    inc: Option<IncGraph>,
    // cell swaps staged since the last `update`: inst → (original cell, current cell). The
    // original is the cell as of the last update, so a re-staged instance keeps one delta.
    pending: HashMap<String, (String, String)>,
    n_inc: u64,  // updates served by the cone-localized fast path
    n_full: u64, // updates that fell back to a full re-analysis
}

impl Timer {
    /// Build the timing state, run the analysis once, and retain the inputs for mutation. `O(N)`.
    pub fn build(
        nl: &Netlist,
        lib: &Lib,
        job: &StaJob,
        spef: Option<&Spef>,
    ) -> Result<Timer, StaError> {
        let (report, timing, inc) = build_report(nl, lib, job, spef, true)?;
        Ok(Timer {
            report,
            timing,
            nl: nl.clone(),
            lib: lib.clone(),
            job: job.clone(),
            spef: spef.cloned(),
            dirty: false,
            inc,
            pending: HashMap::new(),
            n_inc: 0,
            n_full: 0,
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
        self.timing.arrival.get(p).copied().unwrap_or(f64::NEG_INFINITY)
    }
    /// Earliest (hold / early) arrival time at `p`, ns.
    pub fn arrival_min(&self, p: PinId) -> f64 {
        self.timing.arr_min.get(p).copied().unwrap_or(f64::INFINITY)
    }
    /// Setup-corner output slew at `p`, ns.
    pub fn slew(&self, p: PinId) -> f64 {
        self.timing.slew.get(p).copied().unwrap_or(0.0)
    }
    /// Capacitive load on `p` when it drives a net, pF (0 if `p` is not a net driver).
    pub fn load(&self, p: PinId) -> f64 {
        self.timing.node_load.get(p).copied().unwrap_or(0.0)
    }
    /// Required time at `p` — `Some` only at a reached, non-false-path setup endpoint.
    pub fn required(&self, p: PinId) -> Option<f64> {
        let excluded = self.timing.excluded_setup.get(p).copied().unwrap_or(false);
        (self.is_endpoint(p) && self.arrival(p).is_finite() && !excluded)
            .then(|| self.timing.endpoint_req[p])
    }
    /// Setup slack at `p` (`required − arrival`) — `Some` only at a setup endpoint.
    pub fn slack(&self, p: PinId) -> Option<f64> {
        self.required(p).map(|r| r - self.arrival(p))
    }
    /// Every setup endpoint and its slack, worst (most negative) first — the list an
    /// optimizer ranks candidate moves from. Consistent with `wns`/`tns`.
    pub fn endpoint_slacks(&self) -> Vec<(PinId, f64)> {
        let t = &self.timing;
        let mut v: Vec<(PinId, f64)> = (0..t.labels.len())
            .filter(|&p| t.is_endpoint[p] && t.arrival[p].is_finite() && !t.excluded_setup[p])
            .map(|p| (p, t.endpoint_req[p] - t.arrival[p]))
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

    // ---- mutation + update (Phase 2) ----

    /// The current (possibly mutated) working netlist — materialize the resized design.
    pub fn netlist(&self) -> &Netlist {
        &self.nl
    }

    /// Stage a netlist mutation. Returns `false` if it doesn't apply (e.g. unknown instance).
    /// Nothing is recomputed until [`update`](Self::update); the cached report/queries reflect
    /// the *last updated* state until then.
    pub fn stage(&mut self, m: Move) -> bool {
        match m {
            Move::Resize { inst, cell } => match self.nl.insts.iter_mut().find(|i| i.name == inst) {
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
            },
        }
    }

    /// Convenience for the common move: swap `inst`'s library cell (resize / Vt-swap).
    pub fn resize(&mut self, inst: &str, cell: &str) -> bool {
        self.stage(Move::Resize { inst: inst.to_string(), cell: cell.to_string() })
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
            build_report(&self.nl, &self.lib, &self.job, self.spef.as_ref(), true)?;
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
    Ok(build_report(nl, lib, job, spef, false)?.0)
}

/// Build the timing graph, propagate arrival/required, and return the report.
///
/// (Phase 1+ splits this into persistent graph construction + propagation that the
/// [`Timer`] retains for per-pin queries and incremental dirty-cone updates.)
fn build_report(
    nl: &Netlist,
    lib: &Lib,
    job: &StaJob,
    spef: Option<&Spef>,
    capture: bool,
) -> Result<(TimingReport, Timing, Option<IncGraph>), StaError> {
    let mut labels: Vec<String> = Vec::new();
    let mut key2idx: HashMap<String, usize> = HashMap::new();
    let mut is_endpoint: Vec<bool> = Vec::new();

    let node = |key: String, label: String, key2idx: &mut HashMap<String, usize>,
                    labels: &mut Vec<String>, is_endpoint: &mut Vec<bool>| -> usize {
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
        nets.entry(n.to_string()).or_insert(Net { driver: None, sinks: Vec::new(), load: 0.0 });
    };

    // primary input ports drive a net of the same name
    let mut input_ports: Vec<(usize, String)> = Vec::new();
    for p in &nl.inputs {
        let idx = node(port_key(p), p.clone(), &mut key2idx, &mut labels, &mut is_endpoint);
        ensure_net(&mut nets, p);
        nets.get_mut(p).unwrap().driver = Some(idx);
        input_ports.push((idx, p.clone()));
    }
    // primary output ports are endpoints + sinks of their net
    let mut output_ports: Vec<(usize, String)> = Vec::new();
    for p in &nl.outputs {
        let idx = node(port_key(p), p.clone(), &mut key2idx, &mut labels, &mut is_endpoint);
        is_endpoint[idx] = true;
        ensure_net(&mut nets, p);
        let net = nets.get_mut(p).unwrap();
        net.sinks.push(idx);
        net.load += job.output_load;
        output_ports.push((idx, p.clone()));
    }

    // instance pins. A sequential cell's data pins (with a setup constraint)
    // are *capture* endpoints; its Q launches via the CK->Q delay arc.
    // (D pin node, constraint table(s), this flop's CK pin key) — the constraint is
    // interpolated at the operating slews later; the CK key resolves to a node whose
    // arrival is the clock insertion delay to this flop (skew).
    let mut flop_d: Vec<(usize, Vec<Constraint>, Option<String>)> = Vec::new();
    let mut flop_hold: Vec<(usize, Vec<Constraint>, Option<String>)> = Vec::new();
    let mut ck_node_list: Vec<usize> = Vec::new(); // nodes that are clock (CK) pins
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
            ensure_net(&mut nets, net);
            match cell.pins.get(pin).map(|p| p.direction) {
                Some(Dir::Out) => {
                    nets.get_mut(net).unwrap().driver.get_or_insert(idx);
                }
                Some(Dir::In) => {
                    // Miller-aware receiver load when the pin carries a CCS receiver
                    // model, else the static `capacitance`.
                    let cap = cell.pins[pin].load_cap();
                    let nref = nets.get_mut(net).unwrap();
                    nref.sinks.push(idx);
                    nref.load += cap;
                    if cell.pins[pin].clock {
                        ck_node_list.push(idx); // clock pin — root of insertion-delay paths
                    }
                    if cell.is_seq {
                        if !cell.pins[pin].setup.is_empty() {
                            is_endpoint[idx] = true; // data pin = setup capture endpoint
                            flop_d.push((idx, cell.pins[pin].setup.clone(), ck_key.clone()));
                        }
                        if !cell.pins[pin].hold.is_empty() {
                            flop_hold.push((idx, cell.pins[pin].hold.clone(), ck_key.clone()));
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
    let mut seed = vec![0.0f64; n];
    for (idx, name) in &input_ports {
        if !clock_srcs.contains(name) {
            seed[*idx] = job.input_delay_for(name);
        }
    }
    for (idx, name) in &output_ports {
        endpoint_req[*idx] = period - job.output_delay_for(name) - job.setup_uncertainty;
    }
    let mut node_load = vec![0.0f64; n];
    for (netname, net) in &nets {
        if let Some(d) = net.driver {
            // driver load = receiver pin caps + wire cap from SPEF (fF -> pF)
            let wire = spef.map(|s| s.wire_load_pf(netname)).unwrap_or(0.0);
            node_load[d] = net.load + wire;
        }
    }

    // ---- pass 2: edges ---------------------------------------------------
    let mut out_edges: Vec<Vec<Edge>> = (0..n).map(|_| Vec::new()).collect();
    let mut indeg = vec![0usize; n];

    // cell arcs: related input pin -> output pin (within an instance)
    for inst in &nl.insts {
        let Some(cell) = lib.cell(&inst.cell) else {
            continue; // physical-only cell skipped in pass 1
        };
        let conn: HashMap<&str, &str> =
            inst.conns.iter().map(|(p, net)| (p.as_str(), net.as_str())).collect();
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
                out_edges[i_idx].push(Edge { to: o_idx, kind: EdgeKind::Cell(Box::new(arc.clone())) });
                indeg[o_idx] += 1;
            }
        }
    }
    // index the nets so net arcs can look up a (mutable, per-pass) delay table
    let net_order: Vec<String> = nets.keys().cloned().collect();
    let net_idx: HashMap<&str, usize> =
        net_order.iter().enumerate().map(|(i, nm)| (nm.as_str(), i)).collect();
    let nn = net_order.len();
    let mut net_res = vec![0.0f64; nn];
    let mut net_cap = vec![0.0f64; nn];
    let mut net_drv: Vec<Option<usize>> = vec![None; nn];
    let mut net_drv_ip: Vec<Option<(String, String)>> = vec![None; nn]; // driver (inst, pin)
    let mut net_cpl: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nn]; // (aggressor net idx, Cc)
    let ip_of = |node: usize| -> Option<(String, String)> {
        labels[node].split_once('/').map(|(a, b)| (a.to_string(), b.to_string()))
    };
    for (name, net) in &nets {
        let i = net_idx[name.as_str()];
        net_drv[i] = net.driver;
        net_drv_ip[i] = net.driver.and_then(ip_of);
        if let Some(rc) = spef.and_then(|s| s.nets.get(name)) {
            net_res[i] = rc.res_ohm;
            net_cap[i] = rc.cap_ff;
            for (agg, cc) in &rc.coupling {
                if let Some(&ai) = net_idx.get(agg.as_str()) {
                    net_cpl[i].push((ai, *cc));
                }
            }
        }
    }

    // effective-capacitance shielding per driver node (CCS-into-RC): the driver
    // sees a near cap + a far cap shielded behind the wire resistance, so its cell
    // delay is computed at Ceff < total on resistive nets.
    let mut shield: Vec<Option<(f64, f64)>> = vec![None; n]; // driver node -> (C1 pF, tau ns)
    if let Some(s) = spef {
        for (name, net) in &nets {
            if let Some(d) = net.driver {
                let i = net_idx[name.as_str()];
                if let (Some(rc), Some((di, dp))) = (s.nets.get(name), &net_drv_ip[i]) {
                    if let Some(dnode) = rc.pin_node(di, dp) {
                        if let Some((c1_ff, tau)) = rc.pi_reduce(dnode) {
                            shield[d] = Some((c1_ff / 1000.0, tau));
                        }
                    }
                }
            }
        }
    }

    // net arcs: driver -> each sink. Each arc gets its own delay (per-pin Elmore),
    // indexed by an arc id. Record the sink's (inst,pin) to resolve its SPEF node.
    struct ArcInfo {
        net_idx: usize,
        sink_ip: Option<(String, String)>,
    }
    let mut arcs: Vec<ArcInfo> = Vec::new();
    for (name, net) in &nets {
        if let Some(d) = net.driver {
            let i = net_idx[name.as_str()];
            for &s in &net.sinks {
                if s != d {
                    let aid = arcs.len();
                    arcs.push(ArcInfo { net_idx: i, sink_ip: ip_of(s) });
                    out_edges[d].push(Edge { to: s, kind: EdgeKind::Net(aid) });
                    indeg[s] += 1;
                }
            }
        }
    }
    let n_arcs = arcs.len();

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
            p.arcs.iter().any(|a| !a.sigma_rise.values.is_empty() || !a.sigma_fall.values.is_empty())
        })
    });
    let pocv = job.pocv_sigma > 0.0 || has_lvf;
    let aocv = !pocv && (!job.aocv_late.is_empty() || !job.aocv_early.is_empty());
    // The incremental fast path ([`Timer::update`]) is eligible only in the simple timing
    // context: ideal interconnect (no SPEF/SI), flat derate (no AOCV/POCV bands), and no
    // path-based pass. Outside it we don't capture the graph and `update` does a full pass.
    let simple_ctx = capture && spef.is_none() && !pocv && !aocv && !job.pba;
    let mut inc_setup: Vec<SetupRec> = Vec::new();
    let mut inc_hold: Vec<HoldRec> = Vec::new();
    let pocv_sigma = job.pocv_sigma;
    let n_sigma = job.pocv_n;
    // per-cell-stage derate on the nominal delay (1.0 for POCV — it uses sigma)
    let cell_derate = |late: bool, stage: usize| -> f64 {
        if aocv {
            aocv_lookup(if late { &job.aocv_late } else { &job.aocv_early }, stage as f64)
        } else if pocv {
            1.0
        } else if late {
            late_derate
        } else {
            early_derate
        }
    };

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
    // `nd` = per-arc net delay, `ns` = per-arc degraded sink slew (0 = keep driver slew).
    #[allow(clippy::type_complexity)]
    let relax = |nd: &[f64], ns: &[f64], late: bool| -> (Vec<f64>, Vec<f64>, Vec<Option<usize>>, Vec<usize>, Vec<[f64; 2]>, Vec<[f64; 2]>, Vec<[Option<usize>; 2]>) {
        let init = if late { f64::NEG_INFINITY } else { f64::INFINITY };
        let mut arr = vec![[init; 2]; n]; // per-lane metric (derated / +-N*sigma)
        let mut arr_nom = vec![[0.0f64; 2]; n];
        let mut var = vec![[0.0f64; 2]; n];
        let mut depth = vec![[0usize; 2]; n];
        let mut slew = vec![[input_slew; 2]; n];
        let mut from: Vec<[Option<usize>; 2]> = vec![[None; 2]; n];
        let mut indeg_work = indeg.clone();
        let mut order: Vec<usize> = Vec::new();
        for v in 0..n {
            if indeg_work[v] == 0 {
                arr[v] = [seed[v]; 2]; // input ports seed with their SDC arrival delay
                arr_nom[v] = [seed[v]; 2];
                order.push(v);
            }
        }
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for e in &out_edges[u] {
                let v = e.to;
                let load = node_load[v];
                match &e.kind {
                    EdgeKind::Net(i) => {
                        // interconnect: rise->rise, fall->fall, same delay, no derate.
                        // The sink slew is the transient-degraded value when available
                        // (>0), else the driver slew passes through unchanged.
                        let d = nd[*i];
                        let sink_slew = ns[*i];
                        for l in 0..2 {
                            let a = arr[u][l];
                            if !a.is_finite() {
                                continue;
                            }
                            let metric = a + d; // band carries (no new variance)
                            let better =
                                if late { metric > arr[v][l] } else { metric < arr[v][l] };
                            if better {
                                arr[v][l] = metric;
                                arr_nom[v][l] = arr_nom[u][l] + d;
                                var[v][l] = var[u][l];
                                depth[v][l] = depth[u][l];
                                slew[v][l] =
                                    if sink_slew > 0.0 { sink_slew } else { slew[u][l] };
                                from[v][l] = Some(u);
                            }
                        }
                    }
                    EdgeKind::Cell(arc) => {
                        for ol in 0..2 {
                            let (dt, st) = if ol == 0 {
                                (&arc.cell_rise, &arc.rise_transition)
                            } else {
                                (&arc.cell_fall, &arc.fall_transition)
                            };
                            for il in 0..2 {
                                // does input edge `il` drive output edge `ol`?
                                let feeds = match arc.sense.as_str() {
                                    "positive_unate" => il == ol,
                                    "negative_unate" => il != ol,
                                    _ => true, // non_unate / unknown: worst of both
                                };
                                if !feeds {
                                    continue;
                                }
                                let a_in = arr[u][il];
                                if !a_in.is_finite() {
                                    continue;
                                }
                                let sin = slew[u][il];
                                // CCS-into-RC: drive the effective capacitance (resistive
                                // shielding), iterating Ceff <-> output transition to a
                                // self-consistent point rather than a single lumped pass.
                                let leff = match shield[v] {
                                    Some((c1, tau)) => crate::ccs::ceff_iter(
                                        c1,
                                        load - c1,
                                        tau,
                                        |c| st.lookup(sin, c),
                                    ),
                                    None => load,
                                };
                                // CCS current-source delay when the arc carries it; else NLDM.
                                let (d, sout) = if !arc.ccs.is_empty() {
                                    arc.ccs
                                        .delay_slew(ol == 0, sin, leff, 0.3, 0.7)
                                        .unwrap_or((dt.lookup(sin, leff), st.lookup(sin, leff)))
                                } else {
                                    (dt.lookup(sin, leff), st.lookup(sin, leff))
                                };
                                let stage = depth[u][il] + 1;
                                let derate = cell_derate(late, stage);
                                let nom = arr_nom[u][il] + d * derate;
                                // per-stage delay sigma: LVF table (slew/load-dependent)
                                // when present, else the global pocv_sigma fraction.
                                let sigma = if !pocv {
                                    0.0
                                } else {
                                    let lvf = if ol == 0 { &arc.sigma_rise } else { &arc.sigma_fall };
                                    if !lvf.values.is_empty() {
                                        lvf.lookup(sin, leff)
                                    } else {
                                        pocv_sigma * d
                                    }
                                };
                                let var_c = var[u][il] + sigma * sigma;
                                let metric = if pocv {
                                    let band = n_sigma * var_c.sqrt();
                                    if late { nom + band } else { nom - band }
                                } else {
                                    nom
                                };
                                let better =
                                    if late { metric > arr[v][ol] } else { metric < arr[v][ol] };
                                if better {
                                    arr[v][ol] = metric;
                                    arr_nom[v][ol] = nom;
                                    var[v][ol] = var_c;
                                    depth[v][ol] = stage;
                                    slew[v][ol] = sout;
                                    from[v][ol] = Some(u);
                                }
                            }
                        }
                    }
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
                if a[0] >= a[1] { 0 } else { 1 }
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
            let l = pick(arr[v]);
            arrival[v] = arr[v][l];
            slew_c[v] = slew[v][l];
            from_c[v] = from[v][l];
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
    let compute = |sw: &[f64], net_slew: &[f64]| -> (Vec<f64>, Vec<f64>) {
        // per-net crosstalk cap (fF) from window-overlapping aggressors
        let xc: Vec<f64> = (0..nn)
            .map(|i| {
                let svi = sw[i];
                let mut x = 0.0;
                if svi.is_finite() {
                    for &(ai, cc) in &net_cpl[i] {
                        let window = (net_slew[i] + net_slew[ai]) / 2.0 + guard;
                        if sw[ai].is_finite() && (sw[ai] - svi).abs() <= window {
                            x += (miller - 1.0).max(0.0) * cc;
                        }
                    }
                }
                x
            })
            .collect();
        // waveform-into-RC: a transient response per net (driver-slew-aware), memoized
        // so each sink reads the same simulation. Falls back to per-pin Elmore.
        let tr_net: Vec<Option<BTreeMap<String, (f64, f64)>>> = (0..nn)
            .map(|i| {
                let rc = spef?.nets.get(&net_order[i])?;
                let (di, dp) = net_drv_ip[i].as_ref()?;
                let dn = rc.pin_node(di, dp)?;
                rc.transient(dn, net_slew[i], xc[i])
            })
            .collect();
        // each arc -> (net delay, degraded sink slew); slew 0 means "keep driver slew"
        arcs.iter()
            .map(|a| {
                let i = a.net_idx;
                let Some(rc) = spef.and_then(|s| s.nets.get(&net_order[i])) else {
                    return (0.0, 0.0); // no parasitics -> ideal interconnect
                };
                // transient (waveform-into-RC) delay + degraded slew at the sink
                if let (Some(map), Some((si, sp))) = (&tr_net[i], &a.sink_ip) {
                    if let Some(sn) = rc.pin_node(si, sp) {
                        if let Some(&(delay, slew)) = map.get(sn) {
                            return (delay, slew);
                        }
                    }
                }
                // fallback: per-pin tree Elmore when driver + sink map to SPEF nodes
                if let (Some((di, dp)), Some((si, sp))) = (&net_drv_ip[i], &a.sink_ip) {
                    if let (Some(dt), Some(st)) = (rc.pin_node(di, dp), rc.pin_node(si, sp)) {
                        if let Some(dl) = rc.elmore(dt, xc[i]) {
                            if let Some(&v) = dl.get(st) {
                                return (v, 0.0);
                            }
                        }
                    }
                }
                // last resort: lumped Elmore (R·C) + lumped crosstalk (R·xtalk-cap)
                (net_res[i] * net_cap[i] * 1e-6 + net_res[i] * xc[i] * 1e-6, 0.0)
            })
            .unzip()
    };

    const MAX_SI_ITERS: usize = 20;
    const SI_TOL: f64 = 1e-9; // ns — per-arc delay change below which we stop
    let neg = vec![f64::NEG_INFINITY; nn];
    let zero = vec![0.0f64; nn];
    let (nom_d, nom_s) = compute(&neg, &zero); // nominal: no windows -> no crosstalk
    let mut arc_d = nom_d.clone();
    let mut arc_s = nom_s.clone();
    let mut cycle_checked = false;
    for _ in 0..MAX_SI_ITERS {
        let (arr, slw, _f, ord, _, _, _) = relax(&arc_d, &arc_s, true);
        if !cycle_checked {
            if ord.len() != n {
                return Err(StaError::CombinationalLoop);
            }
            cycle_checked = true;
        }
        let sw: Vec<f64> =
            (0..nn).map(|i| net_drv[i].map(|d| arr[d]).unwrap_or(f64::NEG_INFINITY)).collect();
        let net_slew: Vec<f64> =
            (0..nn).map(|i| net_drv[i].map(|d| slw[d]).unwrap_or(0.0)).collect();
        let (nd, ns) = compute(&sw, &net_slew);
        let delta = (0..n_arcs).map(|k| (nd[k] - arc_d[k]).abs()).fold(0.0, f64::max);
        arc_d = nd;
        arc_s = ns;
        if delta < SI_TOL {
            break;
        }
    }
    // final late propagation consistent with the converged per-arc delays, and the
    // early (min-delay) propagation used for hold and for early clock arrivals.
    let (arrival, slew, from, order_late, late_arr, late_slew, late_from) =
        relax(&arc_d, &arc_s, true);
    let (arr_min, slew_min, from_min, _ord_min, early_arr, early_slew, early_from) =
        relax(&nom_d, &nom_s, false);

    // ---- clock paths + CRPR ---------------------------------------------
    // Proper OCV uses opposite clock corners on launch vs capture (setup: late
    // launch / early capture; hold: early launch / late capture). The launch and
    // capture clock paths share a segment from the root to their common point;
    // deriving that shared segment two ways is unphysical pessimism, so CRPR credits
    // back its OCV spread (late arrival − early arrival at the common point).
    let mut is_ck = vec![false; n];
    for &i in &ck_node_list {
        is_ck[i] = true;
    }
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
    let crpr_credit = |lck: usize, cck: usize| -> f64 {
        if !crpr_on {
            return 0.0;
        }
        common_point(lck, cck).map(|p| (arrival[p] - arr_min[p]).max(0.0)).unwrap_or(0.0)
    };

    // ---- multi-clock: which clock reaches each flop, and the launch→capture
    // edge relation between two clock domains ------------------------------
    // Map each clock's source node to its period (a single-clock job synthesizes
    // one entry from clock_port/period_ns). A generated/divided clock is just an
    // entry whose source is an internal pin.
    let eff_clocks: Vec<(String, f64)> = if job.clocks.is_empty() {
        vec![(job.clock_port.clone(), period)]
    } else {
        job.clocks.iter().map(|(_, src, per)| (src.clone(), *per)).collect()
    };
    let mut clock_src: HashMap<usize, f64> = HashMap::new();
    for (src, per) in &eff_clocks {
        let node = match src.split_once('/') {
            Some((inst, pin)) => key2idx.get(&pin_key(inst, pin)).copied(),
            None => key2idx.get(&port_key(src)).copied(),
        };
        if let Some(nd) = node {
            clock_src.insert(nd, *per);
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
    // (setup, hold) launch→capture edge relation between launch period `pl` and
    // capture period `pc`. Same clock → (pc, 0) (the common case). Else scan launch
    // edges over a common multiple: setup = tightest positive capture-after-launch,
    // hold = the capture edge one period earlier (worst across launches).
    let edge_relation = |pl: f64, pc: f64| -> (f64, f64) {
        if (pl - pc).abs() < 1e-9 {
            return (pc, 0.0);
        }
        let hyper = pl * pc; // a common multiple of both ( >= LCM )
        let mut setup_rel = f64::INFINITY;
        let mut hold_rel = f64::NEG_INFINITY;
        let mut j = 0usize;
        loop {
            let le = j as f64 * pl;
            if le >= hyper || j > 100_000 {
                break;
            }
            let k = (le / pc).floor() + 1.0; // first capture edge strictly after le
            let ce = k * pc;
            setup_rel = setup_rel.min(ce - le);
            hold_rel = hold_rel.max((ce - pc) - le);
            j += 1;
        }
        if !setup_rel.is_finite() {
            setup_rel = pc;
        }
        (setup_rel, hold_rel)
    };

    // worst (max) constraint over a pin's groups, interpolated at the operating
    // clock + data transitions — matches how delay arcs are looked up.
    let eval_cons = |cons: &[Constraint], clk_slew: f64, data_slew: f64| -> f64 {
        cons.iter().map(|c| c.eval(clk_slew, data_slew)).fold(f64::NEG_INFINITY, f64::max)
    };

    // timing exceptions, matched on launch/capture instance (or port) names.
    let inst_of = |node: usize| labels[node].split('/').next().unwrap_or("").to_string();
    let match_exc = |ln: &str, cn: &str| {
        job.exceptions
            .iter()
            .find(|e| (e.from == "*" || e.from == ln) && (e.to == "*" || e.to == cn))
    };
    let mut excluded_setup = vec![false; n]; // false-path endpoints (skip setup)

    // setup capture uses the EARLY clock; CRPR adds back the shared-path pessimism;
    // the capture window is the launch→capture edge relation (one period intra-domain),
    // shifted out by a multicycle exception (false paths drop the endpoint).
    for (idx, setup, ck) in &flop_d {
        let cap = ck_node(ck);
        let lck = launch_ck(*idx, &from);
        let cap_early = cap.map(|i| arr_min[i]).filter(|a| a.is_finite()).unwrap_or(0.0);
        let crpr = match (lck, cap) {
            (Some(l), Some(c)) => crpr_credit(l, c),
            _ => 0.0,
        };
        let ck_slew = cap.map(|i| slew[i]).unwrap_or(input_slew);
        let setup_v = eval_cons(setup, ck_slew, slew[*idx]);
        let pc = cap.map(&clock_period_of).unwrap_or(period);
        let pl = lck.map(&clock_period_of).unwrap_or(pc);
        let (mut setup_rel, _) = edge_relation(pl, pc);
        let ln = lck.map(&inst_of).unwrap_or_default();
        if let Some(e) = match_exc(&ln, &inst_of(*idx)) {
            match e.kind {
                ExcKind::FalsePath => excluded_setup[*idx] = true,
                ExcKind::Multicycle(cyc) => setup_rel += cyc.saturating_sub(1) as f64 * pc,
            }
        }
        let base = cap_early + setup_rel + crpr - job.setup_uncertainty;
        endpoint_req[*idx] = base - setup_v;
        // capture the clock-side constants so the fast path can re-derive required time
        // from the (possibly changed) data slew without re-walking the clock network.
        if simple_ctx && !excluded_setup[*idx] {
            inc_setup.push(SetupRec { idx: *idx, base, cons: setup.clone(), ck_slew, launch_ck: lck });
        }
    }

    // ---- setup slack + worst path ---------------------------------------
    // Each endpoint's required time is fixed (period at outputs, period - setup at
    // flop D), so slack = required - latest arrival; no backward pass needed.
    let mut wns = f64::INFINITY;
    let mut tns = 0.0;
    let mut worst = None;
    let mut endpoints = 0;
    for v in 0..n {
        if !is_endpoint[v] || arrival[v] == f64::NEG_INFINITY || excluded_setup[v] {
            continue; // unreached or false-path-excluded
        }
        endpoints += 1;
        let slack = endpoint_req[v] - arrival[v];
        if slack < 0.0 {
            tns += slack;
        }
        if slack < wns {
            wns = slack;
            worst = Some(v);
        }
    }

    let mut worst_path = Vec::new();
    let worst_endpoint = match worst {
        Some(mut v) => {
            let end_label = labels[v].clone();
            let mut chain = vec![v];
            while let Some(u) = from[v] {
                chain.push(u);
                v = u;
            }
            chain.reverse();
            for idx in chain {
                worst_path.push(PathNode {
                    label: labels[idx].clone(),
                    arrival: arrival[idx],
                    slew: slew[idx],
                });
            }
            end_label
        }
        None => String::new(),
    };

    // ---- path-based analysis (optional) ---------------------------------
    // Re-time the worst path and its 1-exchange fan-in alternatives with strictly
    // path-local slew (flat late derate; AOCV-depth not re-applied). Catches a
    // non-greedy worst path: where GBA's local max-arrival pick at a reconvergent
    // node took the faster-slew fan-in but a slower-slew one is worse downstream.
    let pba_wns = if job.pba {
        let mut preds: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (u, edges) in out_edges.iter().enumerate() {
            for e in edges {
                preds[e.to].push(u);
            }
        }
        let cell_dly = |arc: &Arc, sin: f64, vnode: usize| -> (f64, f64) {
            let cl = node_load[vnode];
            let leff = match shield[vnode] {
                Some((c1, tau)) => crate::ccs::ceff_iter(c1, cl - c1, tau, |c| {
                    arc.rise_transition.lookup(sin, c).max(arc.fall_transition.lookup(sin, c))
                }),
                None => cl,
            };
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
                    arc.cell_rise.lookup(sin, leff).max(arc.cell_fall.lookup(sin, leff)),
                    arc.rise_transition.lookup(sin, leff).max(arc.fall_transition.lookup(sin, leff)),
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
                        t += arc_d[*i];
                        if arc_s[*i] > 0.0 {
                            s = arc_s[*i];
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
                    let alt_path: Vec<usize> =
                        prefix(alt).iter().chain(gba_path[wi..].iter()).copied().collect();
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
    for (idx, hold, ck) in &flop_hold {
        let idx = *idx;
        if arr_min[idx] == f64::INFINITY {
            continue; // unreached
        }
        let cap = ck_node(ck);
        let lck = launch_ck(idx, &from_min);
        let pc = cap.map(&clock_period_of).unwrap_or(period);
        let pl = lck.map(&clock_period_of).unwrap_or(pc);
        let (_, mut hold_rel) = edge_relation(pl, pc);
        // apply exceptions: false path drops the endpoint, multicycle shifts the
        // hold capture back by (cycles − 1).
        let ln = lck.map(&inst_of).unwrap_or_default();
        if let Some(e) = match_exc(&ln, &inst_of(idx)) {
            match e.kind {
                ExcKind::FalsePath => continue,
                ExcKind::Multicycle(cyc) => hold_rel += cyc.saturating_sub(1) as f64 * pc,
            }
        }
        hold_endpoints += 1;
        let cap_late = cap.map(|i| arrival[i]).filter(|a| a.is_finite()).unwrap_or(0.0);
        let crpr = match (lck, cap) {
            (Some(l), Some(c)) => crpr_credit(l, c),
            _ => 0.0,
        };
        let ck_slew = cap.map(|i| slew_min[i]).unwrap_or(input_slew);
        let hold_v = eval_cons(hold, ck_slew, slew_min[idx]);
        let base = crpr - cap_late - hold_rel - job.hold_uncertainty;
        // earliest data must arrive after the (late) capture edge + hold relation
        let slack = arr_min[idx] + base - hold_v;
        hold_slacks.push((idx, slack));
        if simple_ctx {
            inc_hold.push(HoldRec { idx, base, cons: hold.clone(), ck_slew, launch_ck: lck });
        }
        if slack < 0.0 {
            ths += slack;
        }
        if slack < whs {
            whs = slack;
            worst_hold = Some(idx);
        }
    }
    let mut worst_hold_path = Vec::new();
    let worst_hold_endpoint = match worst_hold {
        Some(mut v) => {
            let end_label = labels[v].clone();
            let mut chain = vec![v];
            while let Some(u) = from_min[v] {
                chain.push(u);
                v = u;
            }
            chain.reverse();
            for idx in chain {
                worst_hold_path.push(PathNode {
                    label: labels[idx].clone(),
                    arrival: arr_min[idx],
                    slew: slew_min[idx],
                });
            }
            end_label
        }
        None => String::new(),
    };

    let report = TimingReport {
        wns: if endpoints == 0 { f64::INFINITY } else { wns },
        tns,
        endpoints,
        worst_endpoint,
        worst_path,
        whs: if hold_endpoints == 0 { f64::INFINITY } else { whs },
        ths,
        hold_endpoints,
        worst_hold_endpoint,
        worst_hold_path,
        hold_slacks,
        pba_wns,
    };
    // Retain a per-pin snapshot for the Timer's query API (Phase 1). Cloned, not moved:
    // closures above still hold immutable borrows of these arrays through the return.
    let timing = Timing::new(
        &labels,
        &is_endpoint,
        &excluded_setup,
        &arrival,
        &slew,
        &arr_min,
        &node_load,
        &endpoint_req,
    );

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
                    EdgeKind::Cell(arc) => InEdge::Cell { from: u, arc: arc.clone() },
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
            label2idx: labels.iter().enumerate().map(|(i, l)| (l.clone(), i)).collect(),
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
            late: Lanes { arr: late_arr, slew: late_slew, from: late_from },
            early: Lanes { arr: early_arr, slew: early_slew, from: early_from },
            arrival: arrival.clone(),
            slew: slew.clone(),
            from: from.clone(),
            arr_min: arr_min.clone(),
            slew_min: slew_min.clone(),
            from_min: from_min.clone(),
            endpoint_req: endpoint_req.clone(),
            overrides: HashMap::new(),
        };
        Some(IncGraph { topo, state })
    } else {
        None
    };
    Ok((report, timing, inc))
}
