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
//! fan-in alternatives (path-local slew), without GBA's merged worst slew. Pure std
//! — unit-tested.

use std::collections::{BTreeMap, HashMap};

use crate::inc::{checks_edge, HoldRec, InEdge, IncGraph, IncState, IncTopo, Lanes, Pred, SetupRec};
use crate::job::{ExcKind, Exception, StaJob};
use crate::liberty::{Arc, ClkEdge, Constraint, Dir, Lib};
use crate::names::{instance_of, split_inst_pin};
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
                write!(
                    f,
                    "combinational loop (sequential timing not modeled in v0)"
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

/// The reference's comparison tolerance (`Fuzzy.cc`), in its `float` arithmetic.
fn fuzzy_equal(v1: f32, v2: f32) -> bool {
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
pub(crate) fn cycle_accting(src_period: f64, src_time: f64, tgt_period: f64, tgt_time: f64) -> (f64, f64) {
    let less = |a: f32, b: f32| a < b && !fuzzy_equal(a, b);
    let less_equal = |a: f32, b: f32| a < b || fuzzy_equal(a, b);
    let greater = |a: f32, b: f32| a > b && !fuzzy_equal(a, b);
    // `MinMax::min()->initValue()`: the delays start at INF and shrink. Compared in float as the
    // reference compares them; the edges chosen are returned as their f64 difference, the
    // arithmetic the rest of this timer is in (a same-clock setup relation is the period exactly).
    let (mut setup, mut hold) = (1e30f32, 1e30f32);
    let (mut setup_t, mut hold_t) = (tgt_period, 0.0);
    if !(tgt_period > 0.0 && src_period > 0.0) {
        return (tgt_period, 0.0);
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
        let mut src_cycle = first_cycle(src_time, src_period);
        loop {
            let src_cycle_start = src_cycle as f64 * src_period;
            let src_t = src_cycle_start + src_time;
            // both found, and the two clocks back in step
            if tgt_past_src && src_past_tgt && fuzzy_equal(src_cycle_start as f32, tgt_cycle_start as f32) {
                return (setup_t, hold_t);
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
            src_cycle += 1;
        }
        tgt_cycle += 1;
    }
    // no common period within the cycle limit: what was found
    (setup_t, hold_t)
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

enum EdgeKind {
    Net(usize), // net index — delay looked up from the per-net delay table
    Cell(Box<Arc>),
}

struct Edge {
    to: usize,
    kind: EdgeKind,
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
    spef: Option<Spef>,
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
    spef: Option<&Spef>,
    capture: bool,
) -> Result<(TimingReport, Timing, Option<IncGraph>), StaError> {
    // A clock source written as a pattern (`create_clock … [get_ports *clk*]`) is every port it
    // matches, as the reference's `get_ports` resolves it. Kept literal it named no port, the
    // clock reached nothing, and every register went unchecked ("no constrained paths").
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
        for mm in 0..2 {
            for l in 0..2 {
                net.load[mm][l] += job.output_load;
            }
        }
        output_ports.push((idx, p.clone()));
    }

    // instance pins. A sequential cell's data pins (with a setup constraint)
    // are *capture* endpoints; its Q launches via the CK->Q delay arc.
    // (D pin node, constraint table(s), this flop's CK pin key) — the constraint is
    // interpolated at the operating slews later; the CK key resolves to a node whose
    // arrival is the clock insertion delay to this flop (skew).
    let mut flop_d: Vec<(usize, Vec<Constraint>, Option<String>)> = Vec::new();
    // the recovery / removal pins among them: the reference reports their checks in their own
    // path group, `asynchronous`
    let mut async_end: std::collections::HashSet<usize> = std::collections::HashSet::new();
    let mut flop_hold: Vec<(usize, Vec<Constraint>, Option<String>)> = Vec::new();
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
                        if !cell.pins[pin].setup.is_empty() {
                            is_endpoint[idx] = true; // data pin = setup capture endpoint
                            flop_d.push((idx, cell.pins[pin].setup.clone(), ck_key.clone()));
                        }
                        if !cell.pins[pin].hold.is_empty() {
                            flop_hold.push((idx, cell.pins[pin].hold.clone(), ck_key.clone()));
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
                        if !cell.pins[pin].recovery.is_empty() {
                            is_endpoint[idx] = true;
                            flop_d.push((idx, cell.pins[pin].recovery.clone(), ck_key.clone()));
                            async_end.insert(idx);
                        }
                        if !cell.pins[pin].removal.is_empty() {
                            flop_hold.push((idx, cell.pins[pin].removal.clone(), ck_key.clone()));
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
    let mut seed = vec![0.0f64; n];
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
            match job.input_delay_for(name) {
                Some(d) => seed[*idx] = d,
                None => {
                    unconstrained_in[*idx] = true;
                    unconstrained_names.push(name.as_str());
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

    for (idx, name) in &output_ports {
        endpoint_req[*idx] = period - job.output_delay_for(name) - job.setup_uncertainty;
    }
    let mut node_load = vec![[[0.0f64; 2]; 2]; n];
    for (netname, net) in &nets {
        if let Some(d) = net.driver {
            // driver load = receiver pin caps + wire cap from SPEF (fF -> the library's unit),
            // per output edge
            let wire = spef.and_then(|s| s.nets.get(netname)).map_or(0.0, |rc| rc.cap_ff / ff_per_cap);
            node_load[d] = net.load.map(|e| e.map(|c| c + wire));
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
                out_edges[i_idx].push(Edge {
                    to: o_idx,
                    kind: EdgeKind::Cell(Box::new(arc.clone())),
                });
                indeg[o_idx] += 1;
            }
        }
    }
    // index the nets so net arcs can look up a (mutable, per-pass) delay table
    let net_order: Vec<String> = nets.keys().cloned().collect();
    let net_idx: HashMap<&str, usize> = net_order
        .iter()
        .enumerate()
        .map(|(i, nm)| (nm.as_str(), i))
        .collect();
    let nn = net_order.len();
    let mut net_res = vec![0.0f64; nn];
    let mut net_cap = vec![0.0f64; nn];
    let mut net_drv: Vec<Option<usize>> = vec![None; nn];
    let mut net_drv_ip: Vec<Option<(String, String)>> = vec![None; nn]; // driver (inst, pin)
    let mut net_cpl: Vec<Vec<(usize, f64)>> = vec![Vec::new(); nn]; // (aggressor net idx, Cc)
    let ip_of = |node: usize| -> Option<(String, String)> {
        split_inst_pin(&labels[node]).map(|(a, b)| (a.to_string(), b.to_string()))
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
                if let (Some(rc), Some((di, dp))) = (s.nets.get(name), &net_drv_ip[i]) {
                    if let Some(dnode) = rc.pin_node(di, dp) {
                        // the same capacitance content the reduction sees: coupling
                        // grounded at factor 1.0, plus Liberty receiver caps at the sinks
                        let cc: f64 = net_cpl[i].iter().map(|(_, c)| *c).sum();
                        for (mm, lane) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                        let mut pcap: BTreeMap<String, f64> = BTreeMap::new();
                        for &sk in &net.sinks {
                            if let Some((si, sp)) = ip_of(sk) {
                                if let Some(nd) = rc.pin_node(&si, &sp) {
                                    *pcap.entry(nd.to_string()).or_default() +=
                                        pin_cap_node.get(&sk).map_or(0.0, |c| c[mm][lane]) * ff_per_cap;
                                }
                            } else if output_ports.iter().any(|(idx, _)| *idx == sk) {
                                // A top-level output port: its external load at its own node
                                // (`ReduceToPi::pinCapacitance` → `Sdc::portExtCap`). Without
                                // it the Pi of a net driving a port carries only the wire, and
                                // the gate sees almost no load (fft_ctrl_tlul's sram_clk_o:
                                // 0.0706 ns against sign-off's 0.2125 ns at output140).
                                *pcap.entry(labels[sk].clone()).or_default() += job.output_load * ff_per_cap;
                            }
                        }
                        if let Some((c2_ff, rpi, c1_ff)) = rc.pi_reduce(dnode, cc, &pcap) {
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
                let pi = spef.and_then(|s| s.nets.get(name)).and_then(|rc| {
                    let cc: f64 = net_cpl[i].iter().map(|(_, c)| *c).sum();
                    let mut pcap: BTreeMap<String, f64> = BTreeMap::new();
                    for &sk in &net.sinks {
                        if let Some((si, sp)) = ip_of(sk) {
                            if let Some(nd) = rc.pin_node(&si, &sp) {
                                *pcap.entry(nd.to_string()).or_default() += pin_cap_node.get(&sk).map_or(0.0, |c| c[mm][ol]) * ff_per_cap;
                            }
                        } else if output_ports.iter().any(|(o, _)| *o == sk) {
                            *pcap.entry(labels[sk].clone()).or_default() += job.output_load * ff_per_cap;
                        }
                    }
                    rc.pi_reduce(name, cc, &pcap).map(|(c2, rpi, c1)| (c2 / ff_per_cap, rpi, c1 / ff_per_cap))
                });
                for arc in arcs.iter().filter(|a| a.out_edges.drives(ol == 0)) {
                    let (dt, st) = if ol == 0 { (&arc.cell_rise, &arc.rise_transition) } else { (&arc.cell_fall, &arc.fall_transition) };
                    for il in 0..2 {
                        let feeds = arc.feeds(il == 0, ol == 0); // the reference's edge pairs (latch D->Q, tri-state by kind)
                        if !feeds {
                            continue;
                        }
                        let sin = spec.input_transition[il];
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
                        if std::env::var_os("VYGES_STA_DMP_TRACE").is_some() {
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
        sink_ip: Option<(String, String)>,
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
                        sink_ip: ip_of(s),
                    });
                    out_edges[d].push(Edge {
                        to: s,
                        kind: EdgeKind::Net(aid),
                    });
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
            p.arcs
                .iter()
                .any(|a| !a.sigma_rise.values.is_empty() || !a.sigma_fall.values.is_empty())
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
    let fall_launch = ck_lane_list.iter().any(|&(_, l)| l == Some(1));
    // The fast path keeps ONE clock-side base per endpoint, so one capture edge per pin.
    let mixed_capture =
        flop_d.iter().chain(&flop_hold).any(|(_, cons, _)| cons.windows(2).any(|w| w[0].clk_edge != w[1].clk_edge));
    let simple_ctx = capture && spef.is_none() && !pocv && !aocv && !job.pba && !fall_launch && !mixed_capture;
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
    let seed_main: Vec<[f64; 2]> = seed.iter().map(|&s| [s; 2]).collect();
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
            // 50 % duty: the rising edge at t=0, the falling edge half a period later.
            seed_clk[*idx] = [0.0, period_of_src(name) / 2.0];
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
        let mut stack: Vec<usize> = clock_src.keys().copied().collect();
        for &s in &stack {
            clock_reached[s] = true;
        }
        while let Some(u) = stack.pop() {
            if is_ck[u] {
                continue; // the clock edge stops here; CK->Q is a new launch
            }
            for e in &out_edges[u] {
                if !clock_reached[e.to] {
                    clock_reached[e.to] = true;
                    stack.push(e.to);
                }
            }
        }
    }
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
            if let Some(nd) = node {
                ideal_slew[nd] = Some(*sl);
                stack.push(nd);
            }
        }
        while let Some(u) = stack.pop() {
            if is_ck[u] {
                continue;
            }
            for e in &out_edges[u] {
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
                 fixed_slew: Option<&[[f64; 2]]>|
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
        let records = !block_seq && launch.is_none();
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
        let mut indeg_work = indeg.clone();
        let mut order: Vec<usize> = Vec::new();
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
                order.push(v);
            }
        }
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for e in &out_edges[u] {
                let v = e.to;
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
                            if !a.is_finite() {
                                continue;
                            }
                            let metric = a + d; // band carries (no new variance)
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
                                    if launch.is_some_and(|le| le != il) {
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
                                if !a_in.is_finite() {
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
                                        // `VYGES_STA_DMP_TRACE=1`: the reference's `dmp_ceff` debug
                                        // line per driver edge, in SI, beside the result.
                                        if std::env::var_os("VYGES_STA_DMP_TRACE").is_some() {
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
                                // when present, else the global pocv_sigma fraction.
                                let sigma = if !pocv {
                                    0.0
                                } else {
                                    let lvf = if ol == 0 {
                                        &arc.sigma_rise
                                    } else {
                                        &arc.sigma_fall
                                    };
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
    // Dump it with VYGES_STA_RC_TRACE=1 (and VYGES_STA_RC_NET=<net> for one net).
    const RC_NONE: u8 = 0;
    const RC_TRANSIENT: u8 = 1;
    const RC_TREE_ELMORE: u8 = 2;
    const RC_LUMPED: u8 = 3;
    // Per-net receiver capacitance, keyed by SPEF node — what `ReduceToPi::pinCapacitance`
    // contributes to `downstreamCap` upstream. Built once: it does not depend on arrivals.
    // ⚠️ Liberty caps are pF and SPEF node caps are fF, so convert.
    // Per edge (lane 0 rise, 1 fall): each edge's Elmore walks that edge's receiver caps.
    // `[analysis][edge]` like the driver load.
    let net_pin_cap: Vec<[[BTreeMap<String, f64>; 2]; 2]> = {
        let e = || [BTreeMap::new(), BTreeMap::new()];
        let mut v: Vec<[[BTreeMap<String, f64>; 2]; 2]> = (0..nn).map(|_| [e(), e()]).collect();
        if let Some(sp_all) = spef {
            for a in arcs.iter() {
                let Some(rc) = sp_all.nets.get(&net_order[a.net_idx]) else {
                    continue;
                };
                if let Some((si, spn)) = &a.sink_ip {
                    if let Some(node) = rc.pin_node(si, spn) {
                        let c = pin_cap_node.get(&a.sink).copied().unwrap_or([[0.0; 2]; 2]);
                        for mm in 0..2 {
                            for l in 0..2 {
                                *v[a.net_idx][mm][l].entry(node.to_string()).or_default() += c[mm][l] * ff_per_cap;
                            }
                        }
                    }
                } else if output_ports.iter().any(|(idx, _)| *idx == a.sink) {
                    // A top-level output port: its external load at its own node, as the Pi above
                    // takes it (`ReduceToPi::pinCapacitance` → `Sdc::portExtCap`) — the reduction
                    // that gives the Pi also gives each load's Elmore. Without it the wire delay to
                    // every other load of a net that drives a port missed the port's load.
                    for lanes in v[a.net_idx].iter_mut() {
                        for caps in lanes.iter_mut() {
                            *caps.entry(labels[a.sink].clone()).or_default() += job.output_load * ff_per_cap;
                        }
                    }
                }
            }
        }
        v
    };
    let rc_branch = std::cell::RefCell::new(vec![RC_NONE; n_arcs]);
    // `rc_model: transient` in the job asks for our waveform model in front of the
    // reference one. Default is the reference.
    let transient_first = job.rc_model.eq_ignore_ascii_case("transient");
    // ⛔ PER ARC PER LANE. The reference computes a wire delay and a sink slew inside the
    // RISE or FALL arc evaluation — `loadDelaySlew` is called from `gateDelay` for one
    // `RiseFall` — so both are per edge. Collapsing them to one value per arc forces a
    // single driver waveform onto both edges, which was measured to take data-hold rms
    // from 0.0329 to 0.0537 when the per-sink crossing solve was wired against it.
    // `mm`: the analysis (0 early, 1 late), whose receiver caps the Elmore walks.
    let compute_wires = |sw: &[[f64; 2]], net_slew: &[[f64; 2]], mm: usize| -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
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
        let win = |x: [f64; 2]| if x[0] > x[1] { x[0] } else { x[1] };
        let slw_of = |x: [f64; 2]| if x[0] > x[1] { x[0] } else { x[1] };
        let xc: Vec<f64> = (0..nn)
            .map(|i| {
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
            })
            .collect();
        // waveform-into-RC: a transient response per net (driver-slew-aware), memoized
        // so each sink reads the same simulation. Only built when it is actually selected.
        let tr_net: Vec<Option<BTreeMap<String, (f64, f64)>>> = (0..nn)
            .map(|i| {
                if !transient_first {
                    return None;
                }
                let rc = spef?.nets.get(&net_order[i])?;
                let (di, dp) = net_drv_ip[i].as_ref()?;
                let dn = rc.pin_node(di, dp)?;
                // the solve is in ns: the driver slew in, each delay and slew out
                let map = rc.transient(dn, slw_of(net_slew[i]) * ns_per_time, xc[i], lib.thresholds)?;
                Some(map.into_iter().map(|(n, (d, s))| (n, (d * time_per_ns, s * time_per_ns))).collect())
            })
            .collect();
        // each arc -> (net delay, degraded sink slew); slew 0 means "keep driver slew"
        arcs.iter()
            .enumerate()
            .map(|(k, a)| {
                let i = a.net_idx;
                let mark = |b: u8| rc_branch.borrow_mut()[k] = b;
                let Some(rc) = spef.and_then(|s| s.nets.get(&net_order[i])) else {
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
                    let dnode = match &net_drv_ip[i] {
                        Some((di, dp)) => rc.pin_node(di, dp),
                        None => net_drv[i].map(|d| labels[d].as_str()),
                    };
                    let snode = match &a.sink_ip {
                        Some((si, sp)) => rc.pin_node(si, sp),
                        None => Some(labels[a.sink].as_str()),
                    };
                    if let (Some(dt), Some(st)) = (dnode, snode) {
                        let taus = [0, 1].map(|l| rc.elmore(dt, xc[i], &net_pin_cap[i][mm][l]).and_then(|dl| dl.get(st).copied()));
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
                                    let drvr_slew = net_slew[i][l];
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
                    if let (Some(map), Some((si, sp))) = (&tr_net[i], &a.sink_ip) {
                        if let Some(sn) = rc.pin_node(si, sp) {
                            if let Some(&(delay, slew)) = map.get(sn) {
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
                let d = net_res[i] * net_cap[i] * fs_to_time + net_res[i] * xc[i] * fs_to_time;
                ([d, d], [0.0, 0.0])
            })
            .unzip()
    };

    // every wire edge from a port with a driving cell also carries the cell's load-dependent
    // delay, `gate - intrinsic` (`annotateLoadDelays`)
    let compute = |sw: &[[f64; 2]], net_slew: &[[f64; 2]], mm: usize| -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
        let (mut d, s) = compute_wires(sw, net_slew, mm);
        if !port_drive.is_empty() {
            for (dk, a) in d.iter_mut().zip(arcs.iter()) {
                if let Some(pd) = net_drv[a.net_idx].and_then(|dn| port_drive.get(&dn)) {
                    for (x, e) in dk.iter_mut().zip(pd.extra[mm]) {
                        *x += e;
                    }
                }
            }
        }
        (d, s)
    };

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
        let (_a, _s, _f, ord, arr, slw, _fl) = relax(&arc_d, &arc_s, true, &seed_main, false, None, None);
        if !cycle_checked {
            if ord.len() != n {
                return Err(StaError::CombinationalLoop);
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
    if std::env::var_os("VYGES_STA_RC_TRACE").is_some() {
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
                std::env::var("VYGES_STA_RC_NET").ok().as_deref() == Some(&net_order[*i])
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
            let (_a, _s2, _f, _o, _ar, slw, _fl) = relax(&d, &sl, false, &seed_main, false, None, None);
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
    // final late propagation consistent with the converged per-arc delays, and the
    // early (min-delay) propagation used for hold and for early clock arrivals.
    let (arrival, slew, from, order_late, late_arr, late_slew, late_from) =
        relax(&arc_d, &arc_s, true, &seed_main, false, None, None);
    let (arr_min, slew_min, from_min, _ord_min, early_arr, early_slew, early_from) =
        relax(&early_d, &early_s, false, &seed_main, false, None, None);

    // ---- per launching edge -----------------------------------------------
    // The reference keeps one arrival per clock EDGE that launched it (a tag per `ClockEdge`) and
    // checks each against the capture edge its cycle accounting pairs with that launch. The passes
    // above merge every launch into one arrival measured from its own edge, which is exact while
    // every register launches on the same edge. Where one launches on the fall, each edge is timed
    // alone — its own launches only (inputs launch on the rise), at the slews merged over all of
    // them — and every endpoint is checked once per launching edge.
    let edge_runs: Vec<EdgeRun> = if fall_launch {
        let fall_seed: Vec<[f64; 2]> =
            (0..n).map(|i| if clock_src.contains_key(&i) { seed_main[i] } else { [f64::NEG_INFINITY; 2] }).collect();
        [(0, &seed_main), (1, &fall_seed)]
            .into_iter()
            .map(|(edge, sd)| {
                let (_, _, from, _, late_arr, _, late_from) = relax(&arc_d, &arc_s, true, sd, false, Some(edge), Some(&late_slew));
                let (_, _, from_min, _, early_arr, _, early_from) =
                    relax(&early_d, &early_s, false, sd, false, Some(edge), Some(&early_slew));
                EdgeRun { edge, late_arr, late_from, from, early_arr, early_from, from_min }
            })
            .collect()
    } else {
        Vec::new()
    };
    // every launching edge as a view: the one merged pass when all launch on the rise
    let runs: Vec<EdgeView> = if edge_runs.is_empty() {
        vec![EdgeView {
            edge: 0,
            late_arr: &late_arr,
            late_from: &late_from,
            from: &from,
            early_arr: &early_arr,
            early_from: &early_from,
            from_min: &from_min,
        }]
    } else {
        edge_runs.iter().map(EdgeRun::view).collect()
    };

    // Clock-launched arrivals: the same late propagation, but started at the clock
    // source's own edge times and stopped at every flop CK pin. A clock port driving
    // an output port (a clock forwarded off-chip) is a real timed path, and it is one
    // the main pass cannot see: there the falling edge leaves at half a period, so its
    // arrival is that much later than a delay measured from the launch edge. On a block
    // with a forwarded clock this is routinely the critical path.
    let (clk_arr, clk_slew, clk_from, _clk_ord, _, _, _) = if have_clock_launch {
        relax(&arc_d, &arc_s, true, &seed_clk, true, None, None)
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
    // The clock every I/O delay is relative to here (the job keeps no per-delay clock), and the
    // path group an endpoint is reported in: `asynchronous` for recovery and removal, else the
    // capture clock's name (`PathGroups::pathGroup`).
    let primary_clock = job.clocks.first().map_or(job.clock_port.as_str(), |c| c.0.as_str());
    let path_group = |v: usize, cap: Option<usize>| -> String {
        if async_end.contains(&v) {
            "asynchronous".to_string()
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
    let edge_relation = |pl: f64, ll: usize, pc: f64, cl: usize| -> (f64, f64) {
        let (setup, hold) = cycle_accting(pl, if ll == 1 { pl / 2.0 } else { 0.0 }, pc, if cl == 1 { pc / 2.0 } else { 0.0 });
        (setup, -hold)
    };

    // worst (max) constraint over a pin's groups, interpolated at the operating
    // clock + data transitions — matches how delay arcs are looked up.

    // timing exceptions, matched on launch/capture instance (or port) names.
    let inst_of = |node: usize| instance_of(&labels[node]).to_string();
    // Membership, not equality: an exception may name a whole bus on either side, and the
    // rule for that lives once in `Exception::covers` rather than being re-derived here.
    // the HOLD check's exception: a `-setup` false path leaves it timed (Exception::applies_to)
    let match_exc = |ln: &str, cn: &str| job.exceptions.iter().filter(|e| e.applies_to(true)).find(|e| e.covers(ln, cn));
    let mut excluded_setup = vec![false; n]; // false-path endpoints (skip setup)
    // the reference's rule, where the job asks for it: an output with no output delay ends no path
    if job.unconstrained_outputs {
        for (idx, name) in &output_ports {
            if !job.output_delay_declared_for(name) {
                excluded_setup[*idx] = true;
            }
        }
    }
    // Required time PER LAUNCHING EDGE AND DATA EDGE: `req_run[r][v][l]` against `runs[r]`'s
    // arrivals; NaN = no check from that edge. `endpoint_req` keeps the tightest so the query API
    // still answers with one number; the scoring loop below uses them all.
    let mut req_run = vec![vec![[f64::NAN; 2]; n]; runs.len()];
    // An output port's requirement is the same for either data edge; scored per edge like a
    // register's, so its reported path is the edge that set the slack, followed lane by lane
    // (collapsed, each node's latest edge was printed — `_414_/Q` rise beside a falling path).
    // It is captured on the rise; a launch on the fall has that much less of the period.
    for (idx, _) in &output_ports {
        for (r, run) in runs.iter().enumerate() {
            let (rel, _) = edge_relation(period, run.edge, period, 0);
            req_run[r][*idx] = [endpoint_req[*idx] - period + rel; 2];
        }
    }

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
        let pc = cap.map(&clock_period_of).unwrap_or(period);
        let mut checked = false;
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(*idx, run.from);
            let pl = lck.map(&clock_period_of).unwrap_or(pc);
            let ln = lck.map(&inst_of).unwrap_or_default();
            // A `-hold` multicycle or false path does not reach the SETUP check, and `-start`
            // counts the launch clock's cycles where the default `-end` counts the capture clock's.
            let setup_exc = |e: &&Exception| e.applies_to(false);
            let mut shift = 0.0;
            if let Some(e) = job.exceptions.iter().filter(setup_exc).find(|e| e.covers(&ln, &inst_of(*idx))) {
                match e.kind {
                    ExcKind::FalsePath => continue,
                    ExcKind::Multicycle(cyc) => shift = cyc.saturating_sub(1) as f64 * if e.start { pl } else { pc },
                }
            }
            if is_async(lck, cap) {
                continue; // async clock groups: no setup check
            }
            checked = true;
            // Each check at ITS capture edge (`setup_rising` / `_falling`): that edge's clock
            // arrival and slew, and the relation from this run's launch edge to it.
            let base_of = |c: &Constraint| -> (f64, f64) {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                let ck_slew = cap.map(|i| late_slew[i][cl]).unwrap_or(input_slew);
                let crpr = match (lck, cap) {
                    (Some(l), Some(c)) => crpr_credit(l, c, cl, true),
                    _ => 0.0,
                };
                (cap_early + edge_relation(pl, run.edge, pc, cl).0 + shift + crpr - job.setup_uncertainty, ck_slew)
            };
            // ⛔ PER DATA EDGE, for the same reason as hold: a check exists for one edge, so
            // the latest arrival must meet THAT edge's setup constraint, not the worse of the
            // two.
            for (l, req) in req_run[r][*idx].iter_mut().enumerate() {
                *req = setup
                    .iter()
                    .filter(|c| checks_edge(c, l == 0))
                    .map(|c| {
                        let (base, ck_slew) = base_of(c);
                        base - c.eval_edge(l == 0, ck_slew, late_slew[*idx][l])
                    })
                    .fold(f64::INFINITY, f64::min);
            }
            // capture the clock-side constants so the fast path can re-derive required time
            // from the (possibly changed) data slew without re-walking the clock network.
            if simple_ctx {
                if let Some((base, ck_slew)) = setup.first().map(&base_of) {
                    inc_setup.push(SetupRec {
                        idx: *idx,
                        base,
                        cons: setup.clone(),
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
    // each path group's worst setup end, for the report: (slack, endpoint, run, data edge)
    let setup_end: HashMap<usize, usize> = flop_d.iter().enumerate().map(|(e, (idx, _, _))| (*idx, e)).collect();
    let mut setup_group_worst: BTreeMap<String, (f64, usize, usize, usize)> = BTreeMap::new();
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
        if !clk_launched {
            for (r, run) in runs.iter().enumerate() {
                for (l, (&q, &a)) in req_run[r][v].iter().zip(&run.late_arr[v]).enumerate() {
                    if !q.is_nan() && a.is_finite() && q - a < slack {
                        slack = q - a;
                        slack_at = Some((r, l));
                    }
                }
            }
        }
        if !slack.is_finite() {
            slack = endpoint_req[v] - arr_v;
        }
        // Several launching edges: no one required time pairs with the merged arrival, so the
        // slack is folded back into it, as the incremental path folds its per-edge slack — the
        // query API's `required − arrival` is then this endpoint's checked slack.
        if runs.len() > 1 {
            endpoint_req[v] = arr_v + slack;
        }
        if let Some((r, l)) = slack_at {
            let g = path_group(v, setup_end.get(&v).and_then(|&e| ck_node(&flop_d[e].2)));
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
            for e in edges {
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
        let pc = cap.map(&clock_period_of).unwrap_or(period);
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
        let mut checked = false;
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(idx, run.from_min);
            let pl = lck.map(&clock_period_of).unwrap_or(pc);
            // apply exceptions: false path drops the check, multicycle shifts the hold capture
            // back by (cycles − 1).
            let ln = lck.map(&inst_of).unwrap_or_default();
            let mut shift = 0.0;
            if let Some(e) = match_exc(&ln, &inst_of(idx)) {
                match e.kind {
                    ExcKind::FalsePath => continue,
                    ExcKind::Multicycle(cyc) => shift = cyc.saturating_sub(1) as f64 * pc,
                }
            }
            if is_async(lck, cap) {
                continue; // async clock groups: no hold check
            }
            // Each check at ITS capture edge: that edge's late clock arrival and early slew, and
            // the hold relation from this run's launch edge to it.
            let base_of = |c: &Constraint| -> (f64, f64) {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let crpr = match (lck, cap) {
                    (Some(l), Some(c)) => crpr_credit(l, c, cl, false),
                    _ => 0.0,
                };
                let cap_late = cap.map(|i| late_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                let ck_slew = cap.map(|i| early_slew[i][cl]).unwrap_or(input_slew);
                let hold_rel = edge_relation(pl, run.edge, pc, cl).1 + shift;
                (crpr - cap_late - hold_rel - job.hold_uncertainty, ck_slew)
            };
            let mut reached = false;
            for (l, &a) in run.early_arr[idx].iter().enumerate() {
                if !a.is_finite() {
                    continue;
                }
                reached = true;
                for c in hold.iter().filter(|c| checks_edge(c, l == 0)) {
                    let (base, ck_slew) = base_of(c);
                    let sl = a + base - c.eval_edge(l == 0, ck_slew, early_slew[idx][l]);
                    if sl < slack {
                        slack = sl;
                        slack_at = (r, l);
                    }
                }
            }
            checked |= reached;
            if reached && simple_ctx {
                if let Some((base, ck_slew)) = hold.first().map(&base_of) {
                    inc_hold.push(HoldRec {
                        idx,
                        base,
                        cons: hold.clone(),
                        ck_slew,
                        launch_ck: lck,
                        port: false,
                    });
                }
            }
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
        if hold_group_worst.get(&g).is_none_or(|w| slack < w.0) {
            hold_group_worst.insert(g, (slack, idx, slack_at.0, slack_at.1));
        }
    }
    // An output port's HOLD check. `set_output_delay` constrains both analyses (the reference's
    // `PathEndOutputDelay`): the data must not arrive before the capture edge less the
    // external delay — required = hold edge − output delay (+ hold uncertainty), per edge.
    // Without it no output was ever hold-checked: the reference's min report on a design
    // whose only endpoint is an output port says `0.2196 slack`, this timer said nothing.
    for (idx, name) in &output_ports {
        let idx = *idx;
        if excluded_setup[idx] || !arr_min[idx].is_finite() || !job.output_delay_declared_for(name) {
            continue; // false-pathed, unreached, or unconstrained
        }
        let mut slack = f64::INFINITY;
        let mut slack_at = (0, 0);
        for (r, run) in runs.iter().enumerate() {
            let lck = launch_ck(idx, run.from_min);
            let pl = lck.map(&clock_period_of).unwrap_or(period);
            let (_, mut hold_rel) = edge_relation(pl, run.edge, period, 0);
            let ln = lck.map(&inst_of).unwrap_or_default();
            if let Some(e) = match_exc(&ln, name) {
                match e.kind {
                    ExcKind::FalsePath => continue,
                    ExcKind::Multicycle(cyc) => hold_rel += cyc.saturating_sub(1) as f64 * period,
                }
            }
            let base = job.output_delay_for(name) - hold_rel - job.hold_uncertainty;
            let mut reached = false;
            for (l, &a) in run.early_arr[idx].iter().enumerate() {
                if a.is_finite() {
                    reached = true;
                    if a + base < slack {
                        slack = a + base;
                        slack_at = (r, l);
                    }
                }
            }
            if reached && simple_ctx {
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
        let g = path_group(idx, hold_end.get(&idx).and_then(|&e| ck_node(&flop_hold[e].2)));
        if hold_group_worst.get(&g).is_none_or(|w| slack < w.0) {
            hold_group_worst.insert(g, (slack, idx, slack_at.0, slack_at.1));
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
            if e == 1 { "falling edge-triggered flip-flop" } else { "rising edge-triggered flip-flop" }
        };
        let make = |max: bool, v: usize, r: usize, l: usize, group: &str| -> Option<CheckPath> {
            let run = &runs[r];
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
            let lck = launch_ck(v, if max { run.from } else { run.from_min });
            let src_name = lck.and_then(clock_name_of).unwrap_or(primary_clock);
            // the launch clock's period; with no launching clock pin (an input), the period the
            // check defaults to: the capture clock's for a register, the job's for an output
            let pl_or = |default: f64| lck.map(&clock_period_of).unwrap_or(default);
            let pl = pl_or(period);
            let launch_edge = if k.is_some() { s0l } else { 0 };
            let launch = if launch_edge == 1 { pl / 2.0 } else { 0.0 };
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
            let ln = lck.map(&inst_of).unwrap_or_default();
            if out_port.contains(&v) {
                // an output port, captured on the rise of the clock its delay is relative to
                let (setup_rel, hold_rel) = edge_relation(pl, run.edge, period, 0);
                // as the output hold check shifts it
                let mc = match match_exc(&ln, &labels[v]).map(|e| &e.kind) {
                    Some(ExcKind::Multicycle(cyc)) => cyc.saturating_sub(1) as f64 * period,
                    _ => 0.0,
                };
                let (tgt_time, required, uncertainty) = if max {
                    (launch + setup_rel, launch + req_run[r][v][l], -job.setup_uncertainty)
                } else {
                    let rel = hold_rel + mc;
                    (launch + rel, launch + rel + job.hold_uncertainty - job.output_delay_for(&labels[v]), job.hold_uncertainty)
                };
                return Some(CheckPath {
                    max,
                    group: group.to_string(),
                    startpoint,
                    start_reason,
                    endpoint: labels[v].clone(),
                    end_reason: format!("output port clocked by {primary_clock}"),
                    src_clock,
                    src_network_delay,
                    input_delay,
                    lines,
                    arrival,
                    tgt_clock: clock_line(primary_clock, true, tgt_time),
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
            let pc = cap.map(&clock_period_of).unwrap_or(period);
            let pl = pl_or(pc);
            let shift = if max {
                let setup_exc = |e: &&Exception| e.applies_to(false);
                match job.exceptions.iter().filter(setup_exc).find(|e| e.covers(&ln, &inst_of(v))) {
                    Some(e) => match &e.kind {
                        ExcKind::Multicycle(cyc) => cyc.saturating_sub(1) as f64 * if e.start { pl } else { pc },
                        ExcKind::FalsePath => 0.0,
                    },
                    None => 0.0,
                }
            } else {
                match match_exc(&ln, &inst_of(v)).map(|e| &e.kind) {
                    Some(ExcKind::Multicycle(cyc)) => cyc.saturating_sub(1) as f64 * pc,
                    _ => 0.0,
                }
            };
            // (required, capture edge, capture clock arrival, relation) of each constraint;
            // the tightest one set the slack
            let terms = cons.iter().filter(|c| checks_edge(c, l == 0)).map(|c| {
                let cl = usize::from(c.clk_edge == ClkEdge::Fall);
                let crpr = match (lck, cap) {
                    (Some(a), Some(c)) => crpr_credit(a, c, cl, max),
                    _ => 0.0,
                };
                if max {
                    let cap_early = cap.map(|i| early_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                    let ck_slew = cap.map(|i| late_slew[i][cl]).unwrap_or(input_slew);
                    let rel = edge_relation(pl, run.edge, pc, cl).0 + shift;
                    let req = cap_early + rel + crpr - job.setup_uncertainty - c.eval_edge(l == 0, ck_slew, late_slew[v][l]);
                    (req, cl, cap_early, rel, crpr)
                } else {
                    let cap_late = cap.map(|i| late_arr[i][cl]).filter(|a| a.is_finite()).unwrap_or(0.0);
                    let ck_slew = cap.map(|i| early_slew[i][cl]).unwrap_or(input_slew);
                    let rel = edge_relation(pl, run.edge, pc, cl).1 + shift;
                    let req = cap_late + rel + job.hold_uncertainty - crpr + c.eval_edge(l == 0, ck_slew, early_slew[v][l]);
                    (req, cl, cap_late, rel, crpr)
                }
            });
            let (req, cl, cap_arr, rel, crpr) = if max {
                terms.min_by(|a, b| a.0.total_cmp(&b.0))?
            } else {
                terms.max_by(|a, b| a.0.total_cmp(&b.0))?
            };
            let tgt_name = cap.and_then(clock_name_of).unwrap_or(primary_clock);
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
                tgt_clock: clock_line(tgt_name, cl == 0, launch + rel),
                tgt_network_delay: cap_arr,
                uncertainty: if max { -job.setup_uncertainty } else { job.hold_uncertainty },
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
                if let Some(p) = make(max, v, r, l, &g) {
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
    };
    // Retain a per-pin snapshot for the Timer's query API (Phase 1). Cloned, not moved:
    // closures above still hold immutable borrows of these arrays through the return.
    // the query API reports one load per driver: the larger edge's
    let node_load_max: Vec<f64> = node_load.iter().map(|l| l[1][0].max(l[1][1])).collect();
    let timing = Timing::new(
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
                    EdgeKind::Cell(arc) => InEdge::Cell {
                        from: u,
                        arc: arc.clone(),
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
