//! STA engine: build the timing graph, propagate, report slack.
//!
//! From a netlist + Liberty + clock it builds a directed timing graph — cell
//! arcs (input pin → output pin, delay from the NLDM tables given input slew and
//! output load) and net arcs (driver → sinks) — topologically orders it,
//! propagates arrival time + slew forward, derives required time backward from
//! the clock period, and reports WNS / TNS and the worst path.
//!
//! When a SPEF is supplied, net arcs carry a per-pin tree Elmore interconnect
//! delay and the wire capacitance loads the driver; crosstalk (see `si`) adds a
//! window-filtered margin. Analysis covers **max-delay setup** (combinational
//! input → output, and register-to-register: a flop's Q launches via its CK→Q
//! arc, its D pins are capture endpoints with required = period − setup) **and
//! min-delay hold** (a second, min-corner forward pass; earliest data arrival at
//! each flop D must clear its hold constraint). On-chip variation is flat scalar
//! derates by default, or **AOCV** (depth-dependent derate table) / **POCV**
//! (per-stage sigma, N-sigma band growing as sqrt(depth)) when configured. Pure
//! std — unit-tested.

use std::collections::HashMap;

use crate::job::StaJob;
use crate::liberty::{Arc, Dir, Lib};
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

/// Run combinational max-delay STA and return the slack report.
pub fn analyze(
    nl: &Netlist,
    lib: &Lib,
    job: &StaJob,
    spef: Option<&Spef>,
) -> Result<TimingReport, StaError> {
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
    for p in &nl.inputs {
        let idx = node(port_key(p), p.clone(), &mut key2idx, &mut labels, &mut is_endpoint);
        ensure_net(&mut nets, p);
        nets.get_mut(p).unwrap().driver = Some(idx);
    }
    // primary output ports are endpoints + sinks of their net
    for p in &nl.outputs {
        let idx = node(port_key(p), p.clone(), &mut key2idx, &mut labels, &mut is_endpoint);
        is_endpoint[idx] = true;
        ensure_net(&mut nets, p);
        let net = nets.get_mut(p).unwrap();
        net.sinks.push(idx);
        net.load += job.output_load;
    }

    // instance pins. A sequential cell's data pins (with a setup constraint)
    // are *capture* endpoints; its Q launches via the CK->Q delay arc.
    let mut flop_d: Vec<(usize, f64)> = Vec::new(); // (D pin node, setup ns)
    let mut flop_hold: Vec<(usize, f64)> = Vec::new(); // (D pin node, hold ns)
    for inst in &nl.insts {
        let cell = lib.cell(&inst.cell).ok_or_else(|| StaError::UnknownCell(inst.cell.clone()))?;
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
                    let cap = cell.pins[pin].capacitance;
                    let nref = nets.get_mut(net).unwrap();
                    nref.sinks.push(idx);
                    nref.load += cap;
                    if cell.is_seq {
                        if let Some(setup) = cell.pins[pin].setup {
                            is_endpoint[idx] = true; // data pin = setup capture endpoint
                            flop_d.push((idx, setup));
                        }
                        if let Some(hold) = cell.pins[pin].hold {
                            flop_hold.push((idx, hold)); // same pin = hold capture endpoint
                        }
                    }
                }
                _ => {}
            }
        }
    }

    let n = labels.len();
    // required time at each endpoint: clock period, less setup at flop D pins
    let period = job.period_ns;
    let mut endpoint_req = vec![period; n];
    for &(idx, setup) in &flop_d {
        endpoint_req[idx] = period - setup;
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
        let cell = lib.cell(&inst.cell).unwrap();
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
    let pocv = job.pocv_sigma > 0.0;
    let aocv = !pocv && (!job.aocv_late.is_empty() || !job.aocv_early.is_empty());
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
    // `late=true` is the max-delay (setup) path: max-corner cell delay. `late=false`
    // is the min-delay (hold) path: min-corner cell delay, accumulated with a min.
    // Net arcs carry the SPEF delay and neither derate nor add depth/variance.
    let edge_raw = |kind: &EdgeKind, slew_u: f64, load_v: f64, nd: &[f64], late: bool| -> (f64, f64, bool) {
        match kind {
            EdgeKind::Net(i) => (nd[*i], slew_u, false),
            EdgeKind::Cell(a) => {
                let r = a.cell_rise.lookup(slew_u, load_v);
                let f = a.cell_fall.lookup(slew_u, load_v);
                let sr = a.rise_transition.lookup(slew_u, load_v);
                let sf = a.fall_transition.lookup(slew_u, load_v);
                if late {
                    (r.max(f), sr.max(sf), true)
                } else {
                    (r.min(f), sr.min(sf), true)
                }
            }
        }
    };
    let input_slew = job.input_slew;
    // Returns the reported (derated / N-sigma) arrival metric, slew, predecessor,
    // and topo order. Internally tracks nominal arrival, accumulated depth, and
    // variance so AOCV can derate per stage and POCV can band by sqrt(depth).
    let relax = |nd: &[f64], late: bool| -> (Vec<f64>, Vec<f64>, Vec<Option<usize>>, Vec<usize>) {
        let init = if late { f64::NEG_INFINITY } else { f64::INFINITY };
        let mut arrival = vec![init; n]; // the metric: derated (flat/AOCV) or +-N*sigma (POCV)
        let mut arr_nom = vec![0.0f64; n]; // nominal accumulated delay (for the next stage)
        let mut var = vec![0.0f64; n]; // accumulated delay variance (POCV)
        let mut depth = vec![0usize; n]; // cell-stage count on the selected path
        let mut slew = vec![input_slew; n];
        let mut from: Vec<Option<usize>> = vec![None; n];
        let mut indeg_work = indeg.clone();
        let mut order: Vec<usize> = Vec::new();
        for v in 0..n {
            if indeg_work[v] == 0 {
                arrival[v] = 0.0;
                order.push(v);
            }
        }
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for e in &out_edges[u] {
                let (d_nom, sout, is_cell) = edge_raw(&e.kind, slew[u], node_load[e.to], nd, late);
                let stage = if is_cell { depth[u] + 1 } else { depth[u] };
                let derate = if is_cell { cell_derate(late, stage) } else { 1.0 };
                let arr_nom_cand = arr_nom[u] + d_nom * derate;
                let sigma = if pocv && is_cell { pocv_sigma * d_nom } else { 0.0 };
                let var_cand = var[u] + sigma * sigma;
                let metric_cand = if pocv {
                    let band = n_sigma * var_cand.sqrt();
                    if late { arr_nom_cand + band } else { arr_nom_cand - band }
                } else {
                    arr_nom_cand
                };
                let better =
                    if late { metric_cand > arrival[e.to] } else { metric_cand < arrival[e.to] };
                if better {
                    arrival[e.to] = metric_cand;
                    arr_nom[e.to] = arr_nom_cand;
                    var[e.to] = var_cand;
                    depth[e.to] = stage;
                    slew[e.to] = sout;
                    from[e.to] = Some(u);
                }
                indeg_work[e.to] -= 1;
                if indeg_work[e.to] == 0 {
                    order.push(e.to);
                }
            }
        }
        (arrival, slew, from, order)
    };

    // Per-arc interconnect delay, iterated to convergence. Each net's switching
    // window is slew-derived; an aggressor's Miller cap is added (window-overlap)
    // at the victim's net node, and a **per-pin tree Elmore** turns the RC network
    // into a distinct delay for each driver→sink arc (lumped R·C fallback when the
    // SPEF has no usable tree). Arrivals set the windows and the windows feed back
    // into arrivals, so we iterate until the per-arc delays stabilise.
    let guard = job.xtalk_window;
    let miller = job.miller;
    let compute = |sw: &[f64], net_slew: &[f64]| -> Vec<f64> {
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
        arcs.iter()
            .map(|a| {
                let i = a.net_idx;
                let Some(rc) = spef.and_then(|s| s.nets.get(&net_order[i])) else {
                    return 0.0; // no parasitics -> ideal interconnect
                };
                // per-pin tree Elmore when the driver + sink map to SPEF nodes
                if let (Some((di, dp)), Some((si, sp))) = (&net_drv_ip[i], &a.sink_ip) {
                    if let (Some(dt), Some(st)) = (rc.pin_node(di, dp), rc.pin_node(si, sp)) {
                        if let Some(dl) = rc.elmore(dt, xc[i]) {
                            if let Some(&v) = dl.get(st) {
                                return v;
                            }
                        }
                    }
                }
                // fallback: lumped Elmore (R·C) + lumped crosstalk (R·xtalk-cap)
                net_res[i] * net_cap[i] * 1e-6 + net_res[i] * xc[i] * 1e-6
            })
            .collect()
    };

    const MAX_SI_ITERS: usize = 20;
    const SI_TOL: f64 = 1e-9; // ns — per-arc delay change below which we stop
    let neg = vec![f64::NEG_INFINITY; nn];
    let zero = vec![0.0f64; nn];
    let arc_delay_nom = compute(&neg, &zero); // nominal: no windows -> no crosstalk
    let mut arc_delay = arc_delay_nom.clone();
    let mut cycle_checked = false;
    for _ in 0..MAX_SI_ITERS {
        let (arr, slw, _f, ord) = relax(&arc_delay, true);
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
        let next = compute(&sw, &net_slew);
        let delta = (0..n_arcs).map(|k| (next[k] - arc_delay[k]).abs()).fold(0.0, f64::max);
        arc_delay = next;
        if delta < SI_TOL {
            break;
        }
    }
    // final late propagation consistent with the converged per-arc delays
    let (arrival, slew, from, _order) = relax(&arc_delay, true);

    // ---- setup slack + worst path ---------------------------------------
    // Each endpoint's required time is fixed (period at outputs, period - setup at
    // flop D), so slack = required - latest arrival; no backward pass needed.
    let mut wns = f64::INFINITY;
    let mut tns = 0.0;
    let mut worst = None;
    let mut endpoints = 0;
    for v in 0..n {
        if !is_endpoint[v] || arrival[v] == f64::NEG_INFINITY {
            continue;
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

    // ---- hold (early / min-delay) path ----------------------------------
    // Earliest data arrival via min-corner cell delays + nominal (no-crosstalk)
    // interconnect. A flop D pin's hold check: the data must stay stable for
    // `hold` ns after the (same, zero-skew) capture edge, so the earliest arrival
    // must be >= hold. Slack = earliest_arrival - hold.
    let (arr_min, slew_min, from_min, _ord_min) = relax(&arc_delay_nom, false);
    let mut whs = f64::INFINITY;
    let mut ths = 0.0;
    let mut worst_hold = None;
    let mut hold_endpoints = 0;
    for &(idx, hold) in &flop_hold {
        if arr_min[idx] == f64::INFINITY {
            continue; // unreached
        }
        hold_endpoints += 1;
        let slack = arr_min[idx] - hold;
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

    Ok(TimingReport {
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
    })
}
