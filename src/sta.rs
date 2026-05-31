//! STA engine: build the timing graph, propagate, report slack.
//!
//! From a netlist + Liberty + clock it builds a directed timing graph — cell
//! arcs (input pin → output pin, delay from the NLDM tables given input slew and
//! output load) and net arcs (driver → sinks) — topologically orders it,
//! propagates arrival time + slew forward, derives required time backward from
//! the clock period, and reports WNS / TNS and the worst path.
//!
//! When a SPEF is supplied, net arcs carry the lumped Elmore interconnect delay
//! (R·C) and the wire capacitance is added to the driver load; without one the
//! interconnect is ideal. v0 is **combinational max-delay** analysis (primary
//! input → primary output) with a late OCV derate. Register CK→Q arcs are
//! followed when present; flop setup/hold and crosstalk (see `si`) are the
//! upgrades. Pure std — fully unit-tested offline.

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

#[derive(Debug, Clone)]
pub struct PathNode {
    pub label: String,
    pub arrival: f64,
    pub slew: f64,
}

#[derive(Debug, Clone)]
pub struct TimingReport {
    pub wns: f64,           // worst negative slack (ns); >0 means met
    pub tns: f64,           // total negative slack over endpoints (ns)
    pub endpoints: usize,
    pub worst_endpoint: String,
    pub worst_path: Vec<PathNode>,
}

enum EdgeKind {
    Net(f64), // interconnect (Elmore) delay in ns, from SPEF (0 if ideal)
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

    // instance pins
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
                }
                _ => {}
            }
        }
    }

    let n = labels.len();
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
    // net arcs: driver -> each sink, carrying the SPEF Elmore net delay plus the
    // worst-case crosstalk delta-delay from coupling (SI).
    for (netname, net) in &nets {
        if let Some(d) = net.driver {
            let ndelay = spef
                .and_then(|s| s.nets.get(netname))
                .map(|rc| {
                    rc.res_ohm * rc.cap_ff * 1e-6 // nominal Elmore (Cc counted once, grounded)
                        + crate::si::xtalk_delta_ns(rc.res_ohm, rc.coupling_ff, job.miller)
                })
                .unwrap_or(0.0);
            for &s in &net.sinks {
                if s != d {
                    out_edges[d].push(Edge { to: s, kind: EdgeKind::Net(ndelay) });
                    indeg[s] += 1;
                }
            }
        }
    }

    // ---- forward: topological propagation of arrival + slew --------------
    let derate = job.late_derate;
    let edge_delay = |kind: &EdgeKind, slew_u: f64, load_v: f64| -> (f64, f64) {
        match kind {
            EdgeKind::Net(d) => (*d, slew_u),
            EdgeKind::Cell(a) => {
                let d = a.cell_rise.lookup(slew_u, load_v).max(a.cell_fall.lookup(slew_u, load_v));
                let s = a
                    .rise_transition
                    .lookup(slew_u, load_v)
                    .max(a.fall_transition.lookup(slew_u, load_v));
                (d * derate, s)
            }
        }
    };

    let mut arrival = vec![f64::NEG_INFINITY; n];
    let mut slew = vec![job.input_slew; n];
    let mut from: Vec<Option<usize>> = vec![None; n];
    let mut indeg_work = indeg.clone();
    let mut queue: Vec<usize> = Vec::new();
    for v in 0..n {
        if indeg_work[v] == 0 {
            arrival[v] = 0.0; // start point (primary input or undriven sink)
            queue.push(v);
        }
    }
    let mut processed = 0usize;
    let mut head = 0;
    while head < queue.len() {
        let u = queue[head];
        head += 1;
        processed += 1;
        for e in &out_edges[u] {
            let (d, sout) = edge_delay(&e.kind, slew[u], node_load[e.to]);
            let cand = arrival[u] + d;
            if cand > arrival[e.to] {
                arrival[e.to] = cand;
                slew[e.to] = sout;
                from[e.to] = Some(u);
            }
            indeg_work[e.to] -= 1;
            if indeg_work[e.to] == 0 {
                queue.push(e.to);
            }
        }
    }
    if processed != n {
        return Err(StaError::CombinationalLoop);
    }

    // ---- backward: required time from the clock period -------------------
    let period = job.period_ns;
    let mut required = vec![f64::INFINITY; n];
    for v in 0..n {
        if is_endpoint[v] {
            required[v] = period;
        }
    }
    for &u in queue.iter().rev() {
        for e in &out_edges[u] {
            let (d, _) = edge_delay(&e.kind, slew[u], node_load[e.to]);
            let req = required[e.to] - d;
            if req < required[u] {
                required[u] = req;
            }
        }
    }

    // ---- slack + worst path ---------------------------------------------
    let mut wns = f64::INFINITY;
    let mut tns = 0.0;
    let mut worst = None;
    let mut endpoints = 0;
    for v in 0..n {
        if !is_endpoint[v] || arrival[v] == f64::NEG_INFINITY {
            continue;
        }
        endpoints += 1;
        let slack = required[v] - arrival[v];
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

    Ok(TimingReport {
        wns: if endpoints == 0 { f64::INFINITY } else { wns },
        tns,
        endpoints,
        worst_endpoint,
        worst_path,
    })
}
