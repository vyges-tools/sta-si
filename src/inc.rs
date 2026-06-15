//! Incremental timing — the optimizer fast path behind [`crate::sta::Timer::update`].
//!
//! After a cell swap (resize / Vt-swap), only a small **forward cone** of the timing
//! graph can change: the resized cell's own arcs, and the load its input pins present to
//! their drivers. This module persists the built graph and recomputes **only that cone**,
//! reusing the cached arrival/slew for everything outside it — `O(cone)` instead of `O(N)`.
//!
//! It is deliberately narrow. The capture is built (and the fast path is eligible) only for
//! the **simple** timing context: no SPEF (ideal interconnect, so net delays are zero and
//! there is no signal-integrity coupling that would make a local recompute unsound), flat
//! OCV derate (no AOCV/POCV bands), and no path-based pass. Within that context the per-move
//! checks below must also hold, or [`IncGraph::try_update`] returns `None` and the caller
//! does a full re-analysis:
//!   - the new cell is combinational and has the same connected footprint as the old one,
//!   - the changed cone does not reach a clock pin (a clock-network edit is not localizable),
//!   - no setup/hold endpoint in the cone changes which flop launches it (a launch-flop change
//!     would move clock-side constants this fast path caches).
//!
//! Structure: an immutable [`IncTopo`] (adjacency, arc tables, topo order, endpoint records —
//! never mutated, so it is shared and never cloned) and a cheap-to-clone [`IncState`] (the
//! per-node arrival/slew/load arrays + a small override map for swapped cell arcs). The
//! optimizer's `checkpoint`/`restore` snapshot only the state, so speculation stays cheap.
//!
//! The arithmetic here mirrors the simple-context branches of `sta::build_report`'s `relax`
//! (no Ceff shielding since there is no SPEF; net delay 0; flat derate; no sigma band). A
//! shadow-check test (`tests/timer.rs`) asserts the incremental result is byte-identical to a
//! full re-analysis after random moves, so the two code paths can never silently diverge.

use std::collections::HashMap;

use crate::liberty::{Arc, Constraint, Dir, Lib};
use crate::netlist::Netlist;
use crate::sta::{PathNode, Timing, TimingReport};

/// A reverse-adjacency timing edge into a node (simple context: net delay is zero).
#[derive(Clone)]
pub(crate) enum InEdge {
    /// driver → sink interconnect; rise→rise / fall→fall, zero delay, slew passes through.
    Net { from: usize },
    /// a cell arc (related input pin → this output pin), delay from the NLDM tables.
    Cell { from: usize, arc: Box<Arc> },
}

/// Per-lane (0 = rise, 1 = fall) propagation state retained so a cone recompute can seed
/// itself from cached upstream values and reproduce the full-graph result exactly.
#[derive(Clone)]
pub(crate) struct Lanes {
    pub arr: Vec<[f64; 2]>,
    pub slew: Vec<[f64; 2]>,
    pub from: Vec<[Option<usize>; 2]>,
}

/// A setup capture endpoint whose required time depends on the data slew at the pin
/// (`required = base − setup_constraint(ck_slew, data_slew)`); the clock-side `base`
/// (capture-clock arrival + edge relation + CRPR − uncertainty) is cached.
pub(crate) struct SetupRec {
    pub idx: usize,
    pub base: f64,
    pub cons: Vec<Constraint>,
    pub ck_slew: f64,
    pub launch_ck: Option<usize>,
}

/// A hold endpoint: `slack = arr_min + base − hold_constraint(ck_slew, data_slew_min)`,
/// with the clock-side `base` (CRPR − capture-clock arrival − edge relation − uncertainty)
/// cached.
pub(crate) struct HoldRec {
    pub idx: usize,
    pub base: f64,
    pub cons: Vec<Constraint>,
    pub ck_slew: f64,
    pub launch_ck: Option<usize>,
}

/// Immutable timing topology — built once, never mutated, never cloned.
pub(crate) struct IncTopo {
    pub n: usize,
    pub labels: Vec<String>,
    pub label2idx: HashMap<String, usize>,
    pub succ: Vec<Vec<usize>>,      // forward adjacency (cone discovery)
    pub in_edges: Vec<Vec<InEdge>>, // reverse adjacency (node recompute)
    pub topo_pos: Vec<usize>,       // rank of each node in a valid topo order
    pub is_ck: Vec<bool>,
    pub is_endpoint: Vec<bool>,
    pub excluded_setup: Vec<bool>,
    pub net_driver: HashMap<String, usize>, // net name → driver node
    pub setup_recs: Vec<SetupRec>,
    pub hold_recs: Vec<HoldRec>,
    pub late_derate: f64,
    pub early_derate: f64,
    pub input_slew: f64,
}

/// Mutable propagated state — cloned on `checkpoint`, restored on `restore`.
#[derive(Clone)]
pub(crate) struct IncState {
    pub node_load: Vec<f64>,
    pub late: Lanes,
    pub early: Lanes,
    // collapsed (worst-lane) state, kept in sync with the lanes.
    pub arrival: Vec<f64>,
    pub slew: Vec<f64>,
    pub from: Vec<Option<usize>>,
    pub arr_min: Vec<f64>,
    pub slew_min: Vec<f64>,
    pub from_min: Vec<Option<usize>>,
    pub endpoint_req: Vec<f64>, // output ports (constant) + flop D (recomputed via setup_recs)
    /// swapped-in cell arcs for resized output nodes (overrides `IncTopo::in_edges`).
    pub overrides: HashMap<usize, Vec<InEdge>>,
}

/// The persistent graph an [`crate::sta::Timer`] keeps for incremental updates.
pub(crate) struct IncGraph {
    pub topo: IncTopo,
    pub state: IncState,
}

/// worst (max) constraint over a pin's groups, interpolated at the operating slews —
/// identical to `build_report`'s `eval_cons`.
fn eval_cons(cons: &[Constraint], clk_slew: f64, data_slew: f64) -> f64 {
    cons.iter().map(|c| c.eval(clk_slew, data_slew)).fold(f64::NEG_INFINITY, f64::max)
}

/// Collapse a node's lanes to the worst lane index (max for late, min for early).
fn pick(lanes: &Lanes, v: usize, late: bool) -> usize {
    let a = lanes.arr[v];
    if late {
        if a[0] >= a[1] { 0 } else { 1 }
    } else if a[0] <= a[1] {
        0
    } else {
        1
    }
}

impl IncGraph {
    /// The current in-edges of a node: the swapped-in override if resized, else the base.
    fn in_edges(&self, v: usize) -> &[InEdge] {
        self.state.overrides.get(&v).map(Vec::as_slice).unwrap_or(&self.topo.in_edges[v])
    }

    /// Recompute one node's per-lane arrival/slew/from from its in-edges, given the current
    /// working lanes (predecessors already settled in topo order). `late` selects the
    /// max-delay (setup) vs min-delay (hold) lane choice. Mirrors `relax`'s simple branches.
    fn recompute_node(&self, v: usize, lanes: &mut Lanes, late: bool) {
        let init = if late { f64::NEG_INFINITY } else { f64::INFINITY };
        let derate = if late { self.topo.late_derate } else { self.topo.early_derate };
        let load = self.state.node_load[v];
        let mut arr = [init; 2];
        let mut slew = [self.topo.input_slew; 2];
        let mut from = [None; 2];
        for e in self.in_edges(v) {
            match e {
                InEdge::Net { from: u } => {
                    for l in 0..2 {
                        let a = lanes.arr[*u][l];
                        if !a.is_finite() {
                            continue;
                        }
                        let better = if late { a > arr[l] } else { a < arr[l] };
                        if better {
                            arr[l] = a; // zero net delay
                            slew[l] = lanes.slew[*u][l]; // sink slew passes through
                            from[l] = Some(*u);
                        }
                    }
                }
                InEdge::Cell { from: u, arc } => {
                    for ol in 0..2 {
                        let (dt, st) = if ol == 0 {
                            (&arc.cell_rise, &arc.rise_transition)
                        } else {
                            (&arc.cell_fall, &arc.fall_transition)
                        };
                        for il in 0..2 {
                            let feeds = match arc.sense.as_str() {
                                "positive_unate" => il == ol,
                                "negative_unate" => il != ol,
                                _ => true,
                            };
                            if !feeds {
                                continue;
                            }
                            let a_in = lanes.arr[*u][il];
                            if !a_in.is_finite() {
                                continue;
                            }
                            let sin = lanes.slew[*u][il];
                            let leff = load; // no SPEF → no resistive shielding
                            let (d, sout) = if !arc.ccs.is_empty() {
                                arc.ccs
                                    .delay_slew(ol == 0, sin, leff, 0.3, 0.7)
                                    .unwrap_or((dt.lookup(sin, leff), st.lookup(sin, leff)))
                            } else {
                                (dt.lookup(sin, leff), st.lookup(sin, leff))
                            };
                            let metric = a_in + d * derate; // flat derate, no sigma band
                            let better = if late { metric > arr[ol] } else { metric < arr[ol] };
                            if better {
                                arr[ol] = metric;
                                slew[ol] = sout;
                                from[ol] = Some(*u);
                            }
                        }
                    }
                }
            }
        }
        lanes.arr[v] = arr;
        lanes.slew[v] = slew;
        lanes.from[v] = from;
    }

    /// first clock pin reached walking the data path backward (launch flop's CK).
    fn launch_ck(&self, endpoint: usize, pred: &[Option<usize>]) -> Option<usize> {
        let mut v = endpoint;
        while let Some(u) = pred[v] {
            if self.topo.is_ck[u] {
                return Some(u);
            }
            v = u;
        }
        None
    }

    /// Try the fast path for a batch of cell swaps `(inst, old_cell, new_cell)`. Returns the
    /// refreshed report + query snapshot, or `None` to signal the caller to do a full
    /// re-analysis (the move is not safely localizable). On `None` the state may be partly
    /// mutated — the caller discards this `IncGraph` and rebuilds, so that is harmless.
    pub(crate) fn try_update(
        &mut self,
        nl: &Netlist,
        lib: &Lib,
        moves: &[(String, String, String)],
    ) -> Option<(TimingReport, Timing)> {
        let mut seeds: Vec<usize> = Vec::new();

        for (inst, old_c, new_c) in moves {
            let new_cell = lib.cell(new_c)?;
            let old_cell = lib.cell(old_c)?;
            // combinational only: a sequential / clock cell is not handled by this path.
            if new_cell.is_seq || new_cell.clock_pin.is_some() {
                return None;
            }
            let iref = nl.insts.iter().find(|i| &i.name == inst)?;
            for (pin, net) in &iref.conns {
                // footprint must match: same pin, same direction in both cells.
                let nd = new_cell.pins.get(pin).map(|p| p.direction)?;
                let od = old_cell.pins.get(pin).map(|p| p.direction)?;
                if nd != od {
                    return None;
                }
                match nd {
                    Dir::In => {
                        // input pin cap changed → the net's driver sees a different load.
                        let d = *self.topo.net_driver.get(net)?;
                        let old_cap = old_cell.pins[pin].load_cap();
                        let new_cap = new_cell.pins[pin].load_cap();
                        self.state.node_load[d] += new_cap - old_cap;
                        seeds.push(d);
                    }
                    Dir::Out => {
                        // output pin: rebuild its incoming cell arcs from the new cell, held
                        // as an override so the base topology stays immutable.
                        let ox = *self.topo.label2idx.get(&format!("{inst}/{pin}"))?;
                        let conn: HashMap<&str, &str> =
                            iref.conns.iter().map(|(p, n)| (p.as_str(), n.as_str())).collect();
                        let mut edges: Vec<InEdge> = Vec::new();
                        for arc in &new_cell.pins[pin].arcs {
                            if !conn.contains_key(arc.related_pin.as_str()) {
                                continue;
                            }
                            let from =
                                *self.topo.label2idx.get(&format!("{inst}/{}", arc.related_pin))?;
                            edges.push(InEdge::Cell { from, arc: Box::new(arc.clone()) });
                        }
                        self.state.overrides.insert(ox, edges);
                        seeds.push(ox);
                    }
                    _ => {}
                }
            }
        }

        // forward cone = nodes reachable from the seeds. A clock pin in the cone means the
        // edit touches the clock network — not localizable here.
        let n = self.topo.n;
        let mut in_cone = vec![false; n];
        let mut stack = seeds;
        let mut cone: Vec<usize> = Vec::new();
        while let Some(u) = stack.pop() {
            if in_cone[u] {
                continue;
            }
            in_cone[u] = true;
            if self.topo.is_ck[u] {
                return None; // clock-network edit
            }
            cone.push(u);
            for &w in &self.topo.succ[u] {
                if !in_cone[w] {
                    stack.push(w);
                }
            }
        }
        cone.sort_by_key(|&v| self.topo.topo_pos[v]);

        // recompute late + early lanes over the cone, in topo order (sources outside the
        // cone keep their cached lanes; sources inside it have no in-edges → unchanged).
        let mut late = self.state.late.clone();
        let mut early = self.state.early.clone();
        for &v in &cone {
            if self.in_edges(v).is_empty() {
                continue;
            }
            self.recompute_node(v, &mut late, true);
            self.recompute_node(v, &mut early, false);
        }
        // collapse the cone nodes into working collapsed arrays.
        let mut arrival = self.state.arrival.clone();
        let mut slew = self.state.slew.clone();
        let mut from = self.state.from.clone();
        let mut arr_min = self.state.arr_min.clone();
        let mut slew_min = self.state.slew_min.clone();
        let mut from_min = self.state.from_min.clone();
        for &v in &cone {
            let l = pick(&late, v, true);
            arrival[v] = late.arr[v][l];
            slew[v] = late.slew[v][l];
            from[v] = late.from[v][l];
            let m = pick(&early, v, false);
            arr_min[v] = early.arr[v][m];
            slew_min[v] = early.slew[v][m];
            from_min[v] = early.from[v][m];
        }

        // recompute the slew-dependent endpoint required times for cone-touched setup
        // endpoints, bailing if any changed launch flop (cached clock-side terms).
        let mut endpoint_req = self.state.endpoint_req.clone();
        for r in &self.topo.setup_recs {
            if !in_cone[r.idx] {
                continue;
            }
            if self.launch_ck(r.idx, &from) != r.launch_ck {
                return None;
            }
            let setup_v = eval_cons(&r.cons, r.ck_slew, slew[r.idx]);
            endpoint_req[r.idx] = r.base - setup_v;
        }
        for r in &self.topo.hold_recs {
            if in_cone[r.idx] && self.launch_ck(r.idx, &from_min) != r.launch_ck {
                return None;
            }
        }

        // ---- assemble the report (same reductions as build_report) ----
        let mut wns = f64::INFINITY;
        let mut tns = 0.0;
        let mut worst = None;
        let mut endpoints = 0;
        for v in 0..n {
            if !self.topo.is_endpoint[v]
                || arrival[v] == f64::NEG_INFINITY
                || self.topo.excluded_setup[v]
            {
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
        let (worst_endpoint, worst_path) = self.trace(worst, &arrival, &slew, &from);

        let mut whs = f64::INFINITY;
        let mut ths = 0.0;
        let mut worst_hold = None;
        let mut hold_endpoints = 0;
        for r in &self.topo.hold_recs {
            if arr_min[r.idx] == f64::INFINITY {
                continue;
            }
            hold_endpoints += 1;
            let hold_v = eval_cons(&r.cons, r.ck_slew, slew_min[r.idx]);
            let slack = arr_min[r.idx] + r.base - hold_v;
            if slack < 0.0 {
                ths += slack;
            }
            if slack < whs {
                whs = slack;
                worst_hold = Some(r.idx);
            }
        }
        let (worst_hold_endpoint, worst_hold_path) =
            self.trace(worst_hold, &arr_min, &slew_min, &from_min);

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
            pba_wns: None, // pba disqualifies the fast path at build time
        };
        let timing = Timing::new(
            &self.topo.labels,
            &self.topo.is_endpoint,
            &self.topo.excluded_setup,
            &arrival,
            &slew,
            &arr_min,
            &self.state.node_load,
            &endpoint_req,
        );

        // commit the recomputed state.
        self.state.late = late;
        self.state.early = early;
        self.state.arrival = arrival;
        self.state.slew = slew;
        self.state.from = from;
        self.state.arr_min = arr_min;
        self.state.slew_min = slew_min;
        self.state.from_min = from_min;
        self.state.endpoint_req = endpoint_req;
        Some((report, timing))
    }

    /// trace a worst-slack path from `end` back to its source via the `pred` pointers.
    fn trace(
        &self,
        end: Option<usize>,
        arrival: &[f64],
        slew: &[f64],
        pred: &[Option<usize>],
    ) -> (String, Vec<PathNode>) {
        match end {
            Some(mut v) => {
                let end_label = self.topo.labels[v].clone();
                let mut chain = vec![v];
                while let Some(u) = pred[v] {
                    chain.push(u);
                    v = u;
                }
                chain.reverse();
                let path = chain
                    .into_iter()
                    .map(|i| PathNode {
                        label: self.topo.labels[i].clone(),
                        arrival: arrival[i],
                        slew: slew[i],
                    })
                    .collect();
                (end_label, path)
            }
            None => (String::new(), Vec::new()),
        }
    }
}
