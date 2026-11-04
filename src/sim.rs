// SPDX-License-Identifier: Apache-2.0
//! Logic constants, and the timing arcs they disable: `set_case_analysis`, `set_logic_zero` /
//! `_one` / `_dc`, netlist ties (`.E(1'b0)`) and tie cells (`function : "0"` / `"1"`).
//!
//! The stages are the reference's (`Sim::ensureConstantsPropagated`), in its order:
//! 1. [`seed_constants`] (`Sim::seedConstants`): netlist ties first, then `set_logic_*`, then
//!    `set_case_analysis`, then tie-cell outputs. A pin that carries a constraint value keeps it:
//!    a propagated value that differs is dropped with a warning (`Sim::setPinValue`).
//! 2. [`propagate_constants`] (`Sim::propagateConstants`, not through sequentials): an instance
//!    whose input changed has each output re-evaluated from its `function` with the known pin
//!    values substituted (`Sim::evalInstance`); a tri-state output only while its enable is 1. A
//!    driver's value goes to every load of its net. A register's output (`IQ`) is never constant.
//! 3. [`find_disabled_edges`] (`Sim::findDisabledEdges`): for each instance that got a value, every
//!    arc into an output gets a sense with the constants substituted (`Sim::functionSense`): none
//!    when its input is constant or the output no longer depends on it — the arc is cut — else
//!    positive, negative or non-unate, which narrows the edges the arc pairs.
//!
//! The search then (`SearchPred0`, `searchThruSimEdge`): starts nowhere and arrives nowhere at a
//! pin held at 0 or 1, so a constant net carries no arrival and no check; a pin held `rising`
//! (`falling`) lets only its rising (falling) transitions through.
//!
//! Not modelled: an arc's `when` condition and `mode` (the reader keeps neither), and a clock
//! gate's `clock_gating_integrated_cell` output, which the reference sets to 0 when its clock or
//! enable is 0.

use crate::liberty::{Dir, Lib};
use vyges_loom::liberty_func::Func;
use crate::netlist::Netlist;
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

/// A pin's value, as the reference's `LogicValue` (unknown is absence).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Logic {
    Zero,
    One,
    /// `set_case_analysis rise` / `rising`: only rising transitions pass.
    Rise,
    /// `set_case_analysis fall` / `falling`: only falling transitions pass.
    Fall,
}

impl Logic {
    /// The value an SDC command writes: `0` `1` `zero` `one` `rise` `rising` `fall` `falling`;
    /// `None` otherwise — `X` (`set_logic_dc`) included, which [`resolve`] keeps as a constraint
    /// value of its own.
    pub fn from_sdc(v: &str) -> Option<Logic> {
        match v {
            "0" | "zero" => Some(Logic::Zero),
            "1" | "one" => Some(Logic::One),
            "rise" | "rising" => Some(Logic::Rise),
            "fall" | "falling" => Some(Logic::Fall),
            _ => None,
        }
    }

    /// `Some` for 0 / 1, `None` for a transition (not a constant).
    pub fn constant(self) -> Option<bool> {
        match self {
            Logic::Zero => Some(false),
            Logic::One => Some(true),
            _ => None,
        }
    }

    fn of(b: bool) -> Logic {
        if b {
            Logic::One
        } else {
            Logic::Zero
        }
    }

    /// Does a transition on `lane` (0 rise, 1 fall) pass a pin with this value?
    pub fn passes(self, lane: usize) -> bool {
        match self {
            Logic::Rise => lane == 0,
            Logic::Fall => lane == 1,
            _ => false,
        }
    }
}

/// One `set_case_analysis` or `set_logic_*`, as the job carries it: the objects as written.
#[derive(Debug, Clone, PartialEq)]
pub struct CaseSpec {
    /// `set_case_analysis`, `set_logic_zero`, ...
    pub cmd: String,
    pub value: String,
    /// `get_ports`, `get_pins`, `all_inputs`, or empty for bare names
    pub accessor: String,
    /// The accessor's flags and keys, as written (refused when they narrow what names match).
    pub narrowed: bool,
    pub names: Vec<String>,
}

/// A timing arc's sense once the constants are substituted (`TimingSense`, without unknown).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sense {
    Positive,
    Negative,
    NonUnate,
    /// The output no longer depends on the input: the arc is cut.
    None,
}

impl Sense {
    /// Does input edge `il` pair with output edge `ol` (0 rise, 1 fall)?
    pub fn pairs(self, il: usize, ol: usize) -> bool {
        match self {
            Sense::Positive => il == ol,
            Sense::Negative => il != ol,
            Sense::NonUnate => true,
            Sense::None => false,
        }
    }
}

/// What the constants decide.
#[derive(Debug, Default)]
pub struct Sim {
    /// Every pin and port holding a value, by name (`port`, `inst/pin`).
    pub value: HashMap<String, Logic>,
    /// The sense of each arc of an annotated instance, by `(instance, from pin, to pin)`. An arc
    /// absent here keeps its library sense.
    pub sense: HashMap<(String, String, String), Sense>,
    /// What was not applied, and why (for the run's warnings).
    pub warnings: Vec<String>,
}

impl Sim {
    /// Is `name` held at 0 or 1?
    pub fn is_constant(&self, name: &str) -> bool {
        self.value.get(name).is_some_and(|v| v.constant().is_some())
    }

    /// Does a transition on `lane` pass `name` (a pin held rising / falling passes one lane)?
    pub fn passes(&self, name: &str, lane: usize) -> bool {
        self.value.get(name).is_none_or(|v| v.passes(lane))
    }

    /// Nothing held, nothing narrowed.
    pub fn is_empty(&self) -> bool {
        self.value.is_empty() && self.sense.is_empty()
    }
}

/// The design as the propagation walks it: for each net its driver and its loads, for each pin
/// its net.
struct Conn<'a> {
    nl: &'a Netlist,
    lib: &'a Lib,
    /// net -> the pins (and ports) on it: (name, is driver)
    net_pins: BTreeMap<&'a str, Vec<(String, bool)>>,
    /// pin or port name -> its net
    net_of: HashMap<String, &'a str>,
    /// `inst/pin` -> instance index, for input pins (whose change re-evaluates the instance)
    inst_of_input: HashMap<String, usize>,
    /// instance name -> index
    inst_idx: HashMap<&'a str, usize>,
    /// every name an object can match: ports and instance pins, ties included
    ports: Vec<String>,
    pins: Vec<String>,
}

fn conn<'a>(nl: &'a Netlist, lib: &'a Lib) -> Conn<'a> {
    let mut c = Conn {
        nl,
        lib,
        net_pins: BTreeMap::new(),
        net_of: HashMap::new(),
        inst_of_input: HashMap::new(),
        inst_idx: nl.insts.iter().enumerate().map(|(k, i)| (i.name.as_str(), k)).collect(),
        ports: Vec::new(),
        pins: Vec::new(),
    };
    for p in &nl.inputs {
        c.net_pins.entry(p.as_str()).or_default().push((p.clone(), true));
        c.net_of.insert(p.clone(), p.as_str());
        c.ports.push(p.clone());
    }
    for p in &nl.outputs {
        c.net_pins.entry(p.as_str()).or_default().push((p.clone(), false));
        c.net_of.insert(p.clone(), p.as_str());
        c.ports.push(p.clone());
    }
    for (k, inst) in nl.insts.iter().enumerate() {
        let Some(cell) = lib.cell(&inst.cell) else { continue };
        for (pin, net) in &inst.conns {
            let name = format!("{}/{pin}", inst.name);
            let dir = cell.pins.get(pin).map(|p| p.direction);
            let driver = matches!(dir, Some(Dir::Out | Dir::Inout));
            c.net_pins.entry(net.as_str()).or_default().push((name.clone(), driver));
            c.net_of.insert(name.clone(), net.as_str());
            if matches!(dir, Some(Dir::In | Dir::Inout)) {
                c.inst_of_input.insert(name.clone(), k);
            }
            c.pins.push(name);
        }
        for (pin, _) in inst.ties.iter().filter(|(p, _)| !p.is_empty()) {
            let name = format!("{}/{pin}", inst.name);
            c.inst_of_input.insert(name.clone(), k);
            c.pins.push(name);
        }
    }
    c
}

/// The constants of `nl` under `cases` (the SDC's `set_case_analysis` and `set_logic_*`, in file
/// order), and the arcs they narrow — the reference's sequence (see the module doc).
pub fn run(nl: &Netlist, lib: &Lib, cases: &[CaseSpec]) -> Sim {
    let c = conn(nl, lib);
    let mut st = State { sim: Sim::default(), constraint: HashMap::new(), queue: VecDeque::new(), annotate: HashSet::new() };
    seed_constants(&c, cases, &mut st);
    propagate_constants(&c, &mut st);
    find_disabled_edges(&c, &mut st);
    st.sim
}

struct State {
    sim: Sim,
    /// the constraint value of each pin (`set_case_analysis` over `set_logic_*`); `None` is X
    /// (`set_logic_dc`), a constraint value too: a 0 or 1 reaching the pin differs from it
    constraint: HashMap<String, Option<Logic>>,
    /// instances to re-evaluate, in order (`eval_queue_`)
    queue: VecDeque<usize>,
    /// instances that got a value on a pin (`instances_to_annotate_`)
    annotate: HashSet<usize>,
}

/// `Sim::seedConstants`: netlist ties, `set_logic_*`, `set_case_analysis`, tie-cell outputs.
fn seed_constants(c: &Conn, cases: &[CaseSpec], st: &mut State) {
    // the constraint values first: `setPinValue` checks each value against them
    let logic: Vec<&CaseSpec> = cases.iter().filter(|s| s.cmd != "set_case_analysis").collect();
    let case: Vec<&CaseSpec> = cases.iter().filter(|s| s.cmd == "set_case_analysis").collect();
    let logic_pins = resolve(c, &logic, &mut st.sim.warnings);
    let case_pins = resolve(c, &case, &mut st.sim.warnings);
    for (pin, v) in logic_pins.iter().chain(&case_pins) {
        st.constraint.insert(pin.clone(), *v);
    }
    // enqueueConstantPinInputs: pins tied in the netlist
    for inst in &c.nl.insts {
        if c.lib.cell(&inst.cell).is_none() {
            continue;
        }
        for (pin, b) in inst.ties.iter().filter(|(p, _)| !p.is_empty()) {
            set_pin_value(c, st, &format!("{}/{pin}", inst.name), Some(Logic::of(*b)));
        }
    }
    for (pin, v) in logic_pins.into_iter().chain(case_pins) {
        set_pin_value(c, st, &pin, v);
    }
    // setConstFuncPins: tie-cell outputs
    for inst in &c.nl.insts {
        let Some(cell) = c.lib.cell(&inst.cell) else { continue };
        for (pin, _) in &inst.conns {
            if let Some(p) = cell.pins.get(pin).filter(|p| p.is_constant()) {
                let one = p.function.as_deref().is_some_and(|f| f.trim().trim_matches('"').trim() == "1");
                set_pin_value(c, st, &format!("{}/{pin}", inst.name), Some(Logic::of(one)));
            }
        }
    }
}

/// The pins and ports each command names, with its value (`None` for X, `set_logic_dc`), in
/// command order. An object this timer cannot resolve is not applied and said.
fn resolve(c: &Conn, specs: &[&CaseSpec], warn: &mut Vec<String>) -> Vec<(String, Option<Logic>)> {
    let mut out = Vec::new();
    for s in specs {
        // `set_logic_dc` writes X (`Sdc::setLogicValue`): a constraint value, not an absence
        let v = match Logic::from_sdc(&s.value) {
            Some(v) => Some(v),
            None if s.value.eq_ignore_ascii_case("x") => None,
            None => continue,
        };
        let pool: Vec<&String> = match s.accessor.as_str() {
            _ if s.narrowed => {
                warn.push(format!("{} {}: objects selected by a filter or option are not resolved", s.cmd, s.value));
                continue;
            }
            "get_ports" => c.ports.iter().collect(),
            "get_pins" => c.pins.iter().collect(),
            "all_inputs" => c.nl.inputs.iter().collect(),
            "" => c.ports.iter().chain(&c.pins).collect(),
            other => {
                warn.push(format!("{} {}: objects of [{other} ...] are not pins or ports", s.cmd, s.value));
                continue;
            }
        };
        let names: Vec<&String> = if s.accessor == "all_inputs" { pool } else {
            let hit: Vec<&String> = pool.into_iter().filter(|n| s.names.iter().any(|p| crate::exc::named(p, n))).collect();
            if hit.is_empty() {
                warn.push(format!("{} {}: {} matches no pin or port", s.cmd, s.value, s.names.join(" ")));
            }
            hit
        };
        out.extend(names.into_iter().map(|n| (n.clone(), v)));
    }
    out
}

/// `Sim::setPinValue`: a pin's value, unless its constraint says otherwise; an input's change
/// re-evaluates its instance, a driver's goes to its net's loads. `None` is unknown.
fn set_pin_value(c: &Conn, st: &mut State, pin: &str, value: Option<Logic>) {
    if let Some(&k) = st.constraint.get(pin) {
        if value != k {
            if let Some(v) = value {
                let k = k.map_or("X".to_string(), |k| format!("{k:?}"));
                st.sim.warnings.push(format!("propagated logic value {v:?} differs from the constraint value {k} on {pin}"));
            }
            return;
        }
    }
    if st.sim.value.get(pin).copied() == value {
        return;
    }
    match value {
        Some(v) => st.sim.value.insert(pin.to_string(), v),
        None => st.sim.value.remove(pin),
    };
    if let Some(&k) = c.inst_of_input.get(pin) {
        st.annotate.insert(k);
        if st.queue.back() != Some(&k) {
            st.queue.push_back(k);
        }
        return;
    }
    if let Some(&k) = pin.rsplit_once('/').and_then(|(inst, _)| c.inst_idx.get(inst)) {
        st.annotate.insert(k);
    }
    let Some(net) = c.net_of.get(pin) else { return };
    let driver = c.net_pins[net].iter().any(|(n, d)| n == pin && *d);
    if driver {
        let loads: Vec<String> = c.net_pins[net].iter().filter(|(n, d)| !*d && n != pin).map(|(n, _)| n.clone()).collect();
        for l in loads {
            set_pin_value(c, st, &l, value);
        }
    }
}

/// `Sim::propagateConstants` (not through sequentials): re-evaluate queued instances in order.
fn propagate_constants(c: &Conn, st: &mut State) {
    while let Some(k) = st.queue.pop_front() {
        eval_instance(c, st, k);
    }
}

/// The value the instance's pins hold, for an expression of its cell.
fn pin_values<'s>(st: &'s State, inst: &str) -> impl Fn(&str) -> Option<bool> + 's {
    let pre = format!("{inst}/");
    move |port: &str| st.sim.value.get(&format!("{pre}{port}")).and_then(|v| v.constant())
}

/// `Sim::evalInstance`: each output from its function; a tri-state one only while enabled.
fn eval_instance(c: &Conn, st: &mut State, k: usize) {
    let inst = &c.nl.insts[k];
    let Some(cell) = c.lib.cell(&inst.cell) else { return };
    let is_port = |p: &str| cell.pins.contains_key(p);
    let mut updates = Vec::new();
    for (pname, p) in &cell.pins {
        if !matches!(p.direction, Dir::Out | Dir::Inout) {
            continue;
        }
        let Some(f) = p.function.as_deref().and_then(|f| Func::parse_in(f, is_port)) else { continue };
        let vals = pin_values(st, &inst.name);
        let value = match p.three_state().and_then(|t| Func::parse_in(t, is_port)) {
            // `three_state` is the DISABLE: enabled when it is 0
            Some(dis) => match eval(&dis, &vals) {
                Some(false) => eval(&f, &vals),
                _ => None,
            },
            None => eval(&f, &vals),
        }
        .map(Logic::of);
        let name = format!("{}/{pname}", inst.name);
        if value != st.sim.value.get(&name).copied() {
            updates.push((name, value));
        }
    }
    for (name, value) in updates {
        set_pin_value(c, st, &name, value);
    }
}

/// `Sim::findDisabledEdges`: the sense of every arc of each annotated instance.
fn find_disabled_edges(c: &Conn, st: &mut State) {
    let mut ks: Vec<usize> = st.annotate.iter().copied().collect();
    ks.sort_unstable();
    for k in ks {
        let inst = &c.nl.insts[k];
        let Some(cell) = c.lib.cell(&inst.cell) else { continue };
        let is_port = |p: &str| cell.pins.contains_key(p);
        for (to, p) in &cell.pins {
            for arc in &p.arcs {
                let from = &arc.related_pin;
                let from_name = format!("{}/{from}", inst.name);
                let sense = if st.sim.is_constant(&from_name) {
                    Some(Sense::None)
                } else {
                    let f = p.function.as_deref().and_then(|f| Func::parse_in(f, is_port));
                    let vals = pin_values(st, &inst.name);
                    match f {
                        Some(f) if f.ports().contains(&from.as_str()) => {
                            let enabled = p.three_state().and_then(|t| Func::parse_in(t, is_port)).map(|d| eval(&d, &vals));
                            if enabled == Some(Some(true)) {
                                Some(Sense::None) // disabled while the disable holds 1
                            } else {
                                function_sense(&f, from, &vals)
                            }
                        }
                        _ => None,
                    }
                };
                if let Some(s) = sense {
                    st.sim.sense.insert((inst.name.clone(), from.clone(), to.clone()), s);
                }
            }
        }
    }
}

/// The free ports of `f` (those `vals` does not hold), or `None` when too many to enumerate.
fn free_ports<'f>(f: &'f Func, vals: &dyn Fn(&str) -> Option<bool>, skip: &str) -> Option<Vec<&'f str>> {
    let free: Vec<&str> = f.ports().into_iter().filter(|p| *p != skip && vals(p).is_none()).collect();
    (free.len() <= 16).then_some(free)
}

/// Port `p` under assignment `bits` of the `free` ports, else its held value.
fn assigned(free: &[&str], bits: u32, vals: &dyn Fn(&str) -> Option<bool>, p: &str) -> bool {
    match free.iter().position(|q| *q == p) {
        Some(i) => bits >> i & 1 == 1,
        None => vals(p).unwrap_or(false),
    }
}

fn eval_at(f: &Func, v: &dyn Fn(&str) -> bool) -> bool {
    match f {
        Func::Port(p) => v(p),
        Func::Not(a) => !eval_at(a, v),
        Func::Or(a, b) => eval_at(a, v) || eval_at(b, v),
        Func::And(a, b) => eval_at(a, v) && eval_at(b, v),
        Func::Xor(a, b) => eval_at(a, v) != eval_at(b, v),
        Func::One => true,
        Func::Zero => false,
    }
}

/// `Sim::evalExpr`: the expression's value with the held pins substituted — `Some` only when it
/// is the same for every value of the others (the reference's BDD collapsing to 0 or 1).
pub fn eval(f: &Func, vals: &dyn Fn(&str) -> Option<bool>) -> Option<bool> {
    let free = free_ports(f, vals, "")?;
    let mut seen = None;
    for bits in 0u32..(1 << free.len()) {
        let r = eval_at(f, &|p| assigned(&free, bits, vals, p));
        match seen {
            None => seen = Some(r),
            Some(s) if s != r => return None,
            _ => {}
        }
    }
    seen
}

/// `Sim::functionSense`: how `f` follows `port` with the held pins substituted — increasing
/// (positive), decreasing (negative), both (none: no longer depends on it) or neither
/// (non-unate). `None` when there are too many inputs to tell.
pub fn function_sense(f: &Func, port: &str, vals: &dyn Fn(&str) -> Option<bool>) -> Option<Sense> {
    let free = free_ports(f, vals, port)?;
    let (mut inc, mut dec) = (true, true);
    for bits in 0u32..(1 << free.len()) {
        let at = |x: bool, p: &str| if p == port { x } else { assigned(&free, bits, vals, p) };
        let (lo, hi) = (eval_at(f, &|p| at(false, p)), eval_at(f, &|p| at(true, p)));
        inc &= !lo || hi;
        dec &= lo || !hi;
    }
    Some(match (inc, dec) {
        (true, true) => Sense::None,
        (true, false) => Sense::Positive,
        (false, true) => Sense::Negative,
        (false, false) => Sense::NonUnate,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(s: &str) -> Func {
        Func::parse_in(s, |_| true).unwrap()
    }

    /// `evalExpr`: constant only when the held inputs decide it.
    #[test]
    fn an_expression_is_constant_only_when_the_held_pins_decide_it() {
        let b0 = |p: &str| (p == "B").then_some(false);
        assert_eq!(eval(&f("A&B"), &b0), Some(false));
        assert_eq!(eval(&f("A|B"), &b0), None);
        assert_eq!(eval(&f("A|!A"), &|_| None), Some(true), "a tautology is constant");
    }

    /// `functionSense` by monotonicity: a mux select held picks one data input and cuts the
    /// other; an xor input held at 1 inverts the other.
    #[test]
    fn a_held_pin_narrows_or_cuts_the_other_arcs() {
        let mux = f("(A0&!S)|(A1&S)");
        let s0 = |p: &str| (p == "S").then_some(false);
        assert_eq!(function_sense(&mux, "A0", &s0), Some(Sense::Positive));
        assert_eq!(function_sense(&mux, "A1", &s0), Some(Sense::None));
        assert_eq!(function_sense(&mux, "S", &|_| None), Some(Sense::NonUnate));
        let b1 = |p: &str| (p == "B").then_some(true);
        assert_eq!(function_sense(&f("A^B"), "A", &b1), Some(Sense::Negative));
        assert!(Sense::Negative.pairs(0, 1) && !Sense::Negative.pairs(0, 0));
    }
}
