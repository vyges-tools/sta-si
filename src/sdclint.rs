//! SDC constraint **linter** — completeness and consistency checks on a design's
//! constraints, independent of the timing run.
//!
//! A correct slack report is worthless if the constraints are wrong: an
//! unconstrained input has *no* path to check, a missing clock leaves registers
//! untimed, a clock on a port the design doesn't have is a typo that silently does
//! nothing. STA tools compute timing on whatever constraints they're given; they
//! rarely tell you the constraints themselves are incomplete. This module does —
//! purely structurally, from the same SDC + netlist (+ Liberty) the timing engine
//! already loads.
//!
//! It is deliberately conservative: it flags only what is structurally certain
//! (an output with no `set_output_delay`, a clock period of zero, two clocks of the
//! same name), so a clean lint means something.
//!
//! Three **depth passes** sit on that floor, sharing one piece of machinery — a local view of
//! the netlist graph:
//!
//!   * **clock-tree tracing** — which SDC clock actually *reaches* each register, rather than
//!     which one was declared. A clock defined on a port that drives nothing leaves every
//!     register untimed while the SDC looks populated, and a design-wide "registers but no
//!     clocks" check cannot see it.
//!   * **exception reachability** — does a `set_false_path` / `set_multicycle_path` name a
//!     path that structurally exists? A typo'd exception does not fail; it silently does
//!     nothing, and the path the author meant stays timed. That is the failure mode worth
//!     closing, because a wrong constraint does not produce a wrong-looking report — it
//!     produces a **clean** one.
//!   * **endpoint coverage** — one number: what fraction of timing endpoints are constrained
//!     at all. An uncovered endpoint is a path nobody is checking.

use std::collections::{BTreeMap, BTreeSet};

use crate::liberty::{Dir, Lib};
use crate::names::instance_of;
use crate::netlist::Netlist;
use crate::sdc::Sdc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warning,
    /// Not a defect — a measurement worth carrying in the report, like endpoint coverage.
    /// Kept distinct so `errors()`/`warnings()` and any CI gate built on them are unaffected
    /// by adding one.
    Info,
}

impl Severity {
    pub fn tag(self) -> &'static str {
        match self {
            Severity::Error => "error",
            Severity::Warning => "warning",
            Severity::Info => "info",
        }
    }
}

/// Does a `get_ports` pattern match a port name? SDC object patterns are globs — `paddr_i[*]`
/// names a whole bus — so an exact-string comparison reports every bus-wide constraint in a
/// real SDC as targeting a port that does not exist. That is a false positive on valid input,
/// which is the one thing a linter cannot afford.
///
/// `*` matches any run of characters, `?` any single one. Anything without a metacharacter
/// compares exactly, so the common case costs nothing.
fn glob_match(pat: &str, name: &str) -> bool {
    if !pat.contains('*') && !pat.contains('?') {
        return pat == name;
    }
    let (p, n): (Vec<char>, Vec<char>) = (pat.chars().collect(), name.chars().collect());
    // Backtracking matcher: `star` remembers the last `*` so a failed tail can retry one
    // character later, which is what makes `a*b*c` work without regex.
    let (mut pi, mut ni) = (0usize, 0usize);
    let (mut star, mut retry) = (usize::MAX, 0usize);
    while ni < n.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == n[ni]) {
            pi += 1;
            ni += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = pi;
            retry = ni;
            pi += 1;
        } else if star != usize::MAX {
            pi = star + 1;
            retry += 1;
            ni = retry;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

/// How many individual ports/registers a per-object finding lists before collapsing into a
/// count. The findings that matter are lost in a list of hundreds, and the count is what a
/// reader acts on anyway.
const PORT_LIST_CAP: usize = 20;

#[derive(Debug, Clone)]
pub struct Finding {
    pub severity: Severity,
    pub code: &'static str, // stable short id, e.g. "clock-period"
    pub message: String,
}

impl Finding {
    fn err(code: &'static str, message: String) -> Finding {
        Finding {
            severity: Severity::Error,
            code,
            message,
        }
    }
    fn warn(code: &'static str, message: String) -> Finding {
        Finding {
            severity: Severity::Warning,
            code,
            message,
        }
    }
}

#[derive(Debug, Default)]
pub struct LintReport {
    pub findings: Vec<Finding>,
}

impl LintReport {
    /// The endpoint-coverage percentage, parsed back out of its finding.
    ///
    /// Kept as a derived accessor rather than a second stored field so there is exactly one
    /// place the number lives and no way for the two to disagree.
    pub fn endpoint_coverage_pct(&self) -> Option<f64> {
        let m = &self
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")?
            .message;
        m.split_whitespace()
            .find_map(|t| t.strip_suffix('%')?.parse().ok())
    }

    pub fn errors(&self) -> usize {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Error)
            .count()
    }
    pub fn warnings(&self) -> usize {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Warning)
            .count()
    }
}

/// Lint the SDC against the design. `lib` lets the linter tell registers from
/// combinational cells (so "registers but no clock" is real, not a guess).
pub fn lint(nl: &Netlist, sdc: &Sdc, lib: &Lib) -> LintReport {
    lint_with_meta(nl, sdc, lib, None)
}

/// As [`lint`], plus the IP's **declared** clock domains from `vyges-metadata.json`.
///
/// Everything else here reasons from the netlist and the SDC, which between them cannot answer
/// "how many clocks *should* this design have had?" — a netlist has no opinion about intent. The
/// declaration supplies that, and it is the only way to catch a flow config that named one
/// `CLOCK_PORT` on a three-clock IP before the block is hardened rather than after.
pub fn lint_with_meta(
    nl: &Netlist,
    sdc: &Sdc,
    lib: &Lib,
    meta: Option<&crate::ipmeta::IpMeta>,
) -> LintReport {
    let mut f = Vec::new();

    let inputs: BTreeSet<&str> = nl.inputs.iter().map(String::as_str).collect();
    let outputs: BTreeSet<&str> = nl.outputs.iter().map(String::as_str).collect();
    // every net name that exists anywhere in the design
    let mut nets: BTreeSet<&str> = inputs.iter().chain(outputs.iter()).copied().collect();
    for inst in &nl.insts {
        for (_, n) in &inst.conns {
            nets.insert(n.as_str());
        }
    }

    // --- clocks -------------------------------------------------------------
    let has_registers = nl
        .insts
        .iter()
        .any(|i| lib.cells.get(&i.cell).map(|c| c.is_seq).unwrap_or(false));
    if sdc.clocks.is_empty() && has_registers {
        f.push(Finding::err(
            "no-clock",
            "design has registers but the SDC defines no clocks".into(),
        ));
    }

    let mut by_name: BTreeMap<&str, u32> = BTreeMap::new();
    let mut by_source: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for c in &sdc.clocks {
        *by_name.entry(c.name.as_str()).or_default() += 1;
        by_source
            .entry(c.source.as_str())
            .or_default()
            .push(c.name.as_str());
        if c.period <= 0.0 {
            f.push(Finding::err(
                "clock-period",
                format!(
                    "clock `{}` has a non-positive period ({} ns)",
                    c.name, c.period
                ),
            ));
        }
        // a clock whose source is neither a port nor any known net (and not an
        // unresolved inst/pin path) is almost certainly a typo.
        let s = c.source.as_str();
        if !s.contains('/') && !nets.contains(s) {
            f.push(Finding::warn(
                "clock-source",
                format!(
                    "clock `{}` source `{}` is not a port or net in the design",
                    c.name, s
                ),
            ));
        }
    }
    for (name, n) in by_name {
        if n > 1 {
            f.push(Finding::err(
                "dup-clock-name",
                format!("clock `{name}` is defined {n} times"),
            ));
        }
    }
    for (src, names) in by_source {
        if names.len() > 1 {
            f.push(Finding::warn(
                "dup-clock-source",
                format!(
                    "source `{src}` carries {} clocks: {}",
                    names.len(),
                    names.join(", ")
                ),
            ));
        }
    }

    // clock ports are exempt from I/O-delay requirements.
    let clock_ports: BTreeSet<&str> = sdc.clocks.iter().map(|c| c.source.as_str()).collect();

    // --- input / output delay coverage -------------------------------------
    let in_default = sdc.input_delays.iter().any(|d| d.default);
    let out_default = sdc.output_delays.iter().any(|d| d.default);
    let in_ports: BTreeSet<&str> = sdc
        .input_delays
        .iter()
        .flat_map(|d| d.ports.iter().map(String::as_str))
        .collect();
    let out_ports: BTreeSet<&str> = sdc
        .output_delays
        .iter()
        .flat_map(|d| d.ports.iter().map(String::as_str))
        .collect();

    // Listed, then capped. A top-level pad wrapper has hundreds of static configuration
    // outputs that legitimately carry no output delay, and one warning per port buries the
    // findings that matter under a wall the reader scrolls past — which is how a checker gets
    // switched off. Same treatment the per-register check gets.
    let bare_in: Vec<&str> = nl
        .inputs
        .iter()
        .map(String::as_str)
        .filter(|p| {
            !clock_ports.contains(p) && !in_default && !in_ports.iter().any(|q| glob_match(q, p))
        })
        .collect();
    let bare_out: Vec<&str> = nl
        .outputs
        .iter()
        .map(String::as_str)
        .filter(|p| !out_default && !out_ports.iter().any(|q| glob_match(q, p)))
        .collect();
    for (code, kind, cmd, list) in [
        ("unconstrained-input", "input", "set_input_delay", &bare_in),
        (
            "unconstrained-output",
            "output",
            "set_output_delay",
            &bare_out,
        ),
    ] {
        for p in list.iter().take(PORT_LIST_CAP) {
            f.push(Finding::warn(code, format!("{kind} `{p}` has no {cmd}")));
        }
        if list.len() > PORT_LIST_CAP {
            f.push(Finding::warn(
                code,
                format!(
                    "... and {} more {kind}s with no {cmd} ({} in total)",
                    list.len() - PORT_LIST_CAP,
                    list.len()
                ),
            ));
        }
    }

    // an explicit delay targeting a port the design doesn't have
    for p in in_ports
        .iter()
        .filter(|p| !inputs.iter().any(|i| glob_match(p, i)))
    {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_input_delay targets `{p}`, not an input of the design"),
        ));
    }
    for p in out_ports
        .iter()
        .filter(|p| !outputs.iter().any(|o| glob_match(p, o)))
    {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_output_delay targets `{p}`, not an output of the design"),
        ));
    }

    // --- the IP's own declaration, if we were given it -----------------------
    if let Some(m) = meta.filter(|m| !m.is_empty()) {
        let declared_clocks: BTreeSet<&str> =
            sdc.clocks.iter().map(|c| c.source.as_str()).collect();
        for d in &m.domains {
            if !declared_clocks.contains(d.clock.as_str()) {
                f.push(Finding::err(
                    "domain-not-constrained",
                    format!(
                        "the IP declares clock domain `{}` on port `{}`, and the SDC defines no \
                         clock there — every register in that domain is untimed",
                        d.name, d.clock
                    ),
                ));
            }
        }
        // A declared async relationship the SDC does not express means cross-domain paths get
        // timed against an edge relation that does not exist, which produces violations that
        // are not real. Distinct from the above and only a warning: the timing is pessimistic
        // rather than absent.
        if m.async_groups.len() > 1 && sdc.async_groups.len() < 2 {
            f.push(Finding::warn(
                "async-groups-not-declared",
                format!(
                    "the IP declares {} mutually asynchronous clock domains; the SDC has no \
                     set_clock_groups -asynchronous, so paths between them are timed as if \
                     related",
                    m.async_groups.len()
                ),
            ));
        }
    }

    // --- depth passes: clock reach, exception reachability, coverage ---------
    let g = Graph::build(nl, lib);
    let clock_srcs: BTreeMap<&str, &str> = sdc
        .clocks
        .iter()
        .map(|c| (c.source.as_str(), c.name.as_str()))
        .collect();

    // Which clock actually reaches each register. Declared is not reaching.
    let mut unclocked: Vec<&str> = Vec::new();
    let mut clocked = 0usize;
    for (i, inst) in nl.insts.iter().enumerate() {
        let Some(cell) = lib.cells.get(&inst.cell) else {
            continue;
        };
        if !cell.is_seq {
            continue;
        }
        let reached = cell
            .clock_pin
            .as_ref()
            .and_then(|cp| g.net_at(i, cp))
            .and_then(|cn| g.trace_to_source(cn, &clock_srcs));
        match reached {
            Some(_) => clocked += 1,
            None => unclocked.push(inst.name.as_str()),
        }
    }
    // Only worth saying per-register when some clock exists; with none the design-wide error
    // above already says it, and repeating it per register is noise.
    if !sdc.clocks.is_empty() {
        for name in unclocked.iter().take(PORT_LIST_CAP) {
            f.push(Finding::err(
                "register-no-clock-reaches",
                format!("register `{name}`: no SDC clock reaches its clock pin — it is untimed"),
            ));
        }
        if unclocked.len() > PORT_LIST_CAP {
            f.push(Finding::err(
                "register-no-clock-reaches",
                format!(
                    "... and {} more registers no clock reaches ({} in total)",
                    unclocked.len() - PORT_LIST_CAP,
                    unclocked.len()
                ),
            ));
        }
    }

    // Exception reachability. `*` means "any" and is not checkable; a named endpoint is.
    for e in &sdc.exceptions {
        let kind = match e.kind {
            crate::sdc::ExcKind::FalsePath => "set_false_path",
            crate::sdc::ExcKind::Multicycle(_) => "set_multicycle_path",
        };
        // Every named endpoint, not the first — an exception can cut a whole bus, and each
        // member is separately capable of being a typo or of naming a path that is not there.
        let known_object = |o: &String| {
            g.knows(o)
                || clock_srcs.contains_key(o.as_str())
                || sdc.clocks.iter().any(|c| &c.name == o)
        };
        for (side, obj) in e.named_endpoints() {
            if !known_object(obj) {
                f.push(Finding::warn(
                    "exception-unknown-object",
                    format!(
                        "{kind} {side} `{obj}` names nothing in the design — \
                         that endpoint of the exception does nothing"
                    ),
                ));
            }
        }
        // Reachability. Pair by pair when both sides are named, because the answer a reader
        // needs is the specific dead pair rather than that some pair is dead.
        //
        // One-sided exceptions are checked too, and they are the common case in real SDCs —
        // `set_false_path -from [list ...]` with no `-to` at all. An earlier version skipped
        // them, which meant this check quietly did nothing on exactly the shape most designs
        // write. For those, dead means the named object sits on no timing path at all, so
        // cutting it cuts nothing.
        let named_from: Vec<&String> = e
            .from
            .iter()
            .filter(|o| o.as_str() != "*" && known_object(o))
            .collect();
        let named_to: Vec<&String> =
            e.to.iter()
                .filter(|o| o.as_str() != "*" && known_object(o))
                .collect();
        let mut dead = Vec::new();
        match (named_from.is_empty(), named_to.is_empty()) {
            (false, false) => {
                for from in &named_from {
                    for to in &named_to {
                        if !g.reaches(from, to) {
                            dead.push(format!("`{from}` -> `{to}`: no structural path"));
                        }
                    }
                }
            }
            (false, true) => {
                for from in &named_from {
                    if !g.launches_anything(from) {
                        dead.push(format!("`{from}` launches no timing path"));
                    }
                }
            }
            (true, false) => {
                for to in &named_to {
                    if !g.captures_anything(to) {
                        dead.push(format!("`{to}` captures no timing path"));
                    }
                }
            }
            (true, true) => {}
        }
        for what in dead.iter().take(PORT_LIST_CAP) {
            f.push(Finding::warn(
                "exception-unreachable",
                format!(
                    "{kind} {what} — that part of the exception is dead, and any path the \
                     author meant is still timed"
                ),
            ));
        }
        if dead.len() > PORT_LIST_CAP {
            f.push(Finding::warn(
                "exception-unreachable",
                format!(
                    "... and {} more dead {kind} endpoints ({} in total)",
                    dead.len() - PORT_LIST_CAP,
                    dead.len()
                ),
            ));
        }
    }

    // Endpoint coverage, as one number.
    let seq_total = clocked + unclocked.len();
    let out_covered = nl
        .outputs
        .iter()
        .filter(|p| out_default || out_ports.iter().any(|q| glob_match(q, p)))
        .count();
    let total = seq_total + nl.outputs.len();
    let covered = clocked + out_covered;
    let pct = if total == 0 {
        100.0
    } else {
        100.0 * covered as f64 / total as f64
    };
    f.push(Finding {
        severity: if covered < total {
            Severity::Warning
        } else {
            Severity::Info
        },
        code: "endpoint-coverage",
        message: format!(
            "endpoint coverage {pct:.1}% — {covered} of {total} constrained ({clocked}/{seq_total} \
             registers clocked, {out_covered}/{} outputs with a delay)",
            nl.outputs.len()
        ),
    });

    f.sort_by(|a, b| (a.severity as u8, a.code).cmp(&(b.severity as u8, b.code)));
    LintReport { findings: f }
}

/// A local forward/backward view of the netlist, built once and shared by the depth passes.
///
/// Deliberately local rather than borrowed from the timing engine: this module's contract is
/// that it runs *independent of the timing run*, so a constraint problem stays reportable
/// without first computing slack from constraints we already suspect.
struct Graph {
    driver: BTreeMap<String, Option<usize>>, // net -> driving instance (None = primary input)
    sinks: BTreeMap<String, Vec<usize>>,     // net -> instances with it on an input pin
    is_seq: Vec<bool>,
    known: BTreeSet<String>, // every net, port and instance name that exists
    inputs: BTreeSet<String>,
    outputs: BTreeSet<String>,
    inst_index: BTreeMap<String, usize>,
    conns: Vec<Vec<(String, String)>>,
    out_pins: Vec<Vec<String>>,
}

impl Graph {
    fn build(nl: &Netlist, lib: &Lib) -> Graph {
        let mut g = Graph {
            driver: BTreeMap::new(),
            sinks: BTreeMap::new(),
            is_seq: Vec::new(),
            known: BTreeSet::new(),
            inputs: nl.inputs.iter().cloned().collect(),
            outputs: nl.outputs.iter().cloned().collect(),
            inst_index: BTreeMap::new(),
            conns: Vec::new(),
            out_pins: Vec::new(),
        };
        for p in nl.inputs.iter().chain(nl.outputs.iter()) {
            g.driver.entry(p.clone()).or_insert(None);
            g.known.insert(p.clone());
        }
        for (i, inst) in nl.insts.iter().enumerate() {
            let cell = lib.cells.get(&inst.cell);
            g.is_seq.push(cell.map(|c| c.is_seq).unwrap_or(false));
            g.inst_index.insert(inst.name.clone(), i);
            g.known.insert(inst.name.clone());
            g.conns.push(inst.conns.clone());
            let mut outs = Vec::new();
            for (pin, net) in &inst.conns {
                g.known.insert(net.clone());
                if cell.and_then(|c| c.pins.get(pin)).map(|p| p.direction) == Some(Dir::Out) {
                    g.driver.insert(net.clone(), Some(i));
                    outs.push(pin.clone());
                } else {
                    g.sinks.entry(net.clone()).or_default().push(i);
                }
            }
            g.out_pins.push(outs);
        }
        g
    }

    /// `inst/pin` paths resolve on the instance half.
    fn knows(&self, obj: &str) -> bool {
        self.known.contains(obj) || self.known.contains(instance_of(obj))
    }

    fn net_at(&self, inst: usize, pin: &str) -> Option<&str> {
        self.conns[inst]
            .iter()
            .find(|(p, _)| p == pin)
            .map(|(_, n)| n.as_str())
    }

    /// Walk a clock net back through combinational cells to a declared SDC clock source.
    fn trace_to_source<'a>(&self, net: &str, srcs: &BTreeMap<&str, &'a str>) -> Option<&'a str> {
        let mut seen = BTreeSet::new();
        let mut stack = vec![net.to_string()];
        while let Some(n) = stack.pop() {
            if let Some(d) = srcs.get(n.as_str()) {
                return Some(d);
            }
            if !seen.insert(n.clone()) {
                continue;
            }
            let Some(Some(i)) = self.driver.get(&n).copied() else {
                continue; // a primary input that is not a clock source, or undriven
            };
            if self.is_seq[i] {
                continue; // divided/gated off a flop — not traced in v0
            }
            for (pin, nn) in &self.conns[i] {
                if !self.out_pins[i].contains(pin) {
                    stack.push(nn.clone());
                }
            }
        }
        None
    }

    /// The nets an SDC object launches from: a port is itself, an instance is its outputs.
    fn start_nets(&self, obj: &str) -> Vec<String> {
        let base = instance_of(obj);
        if let Some(&i) = self.inst_index.get(base) {
            return self.out_pins[i]
                .iter()
                .filter_map(|p| self.net_at(i, p).map(str::to_string))
                .collect();
        }
        if self.known.contains(obj) {
            return vec![obj.to_string()];
        }
        Vec::new()
    }

    /// Does `from` launch any timing path at all — reaching a register or an output port?
    ///
    /// A `-from` naming an object that drives nothing cuts nothing. This is what makes a
    /// one-sided exception checkable, and one-sided is how real SDCs are usually written.
    fn launches_anything(&self, from: &str) -> bool {
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut stack = self.start_nets(from);
        while let Some(n) = stack.pop() {
            if !seen.insert(n.clone()) {
                continue;
            }
            if self.outputs.contains(&n) {
                return true;
            }
            for &i in self.sinks.get(&n).into_iter().flatten() {
                if self.is_seq[i] {
                    return true; // reached a register: a real endpoint
                }
                for (pin, nn) in &self.conns[i] {
                    if self.out_pins[i].contains(pin) {
                        stack.push(nn.clone());
                    }
                }
            }
        }
        false
    }

    /// Is `to` reached by any timing path — walking back to a register or a primary input?
    fn captures_anything(&self, to: &str) -> bool {
        let base = instance_of(to);
        // Start from the object's input nets (an instance) or the net itself (a port).
        let mut stack: Vec<String> = match self.inst_index.get(base) {
            Some(&i) => self.conns[i]
                .iter()
                .filter(|(p, _)| !self.out_pins[i].contains(p))
                .map(|(_, n)| n.clone())
                .collect(),
            None => vec![to.to_string()],
        };
        let mut seen: BTreeSet<String> = BTreeSet::new();
        while let Some(n) = stack.pop() {
            if !seen.insert(n.clone()) {
                continue;
            }
            match self.driver.get(&n) {
                // An undriven net is only a launch point if it is a primary INPUT. An output
                // port with no driver is dangling — nothing reaches it, which is exactly the
                // case worth reporting rather than passing.
                Some(None) => {
                    if self.inputs.contains(&n) {
                        return true;
                    }
                }
                Some(Some(i)) => {
                    if self.is_seq[*i] {
                        return true; // a register launches
                    }
                    for (pin, nn) in &self.conns[*i] {
                        if !self.out_pins[*i].contains(pin) {
                            stack.push(nn.clone());
                        }
                    }
                }
                None => {}
            }
        }
        false
    }

    /// Is there a structural path from `from` to `to`? Forward through combinational logic,
    /// stopping at sequential cells — which is what a timing path is.
    fn reaches(&self, from: &str, to: &str) -> bool {
        let target_base = instance_of(to);
        let target_inst = self.inst_index.get(target_base).copied();
        let mut seen: BTreeSet<String> = BTreeSet::new();
        let mut stack = self.start_nets(from);
        while let Some(n) = stack.pop() {
            if n == to || n == target_base {
                return true;
            }
            if !seen.insert(n.clone()) {
                continue;
            }
            for &i in self.sinks.get(&n).into_iter().flatten() {
                if Some(i) == target_inst {
                    return true;
                }
                if self.is_seq[i] {
                    continue; // a register ends the path
                }
                for (pin, nn) in &self.conns[i] {
                    if self.out_pins[i].contains(pin) {
                        stack.push(nn.clone());
                    }
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("examples/seq/seq.lib").expect("seq.lib")
    }
    fn nl() -> Netlist {
        // a registered path: in -> DFF -> out, plus the clock port
        crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\nwire q;\n\
             DFF r(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }

    #[test]
    fn clean_sdc_lints_clean() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert_eq!(r.errors(), 0, "{:?}", r.findings);
        assert_eq!(r.warnings(), 0, "{:?}", r.findings);
    }

    #[test]
    fn registers_without_a_clock_is_an_error() {
        let sdc = Sdc::parse("set_input_delay 1 [all_inputs]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r
            .findings
            .iter()
            .any(|f| f.code == "no-clock" && f.severity == Severity::Error));
    }

    #[test]
    fn zero_period_and_dup_name_are_errors() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 0 [get_ports clk]\n\
             create_clock -name clk -period 10 [get_ports clk]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "clock-period"));
        assert!(r.findings.iter().any(|f| f.code == "dup-clock-name"));
    }

    #[test]
    fn unconstrained_io_and_bad_port_warn() {
        // clock present, but no input/output delays at all, and a stray clock source
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "unconstrained-input")); // din
        assert!(r.findings.iter().any(|f| f.code == "unconstrained-output")); // dout
        assert_eq!(r.errors(), 0);
    }

    #[test]
    fn clock_on_missing_port_warns() {
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports nope]\n").unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(r.findings.iter().any(|f| f.code == "clock-source"));
    }
}

#[cfg(test)]
mod depth_tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("examples/seq/seq.lib").expect("seq.lib")
    }

    /// Two registers in series plus a combinational gate, so there is a real path to reason
    /// about: `din -> ra -> (gate) -> rb -> dout`.
    fn chain() -> Netlist {
        crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(mid));\n\
             DFF rb(.CK(clk),.D(mid),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }

    fn base_sdc() -> String {
        "create_clock -name clk -period 10 [get_ports clk]\n\
         set_input_delay 1 -clock clk [all_inputs]\n\
         set_output_delay 1 -clock clk [all_outputs]\n"
            .into()
    }

    fn has(r: &LintReport, code: &str) -> bool {
        r.findings.iter().any(|f| f.code == code)
    }

    #[test]
    fn an_exception_naming_nothing_is_reported() {
        // The failure this exists for: a typo'd exception does not error, it silently does
        // nothing, and the path the author meant stays timed.
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from ra -to no_such_register\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            has(&r, "exception-unknown-object"),
            "typo'd -to must be reported: {:?}",
            r.findings
        );
    }

    #[test]
    fn an_exception_on_a_real_path_is_quiet() {
        let sdc = Sdc::parse(&format!("{}set_false_path -from ra -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            !has(&r, "exception-unreachable") && !has(&r, "exception-unknown-object"),
            "ra -> rb is a real path and must not be flagged: {:?}",
            r.findings
        );
    }

    #[test]
    fn an_exception_between_two_real_objects_with_no_path_is_reported() {
        // Both ends exist, so the existence check passes — only reachability catches this.
        // Backwards: there is no path from rb to ra.
        let sdc = Sdc::parse(&format!("{}set_false_path -from rb -to ra\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "rb -> ra does not exist and the exception is dead: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_clock_that_reaches_nothing_leaves_registers_untimed() {
        // The SDC looks populated — a clock is defined — but it is on a port that drives no
        // register, so every register is untimed. A design-wide "registers but no clocks"
        // check cannot see this, which is the point of tracing.
        let n = crate::netlist::parse(
            "module t(clk,other,din,dout);\ninput clk,other,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(
            "create_clock -name c -period 10 [get_ports other]\n\
             set_input_delay 1 -clock c [all_inputs]\n\
             set_output_delay 1 -clock c [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "register-no-clock-reaches"),
            "a clock on the wrong port leaves ra untimed: {:?}",
            r.findings
        );
        assert!(
            !has(&r, "no-clock"),
            "a clock IS defined — the old check stays quiet"
        );
    }

    #[test]
    fn coverage_is_reported_and_is_complete_on_a_fully_constrained_design() {
        let sdc = Sdc::parse(&base_sdc()).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        let cov = r
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")
            .expect("coverage is always reported, not only when it is bad");
        assert_eq!(
            cov.severity,
            Severity::Info,
            "complete coverage is not a warning"
        );
        assert!(cov.message.contains("100.0%"), "{}", cov.message);
    }

    #[test]
    fn coverage_falls_when_an_output_is_unconstrained() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n",
        )
        .unwrap();
        let r = lint(&chain(), &sdc, &lib());
        let cov = r
            .findings
            .iter()
            .find(|f| f.code == "endpoint-coverage")
            .unwrap();
        assert_eq!(cov.severity, Severity::Warning);
        assert!(!cov.message.contains("100.0%"), "{}", cov.message);
    }

    #[test]
    fn a_one_sided_exception_on_a_dead_port_is_reported() {
        // The shape real SDCs actually write: `-from [list ...]` with no `-to`. An earlier
        // version only checked when BOTH sides were named, so this — the common case — was
        // silently never checked at all.
        let n = crate::netlist::parse(
            "module t(clk,din,unused,dout);\ninput clk,din,unused; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from [get_ports {{unused}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "`unused` drives nothing, so cutting it cuts nothing: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_one_sided_exception_on_a_live_port_is_quiet() {
        let n = crate::netlist::parse(
            "module t(clk,din,dout);\ninput clk,din; output dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -from [get_ports {{din}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            !has(&r, "exception-unreachable"),
            "`din` reaches a register — a real cut, not a dead one: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_one_sided_to_exception_is_checked_from_the_other_direction() {
        let n = crate::netlist::parse(
            "module t(clk,din,dangling,dout);\ninput clk,din; output dangling,dout;\n\
             DFF ra(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(&format!(
            "{}set_false_path -to [get_ports {{dangling}}]\n",
            base_sdc()
        ))
        .unwrap();
        let r = lint(&n, &sdc, &lib());
        assert!(
            has(&r, "exception-unreachable"),
            "`dangling` is driven by nothing, so nothing is cut: {:?}",
            r.findings
        );
    }

    #[test]
    fn a_wildcard_exception_is_not_checkable_and_is_not_flagged() {
        // `-from *` means "any"; reporting it as unknown would be noise on a legitimate SDC.
        let sdc = Sdc::parse(&format!("{}set_false_path -from * -to rb\n", base_sdc())).unwrap();
        let r = lint(&chain(), &sdc, &lib());
        assert!(
            !has(&r, "exception-unknown-object") && !has(&r, "exception-unreachable"),
            "a wildcard is not a typo: {:?}",
            r.findings
        );
    }
}

#[cfg(test)]
mod accessor_tests {
    use super::*;

    #[test]
    fn the_coverage_accessor_reads_back_what_the_finding_says() {
        // The accessor derives from the finding rather than storing a second copy, so this
        // guards the one way they could drift: a change to the message wording.
        let r = LintReport {
            findings: vec![Finding {
                severity: Severity::Warning,
                code: "endpoint-coverage",
                message: "endpoint coverage 66.7% — 2 of 3 constrained (1/2 registers \
                          clocked, 1/1 outputs with a delay)"
                    .into(),
            }],
        };
        assert_eq!(r.endpoint_coverage_pct(), Some(66.7));
    }

    #[test]
    fn no_coverage_finding_means_no_number_rather_than_a_wrong_one() {
        let r = LintReport {
            findings: vec![Finding::warn("something-else", "unrelated".into())],
        };
        assert_eq!(r.endpoint_coverage_pct(), None);
    }
}

#[cfg(test)]
mod noise_tests {
    use super::*;

    fn lib() -> Lib {
        Lib::load("examples/seq/seq.lib").expect("seq.lib")
    }

    /// A wide top-level wrapper: many outputs, none constrained.
    fn wide(n: usize) -> Netlist {
        let ports: Vec<String> = (0..n).map(|i| format!("o{i}")).collect();
        let decl = ports.join(",");
        crate::netlist::parse(&format!(
            "module t(clk,din,{decl});\ninput clk,din; output {decl};\n\
             DFF r(.CK(clk),.D(din),.Q(o0));\nendmodule\n"
        ))
        .unwrap()
    }

    #[test]
    fn hundreds_of_unconstrained_ports_collapse_to_a_count() {
        // Found on a real pad wrapper: 484 unconstrained outputs produced 484 warnings, and
        // the two findings that mattered were somewhere in the middle of them. A reader
        // scrolls past that, which is how a checker stops being run.
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&wide(200), &sdc, &lib());
        let outs: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.code == "unconstrained-output")
            .collect();
        assert!(
            outs.len() <= PORT_LIST_CAP + 1,
            "listed {} findings for 200 ports — the cap is not applied",
            outs.len()
        );
        let summary = outs.last().expect("a summary line");
        assert!(
            summary.message.contains("200 in total"),
            "the count must survive the cap: {}",
            summary.message
        );
    }

    #[test]
    fn a_small_design_still_names_every_port() {
        // The cap must not cost detail where detail is usable.
        let sdc = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\n").unwrap();
        let r = lint(&wide(3), &sdc, &lib());
        let outs: Vec<_> = r
            .findings
            .iter()
            .filter(|f| f.code == "unconstrained-output")
            .collect();
        assert_eq!(outs.len(), 3, "three ports, three named findings");
        assert!(outs.iter().all(|f| !f.message.contains("in total")));
    }
}

#[cfg(test)]
mod glob_tests {
    use super::*;

    #[test]
    fn bus_wildcards_match_their_members() {
        // `[get_ports {paddr_i[*]}]` is how a real SDC constrains a whole bus. Comparing it
        // as a literal string reported every such line as targeting a nonexistent port.
        assert!(glob_match("paddr_i[*]", "paddr_i[0]"));
        assert!(glob_match("paddr_i[*]", "paddr_i[15]"));
        assert!(!glob_match("paddr_i[*]", "pwdata_i[0]"));
        assert!(glob_match("axi_*_i", "axi_awvalid_i"));
        assert!(glob_match("?clk_i", "pclk_i"));
        assert!(!glob_match("?clk_i", "axi_aclk_i"));
    }

    #[test]
    fn a_pattern_with_no_metacharacter_is_an_exact_match() {
        assert!(glob_match("clk_i", "clk_i"));
        assert!(!glob_match("clk_i", "clk_ii"));
        assert!(!glob_match("clk_i", "xclk_i"));
    }

    #[test]
    fn backtracking_handles_more_than_one_star() {
        assert!(glob_match("a*b*c", "axxbyyc"));
        assert!(!glob_match("a*b*c", "axxbyy"));
        assert!(glob_match("*", "anything"));
    }

    #[test]
    fn a_bus_wide_input_delay_no_longer_reports_a_missing_port() {
        let n = crate::netlist::parse(
            "module t(clk,paddr_i,dout);\ninput clk; input [1:0] paddr_i; output dout;\n\
             DFF r(.CK(clk),.D(paddr_i[0]),.Q(dout));\nendmodule\n",
        )
        .unwrap();
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [get_ports {paddr_i[*]}]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&n, &sdc, &Lib::load("examples/seq/seq.lib").unwrap());
        assert!(
            !r.findings.iter().any(|f| f.code == "delay-unknown-port"),
            "a bus wildcard names real ports: {:?}",
            r.findings
        );
        assert!(
            !r.findings.iter().any(|f| f.code == "unconstrained-input"),
            "and it constrains them: {:?}",
            r.findings
        );
    }
}

#[cfg(test)]
mod declared_domain_tests {
    use super::*;
    use crate::ipmeta::IpMeta;

    fn lib() -> Lib {
        Lib::load("examples/seq/seq.lib").expect("seq.lib")
    }
    fn nl() -> Netlist {
        crate::netlist::parse(
            "module t(clk,pclk,din,dout);\ninput clk,pclk,din; output dout;\n\
             DFF r(.CK(clk),.D(din),.Q(dout));\nendmodule\n",
        )
        .unwrap()
    }
    const TWO_DOMAINS: &str = r#"{"clock_domains":[
        {"name":"core","clock":"clk"},{"name":"apb","clock":"pclk"}],
        "clock_domain_relations":{"asynchronous_groups":[["core"],["apb"]]}}"#;

    #[test]
    fn a_declared_domain_the_sdc_never_constrains_is_an_error() {
        // Exactly the fft_top defect: the IP has two clock domains, the flow config named one,
        // and timing passes while half the design is untimed. Only the declaration catches it.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        let f = r
            .findings
            .iter()
            .find(|f| f.code == "domain-not-constrained")
            .expect("the undeclared domain must be reported");
        assert_eq!(f.severity, Severity::Error);
        assert!(
            f.message.contains("apb") && f.message.contains("pclk"),
            "{}",
            f.message
        );
    }

    #[test]
    fn constraining_every_declared_domain_is_quiet() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             create_clock -name pclk -period 10 [get_ports pclk]\n\
             set_clock_groups -asynchronous -group {clk} -group {pclk}\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        assert!(
            !r.findings.iter().any(
                |f| f.code == "domain-not-constrained" || f.code == "async-groups-not-declared"
            ),
            "a correct SDC must stay quiet: {:?}",
            r.findings
        );
    }

    #[test]
    fn declared_async_domains_with_no_clock_groups_warn_but_do_not_error() {
        // Both domains constrained, but the SDC does not say they are unrelated — the paths
        // between them get timed against an edge relation that does not exist. Pessimistic,
        // not absent, so it is a warning.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             create_clock -name pclk -period 10 [get_ports pclk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let m = IpMeta::parse(TWO_DOMAINS).unwrap();
        let r = lint_with_meta(&nl(), &sdc, &lib(), Some(&m));
        let f = r
            .findings
            .iter()
            .find(|f| f.code == "async-groups-not-declared")
            .expect("reported");
        assert_eq!(f.severity, Severity::Warning);
        assert!(!r
            .findings
            .iter()
            .any(|f| f.code == "domain-not-constrained"));
    }

    #[test]
    fn without_metadata_the_check_does_not_run_at_all() {
        // Most IPs carry no clock_domains yet. Absence of a declaration is not a finding.
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [all_inputs]\n\
             set_output_delay 1 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let r = lint(&nl(), &sdc, &lib());
        assert!(!r
            .findings
            .iter()
            .any(|f| f.code == "domain-not-constrained"));
    }
}
