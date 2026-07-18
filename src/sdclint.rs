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
//! same name), so a clean lint means something. Clock-tree tracing and exception
//! reachability are the depth passes (and partly the job of `vyges-cdc`).

use std::collections::{BTreeMap, BTreeSet};

use crate::liberty::Lib;
use crate::netlist::Netlist;
use crate::sdc::Sdc;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warning,
}

impl Severity {
    pub fn tag(self) -> &'static str {
        match self {
            Severity::Error => "error",
            Severity::Warning => "warning",
        }
    }
}

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

    for p in &nl.inputs {
        let p = p.as_str();
        if clock_ports.contains(p) {
            continue; // a clock input, not a data input
        }
        if !in_default && !in_ports.contains(p) {
            f.push(Finding::warn(
                "unconstrained-input",
                format!("input `{p}` has no set_input_delay"),
            ));
        }
    }
    for p in &nl.outputs {
        let p = p.as_str();
        if !out_default && !out_ports.contains(p) {
            f.push(Finding::warn(
                "unconstrained-output",
                format!("output `{p}` has no set_output_delay"),
            ));
        }
    }

    // an explicit delay targeting a port the design doesn't have
    for p in in_ports.iter().filter(|p| !inputs.contains(**p)) {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_input_delay targets `{p}`, not an input of the design"),
        ));
    }
    for p in out_ports.iter().filter(|p| !outputs.contains(**p)) {
        f.push(Finding::warn(
            "delay-unknown-port",
            format!("set_output_delay targets `{p}`, not an output of the design"),
        ));
    }

    f.sort_by(|a, b| (a.severity as u8, a.code).cmp(&(b.severity as u8, b.code)));
    LintReport { findings: f }
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
