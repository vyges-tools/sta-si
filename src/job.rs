//! STA job: the declarative description of what to analyze.
//!
//! A `.sta` job is a tiny `key: value` file (std-only parser — no deps):
//!
//! ```text
//! design:      top
//! netlist:     top.v               # gate-level structural Verilog
//! lib:         sky130_hd.lib        # one or more (comma-separated)
//! clock:       clk 5.0              # clock port + period (ns)
//! input_slew:  0.05                 # ns, default input transition
//! output_load: 0.005                # pF, load at primary outputs
//! late_derate: 1.05                 # OCV late derate on cell delays (default 1.0)
//! early_derate: 0.95                # OCV early derate (hold/min path; default 1.0)
//! # advanced OCV (optional) — choose ONE refinement over the flat derates above:
//! aocv_late:  1:1.10, 4:1.05, 8:1.02   # depth-dependent late derate (interpolated)
//! aocv_early: 1:0.90, 4:0.95, 8:0.98   # depth-dependent early derate
//! pocv_sigma: 0.05                  # POCV: per-stage 1-sigma as a fraction of delay
//! pocv_n:     3.0                   # POCV: number of sigmas (default 3.0)
//! ```
//!
//! **Multiple clocks** (and generated/divided clocks) are listed with one `clock:`
//! line each — `clock: <name> <source> <period>` (source is a port or `inst/pin`,
//! e.g. a divider output). Cross-domain paths get the tightest launch→capture edge
//! relation; same-domain paths use that domain's period.
//!
//! ```text
//! clock: clk    clk      10.0
//! clock: spiclk spi_clk   4.0
//! clock: divclk u_div/Q  20.0     # generated: divide-by-2 off an internal pin
//! ```
//!
//! An **MCMM** job instead lists per-corner scenario files (each a full `.sta`);
//! the engine runs all and reports the worst setup/hold across them:
//!
//! ```text
//! design:    top
//! scenarios: corner_ss.sta, corner_tt.sta, corner_ff.sta
//! ```

use std::collections::BTreeMap;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct StaJob {
    pub design: String,
    pub netlist: String,
    pub libs: Vec<String>,
    pub spef: Option<String>, // optional parasitics -> wire load + net delay
    pub clock_port: String,   // primary clock (first); back-compat + report header
    pub period_ns: f64,
    // all clocks: (name, source port|inst/pin, period_ns). Empty -> single clock
    // synthesized from clock_port/period_ns. A generated/divided clock is just an
    // entry whose source is an internal pin and period the divided value.
    pub clocks: Vec<(String, String, f64)>,
    pub input_slew: f64,
    pub output_load: f64,
    // I/O timing budget (from SDC set_input_delay / set_output_delay). `input_delay`
    // is the default arrival at primary inputs; `output_delay` the external delay
    // that eats into the period at primary outputs. Per-port entries override.
    pub input_delay: f64,
    pub output_delay: f64,
    pub io_input_delays: Vec<(String, f64)>,
    pub io_output_delays: Vec<(String, f64)>,
    // clock uncertainty (SDC set_clock_uncertainty): tightens setup required time,
    // relaxes hold required time.
    pub setup_uncertainty: f64,
    pub hold_uncertainty: f64,
    pub late_derate: f64,
    pub early_derate: f64, // OCV early derate on cell delays for the hold (min) path
    // OCV mode (resolved at analysis time): POCV if pocv_sigma > 0, else AOCV if an
    // aocv table is present, else flat (the late/early scalar derates above).
    pub pocv_sigma: f64, // POCV 1-sigma as a fraction of each stage's nominal delay
    pub pocv_n: f64,     // number of sigmas for the statistical bound (default 3.0)
    pub aocv_late: Vec<(f64, f64)>, // AOCV late derate vs path depth: (stages, derate)
    pub aocv_early: Vec<(f64, f64)>, // AOCV early derate vs path depth
    pub miller: f64, // crosstalk Miller coupling factor (2.0 worst late; 1.0 disables SI)
    pub xtalk_window: f64, // ns — guard band added to the slew-derived switching window
    pub crpr: bool, // remove clock-reconvergence pessimism on the shared clock path (default true)
    pub pba: bool,  // path-based analysis: re-time critical paths with path-local slews (default false)
    // MCMM: when non-empty, this is a multi-corner/multi-mode job — each entry is a
    // path to a single-scenario `.sta`; the engine runs all and reports the worst
    // setup and worst hold across them. The fields above are then unused.
    pub scenarios: Vec<String>,
    pub exceptions: Vec<Exception>, // false-path / multicycle timing exceptions
    pub sdc: Option<String>, // optional SDC constraints file (merged at load)
    pub base_dir: String,
}

impl StaJob {
    /// Resolve the input arrival for a primary input port (per-port override,
    /// else the default `input_delay`).
    pub fn input_delay_for(&self, port: &str) -> f64 {
        self.io_input_delays
            .iter()
            .find(|(p, _)| p == port)
            .map(|(_, d)| *d)
            .unwrap_or(self.input_delay)
    }

    /// Resolve the external output delay for a primary output port.
    pub fn output_delay_for(&self, port: &str) -> f64 {
        self.io_output_delays
            .iter()
            .find(|(p, _)| p == port)
            .map(|(_, d)| *d)
            .unwrap_or(self.output_delay)
    }
}

/// Timing exception types now live in the shared `vyges-loom` SDC model; re-export
/// them so `crate::job::{Exception, ExcKind}` keeps resolving across the engine.
/// FalsePath drops the path from both setup and hold; Multicycle(n) moves the
/// setup capture n cycles out (hold by n−1).
pub use crate::sdc::{ExcKind, Exception};

#[derive(Debug)]
pub struct JobError(pub String);
impl std::fmt::Display for JobError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "job error: {}", self.0)
    }
}
impl std::error::Error for JobError {}

fn strip_comment(line: &str) -> &str {
    match line.find('#') {
        Some(i) => &line[..i],
        None => line,
    }
}

/// Parse `depth:derate` pairs, e.g. `1:1.10, 2:1.07, 4:1.04` -> [(1,1.10),...].
fn pairs(s: &str) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = s
        .split(',')
        .filter_map(|p| {
            let (a, b) = p.split_once(':')?;
            Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
        })
        .collect();
    v.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    v
}

impl StaJob {
    pub fn parse(text: &str, base_dir: &str) -> Result<StaJob, JobError> {
        let mut kv: BTreeMap<String, String> = BTreeMap::new();
        // clock lines are collected separately — a BTreeMap would dedupe multiple.
        let mut clocks: Vec<(String, String, f64)> = Vec::new();
        let mut exceptions: Vec<Exception> = Vec::new();
        for raw in text.lines() {
            let line = strip_comment(raw).trim();
            if line.is_empty() {
                continue;
            }
            let (k, v) = line
                .split_once(':')
                .ok_or_else(|| JobError(format!("expected 'key: value', got {line:?}")))?;
            let key = k.trim().to_lowercase();
            if key == "clock" {
                // `clock: <port> <period>`  or  `clock: <name> <source> <period>`
                let toks: Vec<&str> = v.split_whitespace().collect();
                let (name, src, per) = match toks.as_slice() {
                    [src, per] => (src.to_string(), src.to_string(), per),
                    [name, src, per] => (name.to_string(), src.to_string(), per),
                    _ => return Err(JobError("clock needs 'port period' or 'name source period'".into())),
                };
                let period: f64 =
                    per.parse().map_err(|_| JobError(format!("bad clock period: {per:?}")))?;
                clocks.push((name, src, period));
                continue;
            }
            if key == "false_path" || key == "multicycle" {
                // `false_path: <from> <to>`   `multicycle: <from> <to> <cycles>`
                let t: Vec<&str> = v.split_whitespace().collect();
                let exc = match (key.as_str(), t.as_slice()) {
                    ("false_path", [from, to]) => {
                        Exception { kind: ExcKind::FalsePath, from: from.to_string(), to: to.to_string() }
                    }
                    ("multicycle", [from, to, n]) => {
                        let cyc: u32 =
                            n.parse().map_err(|_| JobError(format!("bad multicycle count: {n:?}")))?;
                        Exception {
                            kind: ExcKind::Multicycle(cyc),
                            from: from.to_string(),
                            to: to.to_string(),
                        }
                    }
                    _ => return Err(JobError(format!("bad exception: {line:?}"))),
                };
                exceptions.push(exc);
                continue;
            }
            kv.insert(key, v.trim().to_string());
        }
        let get = |k: &str| kv.get(k).cloned().ok_or_else(|| JobError(format!("missing key: {k}")));
        let split_list = |s: &str| {
            s.split(',').map(|t| t.trim().to_string()).filter(|t| !t.is_empty()).collect::<Vec<_>>()
        };
        let scenarios = kv.get("scenarios").map(|s| split_list(s)).unwrap_or_default();
        let mcmm = !scenarios.is_empty();
        let sdc = kv.get("sdc").filter(|s| !s.is_empty()).cloned();
        // clock/netlist/lib are required for a single run, optional for an MCMM job
        // or when an SDC supplies the clock (merged in `load`).
        let (clock_port, period_ns) = match clocks.first() {
            Some((_, src, per)) => (src.clone(), *per),
            None if mcmm || sdc.is_some() => (String::new(), 0.0),
            None => return Err(JobError("missing key: clock".into())),
        };
        let num = |k: &str, d: f64| kv.get(k).and_then(|s| s.parse().ok()).unwrap_or(d);
        let job = StaJob {
            design: get("design")?,
            netlist: kv.get("netlist").cloned().unwrap_or_default(),
            libs: kv.get("lib").map(|s| split_list(s)).unwrap_or_default(),
            spef: kv.get("spef").filter(|s| !s.is_empty()).cloned(),
            clock_port,
            period_ns,
            clocks,
            input_slew: num("input_slew", 0.05),
            output_load: num("output_load", 0.005),
            input_delay: num("input_delay", 0.0),
            output_delay: num("output_delay", 0.0),
            io_input_delays: Vec::new(),
            io_output_delays: Vec::new(),
            setup_uncertainty: num("setup_uncertainty", 0.0),
            hold_uncertainty: num("hold_uncertainty", 0.0),
            late_derate: num("late_derate", 1.0),
            early_derate: num("early_derate", 1.0),
            pocv_sigma: num("pocv_sigma", 0.0),
            pocv_n: num("pocv_n", 3.0),
            aocv_late: kv.get("aocv_late").map(|s| pairs(s)).unwrap_or_default(),
            aocv_early: kv.get("aocv_early").map(|s| pairs(s)).unwrap_or_default(),
            miller: num("miller", 2.0),
            xtalk_window: num("xtalk_window", 0.0), // guard band on top of slew-derived window
            crpr: kv.get("crpr").map(|s| s != "false" && s != "0").unwrap_or(true),
            pba: kv.get("pba").map(|s| s == "true" || s == "1").unwrap_or(false),
            scenarios,
            exceptions,
            sdc,
            base_dir: base_dir.to_string(),
        };
        // When an SDC supplies the clock, defer the final validation until after
        // it is merged in `load`; a bare clockless job (no SDC) still fails here.
        if job.sdc.is_none() {
            job.validate()?;
        }
        Ok(job)
    }

    pub fn load(path: &str) -> Result<StaJob, JobError> {
        let text = std::fs::read_to_string(path).map_err(|e| JobError(format!("{path}: {e}")))?;
        let base = Path::new(path).parent().and_then(|p| p.to_str()).unwrap_or(".");
        let mut job = StaJob::parse(&text, base)?;
        if let Some(sdc_path) = job.sdc.clone() {
            let resolved = job.resolve(&sdc_path);
            let sdc = crate::sdc::Sdc::load(&resolved).map_err(|e| JobError(e.to_string()))?;
            merge_sdc_into(&sdc, &mut job);
            job.validate()?; // now that SDC has supplied the clock
        }
        Ok(job)
    }

    pub fn resolve(&self, rel: &str) -> String {
        if Path::new(rel).is_absolute() || self.base_dir.is_empty() {
            rel.to_string()
        } else {
            Path::new(&self.base_dir).join(rel).to_string_lossy().into_owned()
        }
    }

    /// True when this job orchestrates multiple corner/mode scenarios.
    pub fn is_mcmm(&self) -> bool {
        !self.scenarios.is_empty()
    }

    fn validate(&self) -> Result<(), JobError> {
        if self.is_mcmm() {
            return Ok(()); // each scenario file is validated when it's loaded
        }
        if self.netlist.is_empty() || self.libs.is_empty() {
            return Err(JobError("netlist and at least one lib are required".into()));
        }
        if self.clock_port.is_empty() || self.period_ns <= 0.0 {
            return Err(JobError("clock port + positive period required".into()));
        }
        Ok(())
    }
}

/// Merge loom's parsed SDC constraints onto a [`StaJob`] — the STA-specific
/// half of SDC handling (loom owns the *parser*; this owns the *mapping to the
/// timing job*). Relocated here when the SDC reader moved to `vyges-loom`.
///
/// The job keeps its design / netlist / lib / spef; SDC supplies the timing
/// intent. Explicit `.sta` values are kept where SDC is silent.
pub fn merge_sdc_into(sdc: &crate::sdc::Sdc, job: &mut StaJob) {
    // clocks: SDC is authoritative when present.
    if !sdc.clocks.is_empty() {
        job.clocks =
            sdc.clocks.iter().map(|c| (c.name.clone(), c.source.clone(), c.period)).collect();
        job.clock_port = job.clocks[0].1.clone();
        job.period_ns = job.clocks[0].2;
    }
    // I/O timing: default + per-port. Source latency adds to the I/O budget
    // (an input arrives `latency` later; an output must settle `latency` earlier),
    // matching propagated-clock intent on the boundary.
    let mut in_def = None;
    for d in &sdc.input_delays {
        if d.default {
            in_def = Some(d.value);
        }
        for p in &d.ports {
            job.io_input_delays.push((p.clone(), d.value + sdc.clock_latency));
        }
    }
    if let Some(v) = in_def {
        job.input_delay = v + sdc.clock_latency;
    }
    let mut out_def = None;
    for d in &sdc.output_delays {
        if d.default {
            out_def = Some(d.value);
        }
        for p in &d.ports {
            job.io_output_delays.push((p.clone(), d.value + sdc.clock_latency));
        }
    }
    if let Some(v) = out_def {
        job.output_delay = v + sdc.clock_latency;
    }
    job.setup_uncertainty = sdc.setup_uncertainty;
    job.hold_uncertainty = sdc.hold_uncertainty;
    if let Some(v) = sdc.input_transition {
        job.input_slew = v;
    }
    if let Some(v) = sdc.load {
        job.output_load = v;
    }
    if let Some(v) = sdc.late_derate {
        job.late_derate = v;
    }
    if let Some(v) = sdc.early_derate {
        job.early_derate = v;
    }
    job.exceptions.extend(sdc.exceptions.iter().cloned());
}
