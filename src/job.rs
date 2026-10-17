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
    /// Whether an arrival was actually **declared** for ports the SDC does not name
    /// individually — by a job `input_delay:` key or an SDC default `set_input_delay`.
    ///
    /// A port with no declared arrival is **unconstrained**: it has no timing start point,
    /// so no path may begin there. Seeding it at 0 instead invents a launch at time zero,
    /// which is invisible on setup (a whole period of slack) and manufactures hold
    /// violations against a capture clock that has real insertion delay — measured at
    /// -1.07 ns against OpenSTA's +0.88 on a block whose SDC constrained no inputs.
    pub input_delay_declared: bool,
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
    pub miller: f64,     // crosstalk Miller coupling factor (2.0 worst late; 1.0 disables SI)
    /// Interconnect delay model. `"elmore"` (default) is the transcription of OpenSTA's own
    /// default calculator, `dmp_ceff_elmore` — per-sink Elmore converted to a delay and a
    /// degraded slew at the library thresholds. `"transient"` selects our waveform-into-RC
    /// model instead; it is an ADDITION and has to be asked for, because for two years it
    /// silently took precedence over the reference model and nothing said so.
    pub rc_model: String,
    pub xtalk_window: f64, // ns — guard band added to the slew-derived switching window
    pub crpr: bool, // remove clock-reconvergence pessimism on the shared clock path (default true)
    pub pba: bool, // path-based analysis: re-time critical paths with path-local slews (default false)
    // MCMM: when non-empty, this is a multi-corner/multi-mode job — each entry is a
    // path to a single-scenario `.sta`; the engine runs all and reports the worst
    // setup and worst hold across them. The fields above are then unused.
    pub scenarios: Vec<String>,
    pub exceptions: Vec<Exception>, // false-path / multicycle timing exceptions
    /// `set_clock_groups -asynchronous` groups (clock names). Launch<->capture
    /// paths crossing two different groups are cut (setup and hold).
    pub async_groups: Vec<Vec<String>>,
    /// The SDC's `set_clock_groups`, per command — when present, the clock separation rule
    /// (`crate::sdc::clocks_unrelated`) is read from these, not from `async_groups`.
    pub clock_groups: Vec<crate::sdc::ClockGroups>,
    pub sdc: Option<String>, // optional SDC constraints file (merged at load)
    /// Optional `vyges-metadata.json`. Only the constraint linter reads it, for the IP's
    /// declared `clock_domains` — the one input that says how many clocks the design was
    /// *supposed* to have, which neither the netlist nor the SDC can supply.
    pub metadata: Option<String>,
    pub base_dir: String,
}

impl StaJob {
    /// Resolve the input arrival for a primary input port (per-port override,
    /// else the default `input_delay`).
    /// `None` means the port is **unconstrained** and must not start a timed path.
    pub fn input_delay_for(&self, port: &str) -> Option<f64> {
        // the object as written may be a PATTERN (`[get_ports in*]`), and a later command on the
        // same port replaces an earlier one — the reference's rules
        self.io_input_delays
            .iter()
            .rev()
            .find(|(p, _)| vyges_loom::sdc::object_matches(p, port))
            .map(|(_, d)| *d)
            .or(if self.input_delay_declared {
                Some(self.input_delay)
            } else {
                None
            })
    }

    /// Resolve the external output delay for a primary output port.
    pub fn output_delay_for(&self, port: &str) -> f64 {
        // the object as written may be a PATTERN (`[get_ports in*]`), and a later command on the
        // same port replaces an earlier one — the reference's rules
        self.io_output_delays
            .iter()
            .rev()
            .find(|(p, _)| vyges_loom::sdc::object_matches(p, port))
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
                    _ => {
                        return Err(JobError(
                            "clock needs 'port period' or 'name source period'".into(),
                        ))
                    }
                };
                let period: f64 = per
                    .parse()
                    .map_err(|_| JobError(format!("bad clock period: {per:?}")))?;
                clocks.push((name, src, period));
                continue;
            }
            if key == "false_path" || key == "multicycle" {
                // `false_path: <from> <to>`   `multicycle: <from> <to> <cycles>`
                let t: Vec<&str> = v.split_whitespace().collect();
                let exc = match (key.as_str(), t.as_slice()) {
                    // The job-file form names one object per side; the SDC form can name many
                    // (`-from [list ...]`), which is why the model holds sets.
                    ("false_path", [from, to]) => Exception {
                        kind: ExcKind::FalsePath,
                        from: vec![from.to_string()],
                        to: vec![to.to_string()],
                        ..Default::default()
                    },
                    ("multicycle", [from, to, n]) => {
                        let cyc: u32 = n
                            .parse()
                            .map_err(|_| JobError(format!("bad multicycle count: {n:?}")))?;
                        Exception {
                            kind: ExcKind::Multicycle(cyc),
                            from: vec![from.to_string()],
                            to: vec![to.to_string()],
                            ..Default::default()
                        }
                    }
                    _ => return Err(JobError(format!("bad exception: {line:?}"))),
                };
                exceptions.push(exc);
                continue;
            }
            kv.insert(key, v.trim().to_string());
        }
        let get = |k: &str| {
            kv.get(k)
                .cloned()
                .ok_or_else(|| JobError(format!("missing key: {k}")))
        };
        let split_list = |s: &str| {
            s.split(',')
                .map(|t| t.trim().to_string())
                .filter(|t| !t.is_empty())
                .collect::<Vec<_>>()
        };
        let scenarios = kv
            .get("scenarios")
            .map(|s| split_list(s))
            .unwrap_or_default();
        let mcmm = !scenarios.is_empty();
        let sdc = kv.get("sdc").filter(|s| !s.is_empty()).cloned();
        let metadata = kv.get("metadata").filter(|s| !s.is_empty()).cloned();
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
            input_delay_declared: kv.contains_key("input_delay"),
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
            rc_model: kv
                .get("rc_model")
                .filter(|s| !s.is_empty())
                .cloned()
                .unwrap_or_else(|| "elmore".into()),
            xtalk_window: num("xtalk_window", 0.0), // guard band on top of slew-derived window
            crpr: kv
                .get("crpr")
                .map(|s| s != "false" && s != "0")
                .unwrap_or(true),
            pba: kv
                .get("pba")
                .map(|s| s == "true" || s == "1")
                .unwrap_or(false),
            scenarios,
            exceptions,
            async_groups: Vec::new(),
            clock_groups: Vec::new(),
            sdc,
            metadata,
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
        let base = Path::new(path)
            .parent()
            .and_then(|p| p.to_str())
            .unwrap_or(".");
        let mut job = StaJob::parse(&text, base)?;
        if let Some(sdc_path) = job.sdc.clone() {
            let sdc = job.load_sdc(&sdc_path).map_err(JobError)?;
            merge_sdc_into(&sdc, &mut job);
            job.validate()?; // now that SDC has supplied the clock
        }
        Ok(job)
    }

    /// The job's constraints, read as the reference reads them after linking: a file whose
    /// object queries (`-filter`, `-of_objects`) need the design is read again against the
    /// job's netlist and libraries. Read plainly otherwise, so a file without them costs nothing.
    pub fn load_sdc(&self, sdc_path: &str) -> Result<crate::sdc::Sdc, String> {
        let resolved = self.resolve(sdc_path);
        let plain = crate::sdc::Sdc::load(&resolved).map_err(|e| e.to_string())?;
        if plain.unevaluated_queries.is_empty() || self.netlist.is_empty() {
            return Ok(plain);
        }
        let nl = crate::engine::load_netlist(&self.resolve(&self.netlist)).map_err(|e| e.to_string())?;
        let mut lib = crate::liberty::Lib::default();
        for l in &self.libs {
            let one = crate::liberty::Lib::load(&self.resolve(l)).map_err(|e| e.to_string())?;
            lib.cells.extend(one.cells);
        }
        let design = crate::sdc::SdcDesign::from_netlist(&nl, Some(&lib));
        crate::sdc::Sdc::load_with(&resolved, &design).map_err(|e| e.to_string())
    }

    pub fn resolve(&self, rel: &str) -> String {
        if Path::new(rel).is_absolute() || self.base_dir.is_empty() {
            rel.to_string()
        } else {
            Path::new(&self.base_dir)
                .join(rel)
                .to_string_lossy()
                .into_owned()
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
    // asynchronous clock groups: cut cross-group paths (setup + hold).
    if !sdc.async_groups.is_empty() {
        job.async_groups = sdc.async_groups.clone();
    }
    job.clock_groups = sdc.clock_groups.clone();
    // clocks: SDC is authoritative when present.
    if !sdc.clocks.is_empty() {
        job.clocks = sdc
            .clocks
            .iter()
            .map(|c| (c.name.clone(), c.source.clone(), c.period))
            .collect();
        // A VIRTUAL CLOCK HAS NO SOURCE, and that is a legal SDC construct rather than an
        // omission: `create_clock -name clk -period 1.0` with no object list defines a reference
        // clock for I/O budgeting that launches nothing inside this design. It carries the
        // period and the I/O delays, which is exactly what a block constrained only at its
        // boundary needs, so it must not be rejected for want of a port.
        //
        // Its NAME stands in as the clock's identifier. That is also what the reader used to
        // hand over — it defaulted a sourceless clock's source to its own name — which made a
        // virtual clock indistinguishable from a real one on a port of that name, and would
        // silently launch through such a port if the design happened to have one.
        let c = &job.clocks[0];
        job.clock_port = if c.1.is_empty() { c.0.clone() } else { c.1.clone() };
        job.period_ns = c.2;
    }
    // I/O timing: default + per-port.
    //
    // ⛔ Clock latency is NOT added to the I/O delays. It was — the largest `set_clock_latency`
    // value onto every input and output delay — and the reference does no such thing: a clock's
    // latency (source or network, ideal or propagated clock) is on BOTH the launch and the capture
    // side of a path, so on a path launched and captured by one clock it cancels. Measured on a
    // two-flop design with an input and an output delay: every slack in the reference is the same
    // with 0, 0.5 network, 0.4 source and both; folding it in moved the worst slack by exactly
    // the latency. Where latency does not cancel (clocks with different latencies, latency on a
    // pin, a delay that says it includes latency) see `latency_not_modelled`.
    let mut in_def = None;
    for d in &sdc.input_delays {
        if d.default {
            in_def = Some(d.value);
        }
        for p in &d.ports {
            job.io_input_delays
                .push((p.clone(), d.value));
        }
    }
    if let Some(v) = in_def {
        job.input_delay = v;
        job.input_delay_declared = true;
    }
    let mut out_def = None;
    for d in &sdc.output_delays {
        if d.default {
            out_def = Some(d.value);
        }
        for p in &d.ports {
            job.io_output_delays
                .push((p.clone(), d.value));
        }
    }
    if let Some(v) = out_def {
        job.output_delay = v;
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

/// The clock latency a constraint file states that this timer does not model, in words — `None`
/// when every latency cancels (one latency per clock, the same for every clock, on clocks only).
/// A latency on a clock is on both sides of a path that clock launches and captures; it moves a
/// slack only between clocks of different latency, when it is on a pin or port rather than a
/// clock, or when an I/O delay declares it already includes the latency.
pub fn latency_not_modelled(sdc: &crate::sdc::Sdc) -> Option<String> {
    if sdc.clock_latencies.is_empty() {
        return None;
    }
    let mut why = Vec::new();
    if sdc.clock_latencies.iter().any(|l| l.accessor != "get_clocks") {
        why.push("set on a pin or port".to_string());
    }
    // per clock and kind (source / network), the values stated
    let mut per: std::collections::BTreeMap<(bool, String), Vec<f64>> = Default::default();
    for l in sdc.clock_latencies.iter().filter(|l| l.accessor == "get_clocks") {
        let source = l.flags.iter().any(|f| f == "-source");
        for c in &l.objects {
            per.entry((source, c.clone())).or_default().push(l.value);
        }
    }
    let clocks: std::collections::BTreeSet<&String> = sdc.clocks.iter().map(|c| &c.name).collect();
    for source in [false, true] {
        let vals: Vec<Vec<f64>> = clocks.iter().map(|c| per.get(&(source, c.to_string())).cloned().unwrap_or_default()).collect();
        let first = vals.first().cloned().unwrap_or_default();
        if vals.iter().any(|v| v != &first) || first.windows(2).any(|w| w[0] != w[1]) {
            why.push(format!("{} latency differs between clocks or edges", if source { "source" } else { "network" }));
        }
    }
    let included = ["-source_latency_included", "-network_latency_included"];
    if sdc.input_delays.iter().chain(&sdc.output_delays).any(|d| d.flags.iter().any(|f| included.contains(&f.as_str()))) {
        why.push("an I/O delay that includes it".to_string());
    }
    (!why.is_empty()).then(|| format!("clock latency not modelled where it does not cancel: {}", why.join("; ")))
}

#[cfg(test)]
mod clock_latency_rule {
    use super::*;

    /// Reference rule: a clock's latency is on both the launch and the capture side, so on one
    /// clock it cancels — the I/O delays are not shifted by it (measured: every slack unchanged).
    #[test]
    fn latency_on_one_clock_moves_no_io_delay() {
        let sdc = crate::sdc::Sdc::parse(
            "create_clock -name clk -period 1 [get_ports clk]\n\
             set_input_delay 0.2 -clock clk [get_ports a]\n\
             set_output_delay 0.3 -clock clk [get_ports y]\n\
             set_clock_latency 0.5 [get_clocks clk]\n\
             set_clock_latency -source 0.4 [get_clocks clk]\n",
        )
        .unwrap();
        let mut job = StaJob::parse("design: t\nnetlist: t.v\nlib: t.lib\nclock: clk 1.0\n", ".").unwrap();
        merge_sdc_into(&sdc, &mut job);
        assert_eq!(job.io_input_delays, vec![("a".to_string(), 0.2)]);
        assert_eq!(job.io_output_delays, vec![("y".to_string(), 0.3)]);
        assert!(latency_not_modelled(&sdc).is_none());
    }

    /// Where it does not cancel, the run is told.
    #[test]
    fn latency_that_does_not_cancel_is_named() {
        let two = "create_clock -name a -period 1 [get_ports ca]\ncreate_clock -name b -period 1 [get_ports cb]\n";
        let differ = crate::sdc::Sdc::parse(&format!("{two}set_clock_latency 0.5 [get_clocks a]\n")).unwrap();
        assert!(latency_not_modelled(&differ).unwrap().contains("network latency differs"));
        let same = crate::sdc::Sdc::parse(&format!("{two}set_clock_latency 0.5 [get_clocks {{a b}}]\n")).unwrap();
        assert!(latency_not_modelled(&same).is_none());
        let pin = crate::sdc::Sdc::parse(&format!("{two}set_clock_latency 0.5 [get_pins r1/CK]\n")).unwrap();
        assert!(latency_not_modelled(&pin).unwrap().contains("pin or port"));
    }
}
