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
    pub clock_port: String,
    pub period_ns: f64,
    pub input_slew: f64,
    pub output_load: f64,
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
    // MCMM: when non-empty, this is a multi-corner/multi-mode job — each entry is a
    // path to a single-scenario `.sta`; the engine runs all and reports the worst
    // setup and worst hold across them. The fields above are then unused.
    pub scenarios: Vec<String>,
    pub base_dir: String,
}

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
        for raw in text.lines() {
            let line = strip_comment(raw).trim();
            if line.is_empty() {
                continue;
            }
            let (k, v) = line
                .split_once(':')
                .ok_or_else(|| JobError(format!("expected 'key: value', got {line:?}")))?;
            kv.insert(k.trim().to_lowercase(), v.trim().to_string());
        }
        let get = |k: &str| kv.get(k).cloned().ok_or_else(|| JobError(format!("missing key: {k}")));
        let split_list = |s: &str| {
            s.split(',').map(|t| t.trim().to_string()).filter(|t| !t.is_empty()).collect::<Vec<_>>()
        };
        let scenarios = kv.get("scenarios").map(|s| split_list(s)).unwrap_or_default();
        let mcmm = !scenarios.is_empty();
        // clock/netlist/lib are required for a single run, optional for an MCMM job
        // (each scenario file carries its own).
        let (clock_port, period_ns) = match kv.get("clock") {
            Some(clock) => {
                let mut parts = clock.split_whitespace();
                let port = parts.next().unwrap_or("").to_string();
                let period = parts
                    .next()
                    .and_then(|s| s.parse().ok())
                    .ok_or_else(|| JobError("clock needs 'port period_ns'".into()))?;
                (port, period)
            }
            None if mcmm => (String::new(), 0.0),
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
            input_slew: num("input_slew", 0.05),
            output_load: num("output_load", 0.005),
            late_derate: num("late_derate", 1.0),
            early_derate: num("early_derate", 1.0),
            pocv_sigma: num("pocv_sigma", 0.0),
            pocv_n: num("pocv_n", 3.0),
            aocv_late: kv.get("aocv_late").map(|s| pairs(s)).unwrap_or_default(),
            aocv_early: kv.get("aocv_early").map(|s| pairs(s)).unwrap_or_default(),
            miller: num("miller", 2.0),
            xtalk_window: num("xtalk_window", 0.0), // guard band on top of slew-derived window
            scenarios,
            base_dir: base_dir.to_string(),
        };
        job.validate()?;
        Ok(job)
    }

    pub fn load(path: &str) -> Result<StaJob, JobError> {
        let text = std::fs::read_to_string(path).map_err(|e| JobError(format!("{path}: {e}")))?;
        let base = Path::new(path).parent().and_then(|p| p.to_str()).unwrap_or(".");
        StaJob::parse(&text, base)
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
