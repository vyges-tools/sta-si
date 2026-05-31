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
    pub miller: f64, // crosstalk Miller coupling factor (2.0 worst late; 1.0 disables SI)
    pub xtalk_window: f64, // ns — guard band added to the slew-derived switching window
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
        let clock = get("clock")?;
        let mut parts = clock.split_whitespace();
        let clock_port = parts.next().unwrap_or("").to_string();
        let period_ns = parts
            .next()
            .and_then(|s| s.parse().ok())
            .ok_or_else(|| JobError("clock needs 'port period_ns'".into()))?;
        let num = |k: &str, d: f64| kv.get(k).and_then(|s| s.parse().ok()).unwrap_or(d);
        let job = StaJob {
            design: get("design")?,
            netlist: get("netlist")?,
            libs: get("lib")?
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect(),
            spef: kv.get("spef").filter(|s| !s.is_empty()).cloned(),
            clock_port,
            period_ns,
            input_slew: num("input_slew", 0.05),
            output_load: num("output_load", 0.005),
            late_derate: num("late_derate", 1.0),
            miller: num("miller", 2.0),
            xtalk_window: num("xtalk_window", 0.0), // guard band on top of slew-derived window
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

    fn validate(&self) -> Result<(), JobError> {
        if self.netlist.is_empty() || self.libs.is_empty() {
            return Err(JobError("netlist and at least one lib are required".into()));
        }
        if self.clock_port.is_empty() || self.period_ns <= 0.0 {
            return Err(JobError("clock port + positive period required".into()));
        }
        Ok(())
    }
}
