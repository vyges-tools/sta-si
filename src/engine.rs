//! STA engine wiring: job → netlist + merged Liberty → analyze → report text.
//!
//! Files in / report out; no subprocess (v0 is fully self-contained). OpenSTA
//! is the correlation baseline this engine is checked against, not a runtime
//! dependency — so `analyze_job` runs anywhere.

use crate::job::StaJob;
use crate::liberty::Lib;
use crate::netlist;
use crate::spef::Spef;
use crate::sta::{self, StaError, TimingReport};

const DEMO_LIB: &str = r#"
library (demo) {
  delay_model : table_lookup;
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
}
"#;

const DEMO_NETLIST: &str = r#"
module top ( a, y );
  input a;
  output y;
  wire n1;
  INV u1 ( .A(a),  .Y(n1) );
  INV u2 ( .A(n1), .Y(y)  );
endmodule
"#;

/// A built-in 2-inverter design analyzed offline (for `demo`).
pub fn demo() -> (StaJob, TimingReport) {
    let job = StaJob {
        design: "demo".into(),
        netlist: "(builtin)".into(),
        libs: vec!["(builtin)".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 1.0,
        clocks: vec![],
        input_slew: 0.02,
        output_load: 0.005,
        input_delay: 0.0,
        output_delay: 0.0,
        io_input_delays: vec![],
        io_output_delays: vec![],
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions: vec![],
        async_groups: vec![],
        crpr: true,
        pba: false,
        sdc: None,
        base_dir: String::new(),
    };
    let rep = analyze_inputs(DEMO_NETLIST, DEMO_LIB, &job).unwrap_or(TimingReport {
        wns: f64::INFINITY,
        tns: 0.0,
        endpoints: 0,
        worst_endpoint: String::new(),
        worst_path: Vec::new(),
        whs: f64::INFINITY,
        ths: 0.0,
        hold_endpoints: 0,
        worst_hold_endpoint: String::new(),
        worst_hold_path: Vec::new(),
        hold_slacks: Vec::new(),
        pba_wns: None,
    });
    (job, rep)
}

/// Load the job's netlist + (merged) Liberty and run STA.
pub fn analyze_job(job: &StaJob) -> Result<TimingReport, StaError> {
    analyze_job_opts(job, crate::liberty::LibOpts::default())
}

/// Like [`analyze_job`] but with explicit Liberty load options — e.g. `skip_ccs`
/// for `--liberty-nldm-only`. CCS pruning is a load-time choice (not job state), so
/// it is a parameter here rather than a field on [`StaJob`].
pub fn analyze_job_opts(
    job: &StaJob,
    lib_opts: crate::liberty::LibOpts,
) -> Result<TimingReport, StaError> {
    let nl = netlist::load(&job.resolve(&job.netlist)).map_err(|e| StaError::Parse(e.to_string()))?;
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load_opts(&job.resolve(l), lib_opts)
            .map_err(|e| StaError::Parse(e.to_string()))?;
        lib.cells.extend(one.cells); // later libs override earlier on name clash
    }
    if lib.cells.is_empty() {
        return Err(StaError::Parse("no cells in any .lib".into()));
    }
    let spef = match &job.spef {
        Some(p) => Some(Spef::load(&job.resolve(p)).map_err(|e| StaError::Parse(e.to_string()))?),
        None => None,
    };
    sta::analyze(&nl, &lib, job, spef.as_ref())
}

/// Emit an SDF back-annotation file for the job's design (loads the same Liberty
/// + netlist + SPEF as [`analyze_job`]).
pub fn sdf_for_job(job: &StaJob) -> Result<String, StaError> {
    sdf_for_job_opts(job, crate::liberty::LibOpts::default())
}

/// Like [`sdf_for_job`] but with explicit Liberty load options (`skip_ccs`).
pub fn sdf_for_job_opts(
    job: &StaJob,
    lib_opts: crate::liberty::LibOpts,
) -> Result<String, StaError> {
    let nl = netlist::load(&job.resolve(&job.netlist)).map_err(|e| StaError::Parse(e.to_string()))?;
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load_opts(&job.resolve(l), lib_opts)
            .map_err(|e| StaError::Parse(e.to_string()))?;
        lib.cells.extend(one.cells);
    }
    if lib.cells.is_empty() {
        return Err(StaError::Parse("no cells in any .lib".into()));
    }
    let spef = match &job.spef {
        Some(p) => Some(Spef::load(&job.resolve(p)).map_err(|e| StaError::Parse(e.to_string()))?),
        None => None,
    };
    Ok(crate::sdf::emit(&job.design, &nl, &lib, spef.as_ref()))
}

/// Emit the shared Liberty IR (merged across the job's libs) as JSON — the
/// structured intermediate for inspection / MCP (`--emit-liberty-json`). Uses the
/// same loom `Lib` that both the timer and vyges-power consume, so the dump reflects
/// exactly what the analysis sees. `lib_opts` honours `--liberty-nldm-only`.
pub fn liberty_json_for_job(
    job: &StaJob,
    lib_opts: crate::liberty::LibOpts,
) -> Result<String, StaError> {
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load_opts(&job.resolve(l), lib_opts)
            .map_err(|e| StaError::Parse(e.to_string()))?;
        lib.cells.extend(one.cells);
    }
    if lib.cells.is_empty() {
        return Err(StaError::Parse("no cells in any .lib".into()));
    }
    Ok(lib.to_json())
}

/// Lint a job's SDC constraints (completeness + consistency) against its netlist.
/// Uses the job's SDC file if present, else the inline `.sta` clock definitions.
pub fn lint_job(job: &StaJob) -> Result<crate::sdclint::LintReport, StaError> {
    let nl = netlist::load(&job.resolve(&job.netlist)).map_err(|e| StaError::Parse(e.to_string()))?;
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load(&job.resolve(l)).map_err(|e| StaError::Parse(e.to_string()))?;
        lib.cells.extend(one.cells);
    }
    if lib.cells.is_empty() {
        return Err(StaError::Parse("no cells in any .lib".into()));
    }
    let sdc = match &job.sdc {
        Some(p) => crate::sdc::Sdc::load(&job.resolve(p)).map_err(|e| StaError::Parse(e.to_string()))?,
        None => {
            // no SDC file — lint the inline `.sta` clocks (still catches a bad period,
            // a duplicate, a registered design with no clock at all).
            let mut s = crate::sdc::Sdc::default();
            for (name, source, period) in &job.clocks {
                s.clocks.push(crate::sdc::SdcClock {
                    name: name.clone(),
                    source: source.clone(),
                    period: *period,
                });
            }
            s
        }
    };
    Ok(crate::sdclint::lint(&nl, &sdc, &lib))
}

/// Design could close at least this many× faster than clocked → "over-margined".
const OVER_MARGIN_RATIO: f64 = 1.5;
/// Hold slack (ns) at/below which an endpoint is "hold-critical" — a hold-fixer will pad it.
const HOLD_CRIT_MARGIN_NS: f64 = 0.05;
/// Ignore hold floods below this absolute count (tiny designs are not a burden).
const HOLD_FLOOD_MIN: usize = 8;
/// …and require at least this fraction of hold endpoints to be hold-critical.
const HOLD_FLOOD_FRAC: f64 = 0.10;

/// Timing-health advisory derived from the setup/hold report: the clock the design can
/// actually close at (from the worst setup path), plus an over-margin / hold-flood warning.
///
/// Over-margining a clock (running much slower than the design can close) manufactures
/// hold-critical paths: a post-layout resizer / hold-fixer then floods the design with
/// delay buffers and can hit its buffer budget (e.g. OpenROAD `RSZ-0060`), failing the
/// harden. sta-si can see this at sign-off and warn *before* the harden fails, turning a
/// pass/fail timer into a design-feedback tool (issue #10).
pub struct MarginAdvisory {
    /// Minimum clock period the worst setup path can close at (ns): `period − WNS`.
    pub achievable_ns: f64,
    /// Max frequency at `achievable_ns` (MHz); `None` when the path is ~combinational
    /// (`achievable ≤ 0`) so a finite frequency is not meaningful.
    pub max_freq_mhz: Option<f64>,
    /// Frequency at the target (clocked) period (MHz).
    pub target_freq_mhz: f64,
    /// How many× faster the design could close than it is clocked (`period / achievable`);
    /// `None` when `achievable ≤ 0` (effectively unbounded).
    pub over_margin_ratio: Option<f64>,
    /// Hold endpoints at or below the hold-critical margin (flood risk for a hold-fixer).
    pub hold_critical: usize,
    /// True when the clock is over-margined AND a hold flood is present — the harden-risk case.
    pub warn: bool,
}

impl MarginAdvisory {
    /// Derive the advisory from the target period and a completed report. Returns `None`
    /// when there is no setup timing to reason about (no endpoints, non-finite WNS, or a
    /// non-positive period).
    pub fn compute(period_ns: f64, rep: &TimingReport) -> Option<MarginAdvisory> {
        if rep.endpoints == 0 || !rep.wns.is_finite() || period_ns <= 0.0 {
            return None;
        }
        // Minimum period the worst setup path needs. WNS>0 (margin) shrinks it; WNS<0
        // (violated) grows it beyond the target — i.e. the clock is too fast.
        let achievable = period_ns - rep.wns;
        let (max_freq_mhz, over_margin_ratio) = if achievable > 1e-3 {
            (Some(1000.0 / achievable), Some(period_ns / achievable))
        } else {
            (None, None) // critical path ≈ 0 at this clock — finite freq not meaningful
        };
        // Over-margined when the design can close ≥ OVER_MARGIN_RATIO× faster than clocked
        // (achievable ≤ 0 satisfies this too).
        let over_margin = achievable <= period_ns / OVER_MARGIN_RATIO;
        let hold_critical =
            rep.hold_slacks.iter().filter(|&&(_, sl)| sl <= HOLD_CRIT_MARGIN_NS).count();
        let hold_flood = rep.hold_endpoints > 0
            && hold_critical >= HOLD_FLOOD_MIN
            && (hold_critical as f64) >= HOLD_FLOOD_FRAC * rep.hold_endpoints as f64;
        Some(MarginAdvisory {
            achievable_ns: achievable,
            max_freq_mhz,
            target_freq_mhz: 1000.0 / period_ns,
            over_margin_ratio,
            hold_critical,
            warn: over_margin && hold_flood,
        })
    }
}

/// Render a human-readable timing report.
pub fn render_report(job: &StaJob, rep: &TimingReport) -> String {
    let mut s = String::new();
    s.push_str(&format!("STA report — design {}\n", job.design));
    if job.clocks.len() > 1 {
        let cl: Vec<String> =
            job.clocks.iter().map(|(n, _, p)| format!("{n}@{p:.2}ns")).collect();
        s.push_str(&format!("  clocks: {}   xtalk_miller {:.2}\n", cl.join(", "), job.miller));
    } else {
        s.push_str(&format!(
            "  clock {}  period {:.3} ns   xtalk_miller {:.2}\n",
            job.clock_port, job.period_ns, job.miller
        ));
    }
    let ocv = if job.pocv_sigma > 0.0 {
        format!("POCV — per-stage sigma {:.3}, {:.1}-sigma band", job.pocv_sigma, job.pocv_n)
    } else if !job.aocv_late.is_empty() || !job.aocv_early.is_empty() {
        "AOCV — depth-dependent derate table".to_string()
    } else {
        format!("flat derate — late {:.3} / early {:.3}", job.late_derate, job.early_derate)
    };
    s.push_str(&format!("  OCV: {ocv}   CRPR: {}\n", if job.crpr { "on" } else { "off" }));
    if let Some(pba) = rep.pba_wns {
        let v = if pba >= 0.0 { "MET" } else { "VIOLATED" };
        s.push_str(&format!("  PBA WNS: {pba:.4} ns   [{v}]  (path-based re-timing)\n"));
    }
    s.push_str(&format!("  endpoints: {}\n", rep.endpoints));
    if rep.endpoints == 0 {
        s.push_str("  (no timing endpoints — no primary outputs reached)\n");
        return s;
    }
    let verdict = if rep.wns >= 0.0 { "MET" } else { "VIOLATED" };
    s.push_str(&format!(
        "  WNS: {:.4} ns    TNS: {:.4} ns    [{}]\n\n",
        rep.wns, rep.tns, verdict
    ));
    s.push_str(&format!(
        "  worst path to {}  (slack {:.4} ns):\n",
        rep.worst_endpoint, rep.wns
    ));
    s.push_str(&format!("    {:>9}  {:>7}   node\n", "arrival", "slew"));
    for p in &rep.worst_path {
        s.push_str(&format!("    {:9.4}  {:7.4}   {}\n", p.arrival, p.slew, p.label));
    }
    if rep.hold_endpoints > 0 {
        let hverdict = if rep.whs >= 0.0 { "MET" } else { "VIOLATED" };
        s.push_str(&format!(
            "\n  hold endpoints: {}    WHS: {:.4} ns    THS: {:.4} ns    [{}]\n",
            rep.hold_endpoints, rep.whs, rep.ths, hverdict
        ));
        s.push_str(&format!(
            "  worst hold path to {}  (slack {:.4} ns):\n",
            rep.worst_hold_endpoint, rep.whs
        ));
        s.push_str(&format!("    {:>9}  {:>7}   node\n", "arrival", "slew"));
        for p in &rep.worst_hold_path {
            s.push_str(&format!("    {:9.4}  {:7.4}   {}\n", p.arrival, p.slew, p.label));
        }
    }
    // Timing-health advisory: achievable clock + over-margin / hold-flood warning (#10).
    if let Some(adv) = MarginAdvisory::compute(job.period_ns, rep) {
        match adv.max_freq_mhz {
            Some(f) => s.push_str(&format!(
                "\n  achievable: ~{:.3} ns → ~{:.1} MHz   (target {:.3} ns → ~{:.1} MHz)\n",
                adv.achievable_ns, f, job.period_ns, adv.target_freq_mhz
            )),
            None => s.push_str(&format!(
                "\n  achievable: critical path ≈ 0 at this clock   (target {:.3} ns → ~{:.1} MHz)\n",
                job.period_ns, adv.target_freq_mhz
            )),
        }
        if adv.warn {
            let ratio = adv
                .over_margin_ratio
                .map(|r| format!("~{r:.1}× faster than clocked"))
                .unwrap_or_else(|| "far faster than clocked".to_string());
            let freq = adv
                .max_freq_mhz
                .map(|f| format!("~{f:.1} MHz"))
                .unwrap_or_else(|| "a higher frequency".to_string());
            s.push_str(&format!(
                "  WARNING: over-margin — design closes {ratio} ({:.3} ns achievable vs {:.3} ns target);\n           {} of {} hold endpoints are hold-critical (≤ {:.2} ns) → expect a heavy hold-fix / buffer-budget burden.\n           Consider a faster clock ({freq} achievable) or accept the hold-fix cost.\n",
                adv.achievable_ns, job.period_ns, adv.hold_critical, rep.hold_endpoints, HOLD_CRIT_MARGIN_NS
            ));
        }
    }
    s
}

/// Render the report as machine-readable JSON (std-only, no deps).
pub fn report_json(job: &StaJob, rep: &TimingReport) -> String {
    let num = |v: f64| if v.is_finite() { format!("{v:.6}") } else { "null".to_string() };
    let mut s = String::new();
    s.push('{');
    s.push_str(&format!("\"design\":{:?},", job.design));
    s.push_str(&format!("\"clock\":{:?},", job.clock_port));
    s.push_str(&format!("\"period_ns\":{:.6},", job.period_ns));
    s.push_str(&format!("\"xtalk_miller\":{:.2},", job.miller));
    s.push_str(&format!("\"endpoints\":{},", rep.endpoints));
    s.push_str(&format!("\"wns_ns\":{},", num(rep.wns)));
    s.push_str(&format!("\"tns_ns\":{},", num(rep.tns)));
    s.push_str(&format!("\"met\":{},", rep.endpoints > 0 && rep.wns >= 0.0));
    s.push_str(&format!("\"pba_wns_ns\":{},", rep.pba_wns.map(num).unwrap_or_else(|| "null".into())));
    s.push_str(&format!("\"hold_endpoints\":{},", rep.hold_endpoints));
    s.push_str(&format!("\"whs_ns\":{},", num(rep.whs)));
    s.push_str(&format!("\"ths_ns\":{},", num(rep.ths)));
    s.push_str(&format!("\"hold_met\":{},", rep.hold_endpoints > 0 && rep.whs >= 0.0));
    // Timing-health advisory (#10): achievable clock + over-margin / hold-flood warning.
    if let Some(adv) = MarginAdvisory::compute(job.period_ns, rep) {
        s.push_str(&format!("\"achievable_period_ns\":{:.6},", adv.achievable_ns));
        s.push_str(&format!(
            "\"max_freq_mhz\":{},",
            adv.max_freq_mhz.map(|f| format!("{f:.3}")).unwrap_or_else(|| "null".into())
        ));
        s.push_str(&format!("\"target_freq_mhz\":{:.3},", adv.target_freq_mhz));
        s.push_str(&format!(
            "\"over_margin_ratio\":{},",
            adv.over_margin_ratio.map(|r| format!("{r:.3}")).unwrap_or_else(|| "null".into())
        ));
        s.push_str(&format!("\"hold_critical_endpoints\":{},", adv.hold_critical));
        s.push_str(&format!("\"over_margin_warning\":{},", adv.warn));
    }
    s.push_str(&format!("\"worst_hold_endpoint\":{:?},", rep.worst_hold_endpoint));
    s.push_str(&format!("\"worst_endpoint\":{:?},", rep.worst_endpoint));
    s.push_str("\"worst_path\":[");
    for (i, p) in rep.worst_path.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!(
            "{{\"label\":{:?},\"arrival_ns\":{:.6},\"slew_ns\":{:.6}}}",
            p.label, p.arrival, p.slew
        ));
    }
    s.push_str("]}\n");
    s
}

// ---- MCMM (multi-corner / multi-mode) -----------------------------------

/// One scenario's analysis within an MCMM run.
pub struct ScenarioResult {
    pub name: String,
    pub period_ns: f64,
    pub report: TimingReport,
}

/// Aggregated result of an MCMM job: every scenario, plus the worst setup and
/// worst hold across them (sign-off is the worst corner per check).
pub struct McmmReport {
    pub scenarios: Vec<ScenarioResult>,
}

impl McmmReport {
    /// Worst (minimum) setup slack across scenarios that have setup endpoints.
    pub fn worst_setup(&self) -> Option<(&str, f64)> {
        self.scenarios
            .iter()
            .filter(|s| s.report.endpoints > 0)
            .map(|s| (s.name.as_str(), s.report.wns))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
    /// Worst (minimum) hold slack across scenarios that have hold endpoints.
    pub fn worst_hold(&self) -> Option<(&str, f64)> {
        self.scenarios
            .iter()
            .filter(|s| s.report.hold_endpoints > 0)
            .map(|s| (s.name.as_str(), s.report.whs))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
}

/// Run every scenario `.sta` listed in the MCMM job and collect the results.
/// Each scenario is a full, independent STA (own corner libs, derates, clock).
pub fn analyze_mcmm(job: &StaJob, lib_opts: crate::liberty::LibOpts) -> Result<McmmReport, StaError> {
    let mut scenarios = Vec::new();
    for s in &job.scenarios {
        let sub = StaJob::load(&job.resolve(s)).map_err(|e| StaError::Parse(e.to_string()))?;
        let report = analyze_job_opts(&sub, lib_opts)?; // CLI knob propagates to every corner
        // Label the row by the scenario file (the corner identity, e.g. `ss_n40C_1v60`),
        // not the design name — every scenario shares the same design.
        scenarios.push(ScenarioResult { name: scenario_label(s), period_ns: sub.period_ns, report });
    }
    if scenarios.is_empty() {
        return Err(StaError::Parse("MCMM job lists no scenarios".into()));
    }
    Ok(McmmReport { scenarios })
}

/// A scenario's display label: its file's basename without the `.sta` extension.
fn scenario_label(path: &str) -> String {
    let base = path.rsplit(['/', '\\']).next().unwrap_or(path);
    base.strip_suffix(".sta").unwrap_or(base).to_string()
}

/// Human-readable MCMM report: per-scenario slacks + the worst corner per check.
pub fn render_mcmm(job: &StaJob, rep: &McmmReport) -> String {
    let mut s = String::new();
    s.push_str(&format!("MCMM report — design {}\n", job.design));
    s.push_str(&format!("  scenarios: {}\n\n", rep.scenarios.len()));
    s.push_str(&format!(
        "  {:<20} {:>8}  {:>12}  {:>12}   verdict\n",
        "scenario", "period", "WNS setup", "WHS hold"
    ));
    for sc in &rep.scenarios {
        let r = &sc.report;
        let setup_bad = r.endpoints > 0 && r.wns < 0.0;
        let hold_bad = r.hold_endpoints > 0 && r.whs < 0.0;
        let verdict = match (setup_bad, hold_bad) {
            (false, false) => "MET",
            (true, false) => "SETUP VIOLATED",
            (false, true) => "HOLD VIOLATED",
            (true, true) => "SETUP+HOLD VIOLATED",
        };
        let wns = if r.endpoints > 0 { format!("{:.4}", r.wns) } else { "  —".into() };
        let whs = if r.hold_endpoints > 0 { format!("{:.4}", r.whs) } else { "  —".into() };
        s.push_str(&format!(
            "  {:<20} {:>8.3}  {:>12}  {:>12}   {}\n",
            sc.name, sc.period_ns, wns, whs, verdict
        ));
    }
    s.push('\n');
    match rep.worst_setup() {
        Some((name, wns)) => s.push_str(&format!(
            "  worst setup: {:.4} ns  (scenario {})   [{}]\n",
            wns, name, if wns >= 0.0 { "MET" } else { "VIOLATED" }
        )),
        None => s.push_str("  worst setup: (no setup endpoints)\n"),
    }
    match rep.worst_hold() {
        Some((name, whs)) => s.push_str(&format!(
            "  worst hold:  {:.4} ns  (scenario {})   [{}]\n",
            whs, name, if whs >= 0.0 { "MET" } else { "VIOLATED" }
        )),
        None => s.push_str("  worst hold:  (no hold endpoints)\n"),
    }
    s
}

/// MCMM report as machine-readable JSON.
pub fn mcmm_json(job: &StaJob, rep: &McmmReport) -> String {
    let num = |v: f64| if v.is_finite() { format!("{v:.6}") } else { "null".to_string() };
    let mut s = String::new();
    s.push('{');
    s.push_str(&format!("\"design\":{:?},", job.design));
    s.push_str("\"scenarios\":[");
    for (i, sc) in rep.scenarios.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let r = &sc.report;
        s.push_str(&format!(
            "{{\"name\":{:?},\"period_ns\":{:.6},\"wns_ns\":{},\"tns_ns\":{},\"whs_ns\":{},\"ths_ns\":{}}}",
            sc.name, sc.period_ns, num(r.wns), num(r.tns), num(r.whs), num(r.ths)
        ));
    }
    s.push_str("],");
    let ws = rep.worst_setup();
    let wh = rep.worst_hold();
    s.push_str(&format!(
        "\"worst_setup_ns\":{},",
        ws.map(|x| num(x.1)).unwrap_or_else(|| "null".into())
    ));
    s.push_str(&format!(
        "\"worst_setup_scenario\":{},",
        ws.map(|x| format!("{:?}", x.0)).unwrap_or_else(|| "null".into())
    ));
    s.push_str(&format!(
        "\"worst_hold_ns\":{},",
        wh.map(|x| num(x.1)).unwrap_or_else(|| "null".into())
    ));
    s.push_str(&format!(
        "\"worst_hold_scenario\":{},",
        wh.map(|x| format!("{:?}", x.0)).unwrap_or_else(|| "null".into())
    ));
    let met = ws.map(|x| x.1 >= 0.0).unwrap_or(true) && wh.map(|x| x.1 >= 0.0).unwrap_or(true);
    s.push_str(&format!("\"met\":{met}"));
    s.push_str("}\n");
    s
}

/// Convenience for callers that already hold parsed inputs (used by `demo`).
pub fn analyze_inputs(
    nl_text: &str,
    lib_text: &str,
    job: &StaJob,
) -> Result<TimingReport, StaError> {
    let nl = netlist::parse(nl_text).map_err(|e| StaError::Parse(e.to_string()))?;
    let lib = Lib::parse(lib_text).map_err(|e| StaError::Parse(e.to_string()))?;
    sta::analyze(&nl, &lib, job, None)
}
