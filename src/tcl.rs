//! Layer-1 OpenSTA integration — the `tcl` subcommand.
//!
//! Ingests an **OpenSTA-style TCL *subset*** and drives the vyges-sta-si engine,
//! emitting OpenSTA-flavoured reports. This is **not** a TCL interpreter and **not**
//! a drop-in for LibreLane's `corner.tcl` (which needs OpenSTA's command API +
//! LibreLane's `io.tcl` + OpenROAD's ODB) — see `docs/opensta-integration.md`. It
//! interoperates with the *portable* command set a hand-written OpenSTA script uses:
//!
//! ```tcl
//! read_liberty [-corner c] cells.lib   ;# one or more
//! read_verilog top.v
//! link_design  top
//! read_spef    top.spef                ;# optional
//! read_sdc     top.sdc                  ;# and/or inline SDC below
//! create_clock -name clk -period 5 [get_ports clk]
//! report_checks -path_delay min_max
//! report_wns ; report_tns
//! ```
//!
//! The whole constraint half (`read_sdc` + inline `create_clock`/`set_*`) is routed
//! through the existing [`crate::sdc`] parser, so anything SDC already supports works
//! here unchanged. Commands outside the subset (`read_current_odb`,
//! `estimate_parasitics`, `report_power`, `check_setup`, `report_check_types`,
//! `sta::*`, `write_metric_*`, …) are reported as "ignored" — never silently dropped.

use std::path::Path;

use crate::job::StaJob;
use crate::sdc::Sdc;

#[derive(Debug)]
pub struct TclError(pub String);
impl std::fmt::Display for TclError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "tcl adapter: {}", self.0)
    }
}
impl std::error::Error for TclError {}

/// Which `report_*` commands the script asked for (and their min/max sense).
#[derive(Default, Clone)]
pub struct Reports {
    pub checks: bool,
    pub checks_min: bool,
    pub checks_max: bool,
    pub wns: bool,
    pub wns_min: bool,
    pub tns: bool,
    pub tns_min: bool,
    pub worst_slack: bool,
    pub worst_slack_min: bool,
}

impl Reports {
    /// Nothing explicitly requested → behave like a default `report_checks` + WNS/TNS.
    fn defaulted(mut self) -> Reports {
        if !(self.checks || self.wns || self.tns || self.worst_slack) {
            self.checks = true;
            self.checks_max = true;
            self.wns = true;
            self.tns = true;
        }
        self
    }
}

/// The result of adapting a script: the synthesized job, the requested reports, and
/// the list of commands we recognised but do not support.
pub struct Adapted {
    pub job: StaJob,
    pub reports: Reports,
    pub ignored: Vec<String>,
}

/// SDC constraint verbs we pass straight (raw line) to [`crate::sdc`].
const SDC_VERBS: &[&str] = &[
    "create_clock",
    "create_generated_clock",
    "set_input_delay",
    "set_output_delay",
    "set_clock_uncertainty",
    "set_clock_latency",
    "set_clock_transition",
    "set_propagated_clock",
    "set_input_transition",
    "set_load",
    "set_driving_cell",
    "set_false_path",
    "set_multicycle_path",
    "set_max_delay",
    "set_min_delay",
    "set_disable_timing",
    "set_case_analysis",
    "set_clock_groups",
    "set_timing_derate",
    "group_path",
    "current_design",
];

/// Parse an OpenSTA-subset script and build an equivalent [`StaJob`].
pub fn adapt(script_path: &str) -> Result<Adapted, TclError> {
    let text = std::fs::read_to_string(script_path)
        .map_err(|e| TclError(format!("{script_path}: {e}")))?;
    let base_dir = Path::new(script_path)
        .parent()
        .and_then(|p| p.to_str())
        .filter(|s| !s.is_empty())
        .unwrap_or(".")
        .to_string();

    let mut design = String::new();
    let mut netlist = String::new();
    let mut libs: Vec<String> = Vec::new();
    let mut spef: Option<String> = None;
    let mut combined_sdc = String::new(); // sourced .sdc contents + inline SDC lines, in order
    let mut reports = Reports::default();
    let mut ignored: Vec<String> = Vec::new();

    for raw in commands(&text) {
        let toks = tokenize(&raw);
        let verb = toks.first().map(|s| s.as_str()).unwrap_or("");
        match verb {
            "" => {}
            "read_verilog" => {
                if let Some(f) = first_file(&toks) {
                    netlist = f;
                }
            }
            "link_design" => {
                if let Some(t) = first_file(&toks) {
                    design = t;
                }
            }
            "read_liberty" => {
                if let Some(f) = first_file(&toks) {
                    libs.push(f);
                }
            }
            "read_spef" => {
                if let Some(f) = first_file(&toks) {
                    spef = Some(f);
                }
            }
            "read_sdc" | "source" => match first_file(&toks) {
                Some(f) if verb == "read_sdc" || f.ends_with(".sdc") => {
                    let p = resolve(&base_dir, &f);
                    let c = std::fs::read_to_string(&p)
                        .map_err(|e| TclError(format!("read_sdc {p}: {e}")))?;
                    combined_sdc.push_str(&c);
                    combined_sdc.push('\n');
                }
                _ => ignored.push(raw.clone()),
            },
            "set_cmd_units" | "set_units" => { /* assume ns/pF/kOhm/… (engine default); no-op */ }
            "report_checks" => {
                reports.checks = true;
                let (mn, mx) = path_delay(&toks);
                reports.checks_min |= mn;
                reports.checks_max |= mx;
            }
            "report_wns" => {
                reports.wns = true;
                reports.wns_min |= has_flag(&toks, "-min");
            }
            "report_tns" => {
                reports.tns = true;
                reports.tns_min |= has_flag(&toks, "-min");
            }
            "report_worst_slack" => {
                reports.worst_slack = true;
                reports.worst_slack_min |= has_flag(&toks, "-min");
            }
            v if SDC_VERBS.contains(&v) => {
                combined_sdc.push_str(&raw);
                combined_sdc.push('\n');
            }
            other => ignored.push(other.to_string()),
        }
    }

    if design.is_empty() {
        return Err(TclError(
            "no `link_design <top>` — cannot determine the design".into(),
        ));
    }
    if netlist.is_empty() {
        return Err(TclError("no `read_verilog <netlist>`".into()));
    }
    if libs.is_empty() {
        return Err(TclError("no `read_liberty <lib>`".into()));
    }

    let mut job = base_job(design, netlist, libs, spef, &base_dir);

    if !combined_sdc.trim().is_empty() {
        let sdc = Sdc::parse(&combined_sdc).map_err(|e| TclError(e.to_string()))?;
        crate::job::merge_sdc_into(&sdc, &mut job);
        for ig in &sdc.ignored {
            ignored.push(format!("(sdc) {ig}"));
        }
    }

    // Derive the primary clock from the merged SDC clocks (mirrors StaJob::parse).
    if job.clock_port.is_empty() {
        if let Some((_, src, per)) = job.clocks.first() {
            job.clock_port = src.clone();
            job.period_ns = *per;
        }
    }
    if job.clock_port.is_empty() || job.period_ns <= 0.0 {
        return Err(TclError(
            "no clock defined — add a `create_clock` (inline or via read_sdc)".into(),
        ));
    }

    ignored.sort();
    ignored.dedup();
    Ok(Adapted {
        job,
        reports: reports.defaulted(),
        ignored,
    })
}

/// A bare [`StaJob`] with the same defaults `StaJob::parse` uses; the constraint
/// fields are filled by the SDC merge.
fn base_job(
    design: String,
    netlist: String,
    libs: Vec<String>,
    spef: Option<String>,
    base_dir: &str,
) -> StaJob {
    StaJob {
        design,
        netlist,
        libs,
        spef,
        clock_port: String::new(),
        period_ns: 0.0,
        clocks: Vec::new(),
        input_slew: 0.05,
        output_load: 0.005,
        input_delay: 0.0,
        output_delay: 0.0,
        io_input_delays: Vec::new(),
        io_output_delays: Vec::new(),
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: Vec::new(),
        aocv_early: Vec::new(),
        miller: 2.0,
        xtalk_window: 0.0,
        crpr: true,
        pba: false,
        scenarios: Vec::new(),
        exceptions: Vec::new(),
        async_groups: Vec::new(),
        sdc: None,
        base_dir: base_dir.to_string(),
    }
}

// ---- OpenSTA-flavoured report rendering ---------------------------------

/// Render the analysis as an OpenSTA-style text report honouring the requested
/// `report_*` commands.
pub fn render(job: &StaJob, rep: &crate::sta::TimingReport, reports: &Reports) -> String {
    let mut s = String::new();
    s.push_str(&format!(
        "# vyges-sta-si {} — OpenSTA-compatible adapter (Vyges engine)\n",
        crate::VERSION
    ));
    s.push_str(&format!(
        "# design {}   clock {} @ {:.4} ns\n\n",
        job.design, job.clock_port, job.period_ns
    ));

    let want_max = reports.checks_max || reports.wns || reports.tns || reports.worst_slack;
    if reports.checks && want_max {
        s.push_str(&path_block(
            job,
            "max",
            rep.endpoints,
            &rep.worst_endpoint,
            rep.wns,
            &rep.worst_path,
        ));
    }
    if reports.checks && reports.checks_min {
        s.push_str(&path_block(
            job,
            "min",
            rep.hold_endpoints,
            &rep.worst_hold_endpoint,
            rep.whs,
            &rep.worst_hold_path,
        ));
    }
    // Scalars (OpenSTA `report_wns`/`report_tns`/`report_worst_slack` print the value).
    if reports.wns || reports.worst_slack {
        if reports.wns_min || reports.worst_slack_min {
            s.push_str(&format!("wns(min/hold) {}\n", fmt(rep.whs)));
        } else {
            s.push_str(&format!("wns {}\n", fmt(rep.wns)));
        }
    }
    if reports.tns {
        if reports.tns_min {
            s.push_str(&format!("tns(min/hold) {}\n", fmt(rep.ths)));
        } else {
            s.push_str(&format!("tns {}\n", fmt(rep.tns)));
        }
    }
    s
}

fn path_block(
    job: &StaJob,
    kind: &str,
    endpoints: usize,
    endpoint: &str,
    slack: f64,
    path: &[crate::sta::PathNode],
) -> String {
    let label = if kind == "max" {
        "max (Setup)"
    } else {
        "min (Hold)"
    };
    let mut s = String::new();
    s.push_str(&format!("Path Type: {label}\n"));
    if endpoints == 0 || path.is_empty() {
        s.push_str("  (no constrained paths)\n\n");
        return s;
    }
    let startpoint = path.first().map(|p| p.label.as_str()).unwrap_or("?");
    s.push_str(&format!("  Startpoint: {startpoint}\n"));
    s.push_str(&format!("  Endpoint:   {endpoint}\n"));
    s.push_str(&format!("  Path Group: {}\n", job.clock_port));
    s.push_str(&format!(
        "    {:>10}  {:>8}   {}\n",
        "Arrival", "Slew", "Node"
    ));
    for p in path {
        s.push_str(&format!(
            "    {:10.4}  {:8.4}   {}\n",
            p.arrival, p.slew, p.label
        ));
    }
    let met = if slack >= 0.0 { "MET" } else { "VIOLATED" };
    s.push_str(&format!("    slack {} ({})\n\n", fmt(slack), met));
    s
}

fn fmt(v: f64) -> String {
    if v.is_finite() {
        format!("{v:.4}")
    } else {
        "INF".to_string()
    }
}

// ---- tiny TCL-subset lexer (NOT a TCL interpreter) ----------------------

/// Split a script into raw command strings: joins `\`-continued lines, drops
/// full-line `#` comments, and separates commands on top-level `;`.
fn commands(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut buf = String::new();
    for line in text.lines() {
        let line = line.trim_end();
        if let Some(stripped) = line.strip_suffix('\\') {
            buf.push_str(stripped);
            buf.push(' ');
            continue;
        }
        buf.push_str(line);
        let cmd = buf.trim().to_string();
        buf.clear();
        if cmd.is_empty() || cmd.starts_with('#') {
            continue;
        }
        for part in split_semicolons(&cmd) {
            let p = part.trim();
            if !p.is_empty() && !p.starts_with('#') {
                out.push(p.to_string());
            }
        }
    }
    out
}

/// Split on `;` that are outside `{}`, `[]`, and `"…"`.
fn split_semicolons(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let (mut depth, mut q, mut start) = (0i32, false, 0usize);
    let b = s.as_bytes();
    for i in 0..b.len() {
        match b[i] {
            b'"' => q = !q,
            b'{' | b'[' if !q => depth += 1,
            b'}' | b']' if !q => depth -= 1,
            b';' if !q && depth <= 0 => {
                out.push(s[start..i].to_string());
                start = i + 1;
            }
            _ => {}
        }
    }
    out.push(s[start..].to_string());
    out
}

/// Whitespace-split a command, keeping `{…}`, `[…]` and `"…"` groups whole.
fn tokenize(cmd: &str) -> Vec<String> {
    let mut toks = Vec::new();
    let mut cur = String::new();
    let (mut depth, mut q) = (0i32, false);
    for c in cmd.chars() {
        match c {
            '"' => {
                q = !q;
                cur.push(c);
            }
            '{' | '[' if !q => {
                depth += 1;
                cur.push(c);
            }
            '}' | ']' if !q => {
                depth -= 1;
                cur.push(c);
            }
            c if c.is_whitespace() && depth <= 0 && !q => {
                if !cur.is_empty() {
                    toks.push(std::mem::take(&mut cur));
                }
            }
            c => cur.push(c),
        }
    }
    if !cur.is_empty() {
        toks.push(cur);
    }
    toks
}

/// First non-flag argument of a `read_*`/`link_design` command, unwrapped of any
/// surrounding `{}`/`""`. `-corner X` consumes its value; other flags are skipped.
fn first_file(toks: &[String]) -> Option<String> {
    let mut i = 1;
    while i < toks.len() {
        let t = &toks[i];
        if t.starts_with('-') {
            i += if t == "-corner" { 2 } else { 1 };
            continue;
        }
        return Some(unwrap(t));
    }
    None
}

fn unwrap(s: &str) -> String {
    let s = s.trim();
    let s = s
        .strip_prefix('{')
        .and_then(|x| x.strip_suffix('}'))
        .unwrap_or(s);
    let s = s
        .strip_prefix('"')
        .and_then(|x| x.strip_suffix('"'))
        .unwrap_or(s);
    s.trim().to_string()
}

fn has_flag(toks: &[String], flag: &str) -> bool {
    toks.iter().any(|t| t == flag)
}

/// `(min, max)` requested by `report_checks -path_delay {min|max|min_max}` (default max).
fn path_delay(toks: &[String]) -> (bool, bool) {
    if let Some(i) = toks.iter().position(|t| t == "-path_delay") {
        match toks.get(i + 1).map(|s| s.as_str()) {
            Some("min") => return (true, false),
            Some("min_max") => return (true, true),
            _ => return (false, true), // "max" (the default) or anything else
        }
    }
    (false, true) // OpenSTA default
}

fn resolve(base_dir: &str, rel: &str) -> String {
    if Path::new(rel).is_absolute() || base_dir.is_empty() {
        rel.to_string()
    } else {
        Path::new(base_dir).join(rel).to_string_lossy().into_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tokenizes_keeping_groups() {
        let t = tokenize("set_input_delay 0.1 -clock clk [get_ports a]");
        assert_eq!(
            t,
            ["set_input_delay", "0.1", "-clock", "clk", "[get_ports a]"]
        );
    }

    #[test]
    fn first_file_skips_corner_flag() {
        let t = tokenize("read_liberty -corner ss {cells.lib}");
        assert_eq!(first_file(&t).as_deref(), Some("cells.lib"));
    }

    #[test]
    fn comments_and_continuation() {
        let c = commands("# a comment\nread_verilog \\\n  top.v\nlink_design top ; report_wns");
        assert_eq!(c.len(), 3);
        assert_eq!(tokenize(&c[0]), ["read_verilog", "top.v"]); // continuation joined
        assert_eq!(c[1], "link_design top");
        assert_eq!(c[2], "report_wns");
    }

    #[test]
    fn path_delay_flags() {
        assert_eq!(path_delay(&tokenize("report_checks")), (false, true));
        assert_eq!(
            path_delay(&tokenize("report_checks -path_delay min")),
            (true, false)
        );
        assert_eq!(
            path_delay(&tokenize("report_checks -path_delay min_max")),
            (true, true)
        );
    }
}
