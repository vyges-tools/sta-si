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

/// One `report_*` command as the script gave it, in the script's order: what it asks for and
/// how its numbers are printed (`-digits`, default 2 as the reference's; `-fields`).
#[derive(Clone, Debug, PartialEq)]
pub enum ReportCmd {
    /// `report_checks` (full format): the worst path end of each path group
    Checks { min: bool, max: bool, fields: Fields, digits: usize },
    /// `report_wns`: the worst slack, or 0 when it is positive
    Wns { min: bool, digits: usize },
    /// `report_tns`: the sum of the negative slacks
    Tns { min: bool, digits: usize },
    /// `report_worst_slack`: the worst slack, signed
    WorstSlack { min: bool, digits: usize },
}

/// `report_checks -fields`: the optional columns (Delay, Time, edge and Description are always
/// there), and `input_pins`, which prints each load pin as well as each driver.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fields {
    pub cap: bool,
    pub slew: bool,
    pub input_pins: bool,
}

/// Which `report_*` commands the script asked for (and their min/max sense).
#[derive(Default, Clone)]
pub struct Reports {
    /// every report command, in the script's order — what [`render`] prints
    pub cmds: Vec<ReportCmd>,
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
        if self.cmds.is_empty() {
            self.cmds = vec![
                ReportCmd::Checks { min: false, max: true, fields: Fields::default(), digits: 2 },
                ReportCmd::Wns { min: false, digits: 2 },
                ReportCmd::Tns { min: false, digits: 2 },
            ];
        }
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
    let mut min_libs: Vec<String> = Vec::new();
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
                    // A `-min` library times the reference's min (hold) analysis only. One
                    // library set times both analyses here, so it is set aside — and SAID so —
                    // rather than merged in, where a cell it shares with the max library
                    // silently took whichever was read first.
                    if has_flag(&toks, "-min") && !has_flag(&toks, "-max") {
                        ignored.push(format!("read_liberty -min {f} (a min-analysis library: the max library times both analyses)"));
                        min_libs.push(f);
                    } else {
                        libs.push(f);
                    }
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
                let (fields, unknown) = report_fields(&toks);
                for u in unknown {
                    ignored.push(format!("report_checks -fields {u} (column not printed)"));
                }
                // options that choose OTHER paths than each group's worst are not modelled: say so
                for o in ["-from", "-to", "-through", "-rise_from", "-fall_from", "-rise_to", "-fall_to", "-unconstrained"] {
                    if has_flag(&toks, o) {
                        ignored.push(format!("report_checks {o} (the worst path of each group is reported)"));
                    }
                }
                for o in ["-group_path_count", "-group_count", "-endpoint_path_count", "-endpoint_count"] {
                    if opt_value(&toks, o).is_some_and(|v| v != "1") {
                        ignored.push(format!("report_checks {o} (one path per group is reported)"));
                    }
                }
                if opt_value(&toks, "-format").is_some_and(|f| f != "full") {
                    ignored.push("report_checks -format (the full format is printed)".to_string());
                }
                reports.cmds.push(ReportCmd::Checks { min: mn, max: mx, fields, digits: digits(&toks) });
            }
            "report_wns" => {
                reports.wns = true;
                reports.wns_min |= has_flag(&toks, "-min");
                reports.cmds.push(ReportCmd::Wns { min: has_flag(&toks, "-min"), digits: digits(&toks) });
            }
            "report_tns" => {
                reports.tns = true;
                reports.tns_min |= has_flag(&toks, "-min");
                reports.cmds.push(ReportCmd::Tns { min: has_flag(&toks, "-min"), digits: digits(&toks) });
            }
            "report_worst_slack" => {
                reports.worst_slack = true;
                reports.worst_slack_min |= has_flag(&toks, "-min");
                reports.cmds.push(ReportCmd::WorstSlack { min: has_flag(&toks, "-min"), digits: digits(&toks) });
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
        // only `-min` libraries: they are all there is to time with
        libs.append(&mut min_libs);
    }
    if libs.is_empty() {
        return Err(TclError("no `read_liberty <lib>`".into()));
    }

    let mut job = base_job(design, netlist, libs, spef, &base_dir);

    if !combined_sdc.trim().is_empty() {
        let sdc = Sdc::parse(&combined_sdc).map_err(|e| TclError(e.to_string()))?;
        crate::job::merge_sdc_into(&sdc, &mut job);
        // the reader lists what IT does not model; this timer models these
        for ig in sdc.ignored.iter().filter(|c| !crate::job::MODELLED_HERE.contains(&c.as_str())) {
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
        rc_model: "elmore".into(),
        // The script's SDC is the whole constraint set, as in the reference: an input with no
        // `set_input_delay` is UNCONSTRAINED and starts no path (the merge declares a default
        // when one is given). `true` here timed every such input as arriving at 0.
        input_delay_declared: false,
        design,
        netlist,
        libs,
        spef,
        clock_port: String::new(),
        period_ns: 0.0,
        clocks: Vec::new(),
        // A script sets these or they are 0, as in the reference: no `set_input_transition`, an
        // input starts from slew 0; no `set_load`, an output port loads its driver with nothing.
        // (The `.sta` format keeps its own documented defaults.)
        input_slew: 0.0,
        input_slews: Vec::new(),
        output_load: 0.0,
        input_delay: 0.0,
        output_delay: 0.0,
        output_delay_declared: false,
        unconstrained_outputs: true,
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
        clock_groups: vec![],
        ideal_clocks: Default::default(),
        driving_cells: Default::default(),
        sdc: None,
        metadata: None,
        base_dir: base_dir.to_string(),
    }
}

// ---- OpenSTA-flavoured report rendering ---------------------------------

/// Render the analysis as the reference's text reports, one block per `report_*` command in
/// the script's order: `report_checks` in its full format (the worst path end of each path
/// group, `ReportPath::reportFull`), `report_wns` / `report_tns` / `report_worst_slack` as one
/// line each.
pub fn render(_job: &StaJob, rep: &crate::sta::TimingReport, reports: &Reports) -> String {
    let mut s = String::new();
    for cmd in &reports.cmds {
        match *cmd {
            ReportCmd::Checks { min, max, fields, digits } => {
                // the reference walks min then max (`MinMax::range`)
                let ends: Vec<&crate::sta::CheckPath> =
                    rep.check_paths.iter().filter(|p| (p.max && max) || (!p.max && min)).collect();
                let ends: Vec<&crate::sta::CheckPath> =
                    ends.iter().filter(|p| !p.max).chain(ends.iter().filter(|p| p.max)).copied().collect();
                if ends.is_empty() {
                    s.push_str("No paths found.\n");
                }
                for p in ends {
                    path_end(&mut s, p, fields, digits);
                }
            }
            ReportCmd::Wns { min, digits } => {
                let w = if min { rep.whs } else { rep.wns };
                // "If the worst slack is positive, zero is reported."
                let w = if w > 0.0 || !w.is_finite() { 0.0 } else { w };
                s.push_str(&format!("wns {} {}\n", mm(min), time(w, digits)));
            }
            ReportCmd::Tns { min, digits } => {
                s.push_str(&format!("tns {} {}\n", mm(min), time(if min { rep.ths } else { rep.tns }, digits)));
            }
            ReportCmd::WorstSlack { min, digits } => {
                let w = if min { rep.whs } else { rep.wns };
                s.push_str(&format!("worst slack {} {}\n", mm(min), time(w, digits)));
            }
        }
    }
    s
}

fn mm(min: bool) -> &'static str {
    if min {
        "min"
    } else {
        "max"
    }
}

/// A time at `digits` decimals, `-0.00` printed as `0.00` (the reference filters it).
fn time(v: f64, digits: usize) -> String {
    let t = format!("{v:.digits$}");
    if t.strip_prefix('-').is_some_and(|r| r.chars().all(|c| c == '0' || c == '.')) {
        t[1..].to_string()
    } else {
        t
    }
}

/// The columns of a path report and how each is printed (`ReportPath::reportLine`): every
/// numeric column right-justified to `digits + 5`, the edge one character, the description
/// last; fields separated by one space, trailing spaces trimmed.
struct Cols {
    fields: Fields,
    digits: usize,
}

impl Cols {
    fn width(&self) -> usize {
        self.digits + 5
    }

    fn num(&self, line: &mut String, v: Option<String>) {
        let w = self.width();
        match v {
            Some(t) => line.push_str(&format!("{t:>w$}")),
            None => line.push_str(&" ".repeat(w)),
        }
    }

    /// One line: `cap`, `slew` (when those columns are on), the increment, the total (`minus`:
    /// always signed, as `data arrival time` is), the edge and the description.
    #[allow(clippy::too_many_arguments)]
    fn line(&self, what: &str, cap: Option<f64>, slew: Option<f64>, incr: Option<f64>, total: Option<f64>, minus: bool, rise: Option<bool>) -> String {
        let d = self.digits;
        let mut line = String::new();
        if self.fields.cap {
            self.num(&mut line, cap.map(|c| format!("{c:.d$}")));
            line.push(' ');
        }
        if self.fields.slew {
            self.num(&mut line, slew.map(|v| time(v, d)));
            line.push(' ');
        }
        self.num(&mut line, incr.map(|v| time(v, d)));
        line.push(' ');
        let total = total.map(|v| {
            if minus {
                // the negated value, a leading minus forced on zero
                let t = time(-v, d);
                if t.starts_with('-') {
                    t
                } else if t.chars().all(|c| c == '0' || c == '.') {
                    format!("-{t}")
                } else {
                    t
                }
            } else {
                time(v, d)
            }
        });
        self.num(&mut line, total);
        line.push(' ');
        line.push(match rise {
            Some(true) => '^',
            Some(false) => 'v',
            None => ' ',
        });
        line.push(' ');
        line.push_str(what);
        line.trim_end().to_string()
    }

    fn header(&self) -> String {
        let w = self.width();
        let mut line = String::new();
        if self.fields.cap {
            line.push_str(&format!("{:>w$} ", "Cap"));
        }
        if self.fields.slew {
            line.push_str(&format!("{:>w$} ", "Slew"));
        }
        line.push_str(&format!("{:>w$} {:>w$}   Description", "Delay", "Time"));
        line
    }

    fn dashes(&self) -> String {
        let n = self.width() * (2 + usize::from(self.fields.cap) + usize::from(self.fields.slew)) + 1 + 36 + 6;
        "-".repeat(n)
    }
}

/// `Startpoint: pt (reason)`, split over two lines past 80 columns as the reference splits it.
fn start_end(s: &mut String, key: &str, pt: &str, reason: &str) {
    if key.len() + 2 + pt.len() + 2 + reason.len() + 1 > 80 {
        s.push_str(&format!("{key}: {pt}\n{}  ({reason})\n", " ".repeat(key.len())));
    } else {
        s.push_str(&format!("{key}: {pt} ({reason})\n"));
    }
}

/// One path end in the reference's full format (`ReportPath::reportFull` for a check or an
/// output delay): header, the source clock and data path, the target clock and required time,
/// the slack; two blank lines after.
fn path_end(s: &mut String, p: &crate::sta::CheckPath, fields: Fields, digits: usize) {
    let c = Cols { fields, digits };
    let mut out = |l: String| {
        s.push_str(&l);
        s.push('\n');
    };
    let mut head = String::new();
    start_end(&mut head, "Startpoint", &p.startpoint, &p.start_reason);
    start_end(&mut head, "Endpoint", &p.endpoint, &p.end_reason);
    head.push_str(&format!("Path Group: {}\nPath Type: {}\n", p.group, mm(!p.max)));
    out(head.trim_end_matches('\n').to_string());
    out(String::new());
    out(c.header());
    out(c.dashes());
    // the launch clock, then its network delay (or the input's external delay)
    let clk_slew = |k: &crate::sta::ClockLine| (!k.propagated).then_some(k.slew);
    let ideal_prop = |k: &crate::sta::ClockLine| {
        if k.propagated {
            "clock network delay (propagated)"
        } else {
            "clock network delay (ideal)"
        }
    };
    let edge = |rise: bool| if rise { "rise" } else { "fall" };
    let src = &p.src_clock;
    out(c.line(&format!("clock {} ({} edge)", src.name, edge(src.rise)), None, clk_slew(src), Some(src.time), Some(src.time), false, None));
    out(c.line(ideal_prop(src), None, None, Some(p.src_network_delay), Some(src.time + p.src_network_delay), false, None));
    if let Some((d, rise)) = p.input_delay {
        out(c.line("input external delay", None, None, Some(d), p.lines.first().map(|l| l.time), false, Some(rise)));
    }
    // the data path: every driver, the first and last pin, the launching clock pin (and every
    // load pin with `-fields input_pins`); each increment from the last line printed
    let last = p.lines.len().saturating_sub(1);
    let mut prev = 0.0;
    for (i, l) in p.lines.iter().enumerate() {
        let driver = l.cap.is_some();
        if !(driver || i == 0 || i == last || l.clk_start || fields.input_pins) {
            continue;
        }
        let incr = if i == 0 { 0.0 } else { l.time - prev };
        let cap = if driver { l.cap } else { None };
        out(c.line(&l.desc, cap, Some(l.slew), Some(incr), Some(l.time), false, Some(l.rise)));
        prev = l.time;
    }
    out(c.line("data arrival time", None, None, None, Some(p.arrival), false, None));
    out(String::new());
    // the target clock and the required time
    let tgt = &p.tgt_clock;
    out(c.line(&format!("clock {} ({} edge)", tgt.name, edge(tgt.rise)), None, clk_slew(tgt), Some(tgt.time), Some(tgt.time), false, None));
    let mut t = tgt.time + p.tgt_network_delay;
    out(c.line(ideal_prop(tgt), None, None, Some(p.tgt_network_delay), Some(t), false, None));
    if p.uncertainty != 0.0 {
        t += p.uncertainty;
        out(c.line("clock uncertainty", None, None, Some(p.uncertainty), Some(t), false, None));
    }
    t += p.crpr;
    out(c.line("clock reconvergence pessimism", None, None, Some(p.crpr), Some(t), false, None));
    if let Some((pin, rise)) = &p.tgt_pin {
        out(c.line(pin, None, None, None, Some(tgt.time + p.tgt_network_delay), false, Some(*rise)));
    }
    out(c.line(&p.margin_what, None, None, Some(p.required - t), Some(p.required), false, None));
    out(c.line("data required time", None, None, None, Some(p.required), false, None));
    out(c.dashes());
    out(c.line("data required time", None, None, None, Some(p.required), false, None));
    out(c.line("data arrival time", None, None, None, Some(p.arrival), true, None));
    out(c.dashes());
    let met = if p.slack >= 0.0 { "slack (MET)" } else { "slack (VIOLATED)" };
    out(c.line(met, None, None, None, Some(p.slack), false, None));
    out(String::new());
    out(String::new());
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

/// The value after option `opt`, unwrapped of its braces.
fn opt_value(toks: &[String], opt: &str) -> Option<String> {
    toks.iter().position(|t| t == opt).and_then(|i| toks.get(i + 1)).map(|v| unwrap(v))
}

/// `-digits N` (the reference's default report digits are 2).
fn digits(toks: &[String]) -> usize {
    opt_value(toks, "-digits").and_then(|v| v.parse().ok()).unwrap_or(2)
}

/// `-fields {…}`: the columns asked for, and any not printed here. Names as the reference
/// matches them: `cap`/`capacitance`, `slew`, `input_pin`/`input_pins`/`input`.
fn report_fields(toks: &[String]) -> (Fields, Vec<String>) {
    let mut f = Fields::default();
    let mut unknown = Vec::new();
    if let Some(list) = opt_value(toks, "-fields") {
        for name in list.split_whitespace() {
            match name {
                "cap" | "capacitance" => f.cap = true,
                "slew" => f.slew = true,
                "input" | "input_pin" | "input_pins" => f.input_pins = true,
                other => unknown.push(other.to_string()),
            }
        }
    }
    (f, unknown)
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

    fn adapt_text(name: &str, body: &str) -> Adapted {
        let p = std::env::temp_dir().join(format!("vyg-tcl-{}-{name}.tcl", std::process::id()));
        std::fs::write(&p, body).unwrap();
        let a = adapt(p.to_str().unwrap()).unwrap();
        let _ = std::fs::remove_file(&p);
        a
    }

    const HEAD: &str = "read_liberty c.lib\nread_verilog t.v\nlink_design top\n";

    // Rule: `create_clock -name clk -period P {clk1 clk2 clk3}` is ONE clock whose every port is
    // a source (the reference's own reg1_asap7 test). Taking only the first left r2/r3 unclocked.
    #[test]
    fn one_clock_on_several_ports_clocks_each() {
        let a = adapt_text("multi", &format!("{HEAD}create_clock -name clk -period 500 {{clk1 clk2 clk3}}\n"));
        let srcs: Vec<&str> = a.job.clocks.iter().map(|(n, s, _)| { assert_eq!(n, "clk"); s.as_str() }).collect();
        assert_eq!(srcs, ["clk1", "clk2", "clk3"]);
    }

    // Rule: the script's SDC is the whole constraint set — an input with no set_input_delay is
    // unconstrained and starts no path; a default (`[all_inputs]`) or per-port delay declares it.
    #[test]
    fn an_input_without_input_delay_is_unconstrained() {
        let a = adapt_text("unc", &format!("{HEAD}create_clock -name clk -period 5 [get_ports clk]\n"));
        assert_eq!(a.job.input_delay_for("in1"), None);
        let b = adapt_text("dec", &format!("{HEAD}create_clock -name clk -period 5 [get_ports clk]\nset_input_delay -clock clk 0 {{in1 in2}}\n"));
        assert_eq!(b.job.input_delay_for("in1"), Some(0.0));
        assert_eq!(b.job.input_delay_for("in3"), None);
    }

    // Rule: each `set_input_transition` sets the ports it names, for the edges (`-rise`/`-fall`)
    // and analyses (`-min`/`-max`) it names, a later command replacing what it names; one
    // design-wide value let the last command set every input.
    #[test]
    fn input_transitions_are_per_port_edge_and_analysis() {
        let a = adapt_text(
            "slew",
            &format!(
                "{HEAD}create_clock -name clk -period 5 [get_ports clk]\nset_input_transition 0.1 [all_inputs]\n\
                 set_input_transition 0.02 [get_ports in1]\nset_input_transition -rise -max 0.4 [get_ports in2]\n"
            ),
        );
        assert_eq!(a.job.input_slew_for("in1", false), [[0.02; 2]; 2]);
        assert_eq!(a.job.input_slew_for("in2", false), [[0.1, 0.1], [0.4, 0.1]]);
        assert_eq!(a.job.input_slew_for("in3", false), [[0.1; 2]; 2]);
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
