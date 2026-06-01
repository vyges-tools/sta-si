//! SDC (Synopsys Design Constraints) reader — the standard constraint format.
//!
//! Real flows (synthesis, OpenROAD/LibreLane) emit `.sdc`, not our bespoke
//! `.sta` constraint lines. This module parses the sign-off-relevant subset of
//! SDC (which is Tcl) and **merges it onto a [`StaJob`]**, so a job can say
//! `sdc: top.sdc` and pick up clocks, I/O timing, uncertainty, derates, and
//! timing exceptions exactly as the tool that wrote them intended. The netlist,
//! libraries, and SPEF still come from the `.sta` job (they are not in SDC).
//!
//! Supported commands (others are collected in [`Sdc::ignored`], never fatal):
//!
//! - `create_clock -name N -period P {obj}` and
//!   `create_generated_clock -name N -source S -divide_by D -multiply_by M {obj}`
//! - `set_input_delay` / `set_output_delay` (default via `all_inputs`/`all_outputs`
//!   or `-clock`, plus per-port overrides) — the I/O timing budget
//! - `set_clock_uncertainty [-setup|-hold]` — setup/hold guard band
//! - `set_clock_latency` — captured (source latency applied to the I/O budget)
//! - `set_input_transition`, `set_load` — boundary slew / load
//! - `set_timing_derate -late|-early` — OCV derate
//! - `set_false_path` / `set_multicycle_path` — timing exceptions
//! - `set_units` — time/capacitance scaling to the engine's ns/pF
//!
//! The parser is std-only: it joins `\`-continuations, strips `#` comments,
//! resolves `set var`/`$var`, and understands `{...}` groups, `[get_* ...]`
//! accessors, and `[all_inputs]`/`[all_outputs]`.

use crate::job::{ExcKind, Exception, StaJob};
use std::collections::HashMap;

/// A parsed clock (regular or fully-resolved generated).
#[derive(Debug, Clone)]
pub struct SdcClock {
    pub name: String,
    pub source: String, // port name or inst/pin
    pub period: f64,    // ns
}

/// One `set_input_delay`/`set_output_delay`: a value plus its target. `default`
/// means it came from `all_inputs`/`all_outputs` (or a bare `-clock`).
#[derive(Debug, Clone)]
pub struct IoDelay {
    pub value: f64, // ns
    pub default: bool,
    pub ports: Vec<String>,
}

#[derive(Debug, Default)]
pub struct Sdc {
    pub clocks: Vec<SdcClock>,
    pub input_delays: Vec<IoDelay>,
    pub output_delays: Vec<IoDelay>,
    pub setup_uncertainty: f64,
    pub hold_uncertainty: f64,
    pub clock_latency: f64, // source/network latency (ns), applied to the I/O budget
    pub input_transition: Option<f64>,
    pub load: Option<f64>,
    pub late_derate: Option<f64>,
    pub early_derate: Option<f64>,
    pub exceptions: Vec<Exception>,
    pub ignored: Vec<String>, // commands we recognised but do not model
}

#[derive(Debug)]
pub struct SdcError(pub String);
impl std::fmt::Display for SdcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sdc error: {}", self.0)
    }
}
impl std::error::Error for SdcError {}

// ---- Tcl-subset lexing ---------------------------------------------------

/// Join `\`-continuations and split into logical command lines, dropping `#`
/// comments. A `#` starts a comment only at the beginning of a command (line
/// start, after whitespace) or after a `;`.
fn logical_lines(text: &str) -> Vec<String> {
    let mut joined = String::new();
    for raw in text.lines() {
        let line = raw.strip_suffix('\r').unwrap_or(raw);
        if let Some(stripped) = line.strip_suffix('\\') {
            joined.push_str(stripped);
            joined.push(' ');
        } else {
            joined.push_str(line);
            joined.push('\n');
        }
    }
    let mut out = Vec::new();
    for seg in joined.split(['\n', ';']) {
        let t = seg.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        out.push(t.to_string());
    }
    out
}

/// Split one command into tokens, keeping `{...}` and `[...]` groups whole
/// (nesting-aware) and respecting `"..."`. Braces are stripped from the token;
/// brackets are kept so accessors can be post-processed.
fn tokenize(line: &str) -> Vec<String> {
    let mut toks = Vec::new();
    let cs: Vec<char> = line.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        let c = cs[i];
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if c == '#' {
            break; // trailing comment
        }
        if c == '{' {
            let mut depth = 1;
            let mut s = String::new();
            i += 1;
            while i < cs.len() && depth > 0 {
                match cs[i] {
                    '{' => depth += 1,
                    '}' => {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    _ => {}
                }
                s.push(cs[i]);
                i += 1;
            }
            i += 1; // past '}'
            toks.push(s);
        } else if c == '[' {
            let mut depth = 1;
            let mut s = String::from("[");
            i += 1;
            while i < cs.len() && depth > 0 {
                match cs[i] {
                    '[' => depth += 1,
                    ']' => depth -= 1,
                    _ => {}
                }
                s.push(cs[i]);
                i += 1;
            }
            toks.push(s);
        } else if c == '"' {
            let mut s = String::new();
            i += 1;
            while i < cs.len() && cs[i] != '"' {
                s.push(cs[i]);
                i += 1;
            }
            i += 1;
            toks.push(s);
        } else {
            let mut s = String::new();
            while i < cs.len() && !cs[i].is_whitespace() && cs[i] != '[' && cs[i] != '{' {
                s.push(cs[i]);
                i += 1;
            }
            toks.push(s);
        }
    }
    toks
}

/// Resolve a token to a list of object names. Handles `[get_ports {a b}]`,
/// `[get_pins x/y]`, `[get_clocks clk]`, `[all_inputs]`, `[all_outputs]`, a
/// brace list, or a bare name. Returns the sentinel `*INPUTS*` / `*OUTPUTS*`
/// for the `all_*` accessors so the caller can expand against the netlist.
fn resolve_objs(tok: &str) -> Vec<String> {
    if let Some(inner) = tok.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        let inner = inner.trim();
        let parts = tokenize(inner);
        if parts.is_empty() {
            return Vec::new();
        }
        match parts[0].as_str() {
            "all_inputs" => return vec!["*INPUTS*".into()],
            "all_outputs" => return vec!["*OUTPUTS*".into()],
            "all_registers" => return vec!["*REGS*".into()],
            "get_ports" | "get_pins" | "get_clocks" | "get_nets" | "get_cells" => {
                let mut names = Vec::new();
                for p in &parts[1..] {
                    if p.starts_with('-') {
                        continue; // e.g. -hierarchical
                    }
                    names.extend(p.split_whitespace().map(|s| s.to_string()));
                }
                return names;
            }
            _ => return vec![parts[0].clone()],
        }
    }
    // brace list or bare name -> split on whitespace
    tok.split_whitespace().map(|s| s.to_string()).collect()
}

/// Apply `set var value` substitution to a logical line (`$var`, `${var}`).
fn subst_vars(line: &str, vars: &HashMap<String, String>) -> String {
    if !line.contains('$') {
        return line.to_string();
    }
    let mut out = String::new();
    let cs: Vec<char> = line.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        if cs[i] == '$' {
            let braced = i + 1 < cs.len() && cs[i + 1] == '{';
            let mut j = if braced { i + 2 } else { i + 1 };
            let start = j;
            while j < cs.len() {
                let c = cs[j];
                if braced {
                    if c == '}' {
                        break;
                    }
                } else if !(c.is_alphanumeric() || c == '_') {
                    break;
                }
                j += 1;
            }
            let name: String = cs[start..j].iter().collect();
            if let Some(val) = vars.get(&name) {
                out.push_str(val);
            }
            i = if braced { j + 1 } else { j };
        } else {
            out.push(cs[i]);
            i += 1;
        }
    }
    out
}

/// Pull a flag's value: `-flag value`. Returns the token after the flag.
fn flag_val<'a>(toks: &'a [String], flag: &str) -> Option<&'a String> {
    toks.iter().position(|t| t == flag).and_then(|p| toks.get(p + 1))
}

fn has_flag(toks: &[String], flag: &str) -> bool {
    toks.iter().any(|t| t == flag)
}

/// The trailing positional object token (last token that is not a flag or a
/// flag's value), e.g. the `{obj}` of `create_clock ... {obj}`.
fn trailing_obj(toks: &[String], valued_flags: &[&str]) -> Option<String> {
    let mut skip = false;
    let mut last = None;
    for (k, t) in toks.iter().enumerate().skip(1) {
        if skip {
            skip = false;
            continue;
        }
        if t.starts_with('-') {
            if valued_flags.contains(&t.as_str()) {
                skip = true;
            }
            continue;
        }
        let _ = k;
        last = Some(t.clone());
    }
    last
}

// ---- unit scaling --------------------------------------------------------

/// Parse a `set_units` magnitude like `1ns`, `10ps`, `1pF`, `1ff` into a scale
/// to the engine's base (ns for time, pF for cap). Returns multiplier.
fn unit_scale(spec: &str, time: bool) -> Option<f64> {
    let s = spec.trim().to_lowercase();
    let (num, unit): (String, String) =
        s.chars().partition(|c| c.is_ascii_digit() || *c == '.' || *c == 'e' || *c == '-' || *c == '+');
    let mag: f64 = if num.is_empty() { 1.0 } else { num.parse().ok()? };
    let base = if time {
        match unit.as_str() {
            "s" => 1e9,
            "ms" => 1e6,
            "us" => 1e3,
            "ns" => 1.0,
            "ps" => 1e-3,
            "fs" => 1e-6,
            _ => return None,
        }
    } else {
        match unit.as_str() {
            "f" => 1e6,
            "pf" => 1.0,
            "ff" => 1e-3,
            "nf" => 1e3,
            "uf" => 1e6,
            _ => return None,
        }
    };
    Some(mag * base)
}

// ---- parse ---------------------------------------------------------------

impl Sdc {
    pub fn parse(text: &str) -> Result<Sdc, SdcError> {
        let mut sdc = Sdc::default();
        let mut vars: HashMap<String, String> = HashMap::new();
        let mut t_scale = 1.0; // -> ns
        let mut c_scale = 1.0; // -> pF
        // (name, source, divide, multiply) for generated clocks, resolved last.
        let mut gen: Vec<(String, String, f64, f64)> = Vec::new();

        for line in logical_lines(text) {
            let line = subst_vars(&line, &vars);
            let toks = tokenize(&line);
            if toks.is_empty() {
                continue;
            }
            match toks[0].as_str() {
                "set" => {
                    if toks.len() >= 3 {
                        vars.insert(toks[1].clone(), toks[2].clone());
                    }
                }
                "set_units" => {
                    if let Some(v) = flag_val(&toks, "-time") {
                        if let Some(s) = unit_scale(v, true) {
                            t_scale = s;
                        }
                    }
                    if let Some(v) = flag_val(&toks, "-capacitance") {
                        if let Some(s) = unit_scale(v, false) {
                            c_scale = s;
                        }
                    }
                }
                "create_clock" => {
                    let period: f64 = flag_val(&toks, "-period")
                        .and_then(|v| v.parse().ok())
                        .ok_or_else(|| SdcError("create_clock without -period".into()))?;
                    let obj = trailing_obj(&toks, &["-name", "-period", "-waveform", "-comment"]);
                    let source = obj
                        .as_deref()
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
                    let name = flag_val(&toks, "-name")
                        .cloned()
                        .unwrap_or_else(|| if source.is_empty() { "clk".into() } else { source.clone() });
                    let src = if source.is_empty() { name.clone() } else { source };
                    sdc.clocks.push(SdcClock { name, source: src, period: period * t_scale });
                }
                "create_generated_clock" => {
                    let obj = trailing_obj(
                        &toks,
                        &["-name", "-source", "-divide_by", "-multiply_by", "-edges", "-comment", "-master_clock"],
                    );
                    let target = obj
                        .as_deref()
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
                    let name = flag_val(&toks, "-name").cloned().unwrap_or_else(|| target.clone());
                    let source = flag_val(&toks, "-source")
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
                    let div: f64 = flag_val(&toks, "-divide_by").and_then(|v| v.parse().ok()).unwrap_or(1.0);
                    let mul: f64 = flag_val(&toks, "-multiply_by").and_then(|v| v.parse().ok()).unwrap_or(1.0);
                    let tgt = if target.is_empty() { name.clone() } else { target };
                    gen.push((name, tgt, div.max(1.0), mul.max(1.0)));
                    let _ = source;
                }
                "set_input_delay" | "set_output_delay" => {
                    let val: f64 = toks
                        .get(1)
                        .filter(|t| !t.starts_with('-'))
                        .or_else(|| flag_val(&toks, "-max"))
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0);
                    let obj = trailing_obj(&toks, &["-clock", "-max", "-min", "-reference_pin", "-comment"]);
                    let objs = obj.as_deref().map(resolve_objs).unwrap_or_default();
                    let default = objs.is_empty()
                        || objs.iter().any(|o| o == "*INPUTS*" || o == "*OUTPUTS*");
                    let ports: Vec<String> =
                        objs.into_iter().filter(|o| !o.starts_with('*')).collect();
                    let d = IoDelay { value: val * t_scale, default, ports };
                    if toks[0] == "set_input_delay" {
                        sdc.input_delays.push(d);
                    } else {
                        sdc.output_delays.push(d);
                    }
                }
                "set_clock_uncertainty" => {
                    let val: f64 = toks
                        .iter()
                        .skip(1)
                        .find(|t| !t.starts_with('-') && !t.starts_with('[') && t.parse::<f64>().is_ok())
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0);
                    let v = val * t_scale;
                    let setup = has_flag(&toks, "-setup");
                    let hold = has_flag(&toks, "-hold");
                    if setup || !hold {
                        sdc.setup_uncertainty = sdc.setup_uncertainty.max(v);
                    }
                    if hold || !setup {
                        sdc.hold_uncertainty = sdc.hold_uncertainty.max(v);
                    }
                }
                "set_clock_latency" => {
                    if let Some(v) = toks.iter().skip(1).find_map(|t| {
                        if t.starts_with('-') || t.starts_with('[') {
                            None
                        } else {
                            t.parse::<f64>().ok()
                        }
                    }) {
                        sdc.clock_latency = sdc.clock_latency.max(v * t_scale);
                    }
                }
                "set_input_transition" => {
                    if let Some(v) = toks.get(1).and_then(|t| t.parse::<f64>().ok()) {
                        sdc.input_transition = Some(v * t_scale);
                    }
                }
                "set_load" => {
                    let v = toks
                        .iter()
                        .skip(1)
                        .find_map(|t| if t.starts_with('-') { None } else { t.parse::<f64>().ok() });
                    if let Some(v) = v {
                        sdc.load = Some(v * c_scale);
                    }
                }
                "set_timing_derate" => {
                    if let Some(v) = flag_val(&toks, "-late").and_then(|v| v.parse().ok()) {
                        sdc.late_derate = Some(v);
                    }
                    if let Some(v) = flag_val(&toks, "-early").and_then(|v| v.parse().ok()) {
                        sdc.early_derate = Some(v);
                    }
                }
                "set_false_path" => {
                    let (from, to) = from_to(&toks);
                    sdc.exceptions.push(Exception { kind: ExcKind::FalsePath, from, to });
                }
                "set_multicycle_path" => {
                    let n: u32 = toks
                        .get(1)
                        .filter(|t| !t.starts_with('-'))
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1);
                    let (from, to) = from_to(&toks);
                    sdc.exceptions.push(Exception { kind: ExcKind::Multicycle(n), from, to });
                }
                other => sdc.ignored.push(other.to_string()),
            }
        }

        // resolve generated clocks against their master period.
        for (name, target, div, mul) in gen {
            let master = sdc.clocks.first().map(|c| c.period).unwrap_or(0.0);
            let period = master * div / mul;
            sdc.clocks.push(SdcClock { name, source: target, period });
        }
        Ok(sdc)
    }

    pub fn load(path: &str) -> Result<Sdc, SdcError> {
        let text = std::fs::read_to_string(path).map_err(|e| SdcError(format!("{path}: {e}")))?;
        Sdc::parse(&text)
    }

    /// Merge the SDC constraints onto a job. The job retains its design /
    /// netlist / lib / spef; SDC supplies the timing intent. Explicit `.sta`
    /// values are kept where SDC is silent.
    pub fn merge_into(&self, job: &mut StaJob) {
        // clocks: SDC is authoritative when present.
        if !self.clocks.is_empty() {
            job.clocks =
                self.clocks.iter().map(|c| (c.name.clone(), c.source.clone(), c.period)).collect();
            job.clock_port = job.clocks[0].1.clone();
            job.period_ns = job.clocks[0].2;
        }
        // I/O timing: default + per-port. Source latency adds to the I/O budget
        // (an input arrives `latency` later; an output must settle `latency`
        // earlier), matching propagated-clock intent on the boundary.
        let mut in_def = None;
        for d in &self.input_delays {
            if d.default {
                in_def = Some(d.value);
            }
            for p in &d.ports {
                job.io_input_delays.push((p.clone(), d.value + self.clock_latency));
            }
        }
        if let Some(v) = in_def {
            job.input_delay = v + self.clock_latency;
        }
        let mut out_def = None;
        for d in &self.output_delays {
            if d.default {
                out_def = Some(d.value);
            }
            for p in &d.ports {
                job.io_output_delays.push((p.clone(), d.value + self.clock_latency));
            }
        }
        if let Some(v) = out_def {
            job.output_delay = v + self.clock_latency;
        }
        job.setup_uncertainty = self.setup_uncertainty;
        job.hold_uncertainty = self.hold_uncertainty;
        if let Some(v) = self.input_transition {
            job.input_slew = v;
        }
        if let Some(v) = self.load {
            job.output_load = v;
        }
        if let Some(v) = self.late_derate {
            job.late_derate = v;
        }
        if let Some(v) = self.early_derate {
            job.early_derate = v;
        }
        job.exceptions.extend(self.exceptions.iter().cloned());
    }
}

/// Extract `-from`/`-to` object names (first object of each), `*` if absent.
/// A pin object (`reg/Q`) is reduced to its instance (`reg`) so it matches the
/// engine's instance-level exception matching; a port keeps its name.
fn from_to(toks: &[String]) -> (String, String) {
    let pick = |flags: &[&str]| -> String {
        for f in flags {
            if let Some(v) = flag_val(toks, f) {
                if let Some(name) = resolve_objs(v).into_iter().find(|o| !o.starts_with('*')) {
                    return match name.rsplit_once('/') {
                        Some((inst, _pin)) => inst.to_string(),
                        None => name,
                    };
                }
            }
        }
        "*".to_string()
    };
    (pick(&["-from", "-rise_from", "-fall_from"]), pick(&["-to", "-rise_to", "-fall_to"]))
}
