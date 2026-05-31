//! Liberty (`.lib`) reader + NLDM bilinear interpolation.
//!
//! Reads the timing view the STA engine needs: per cell, each pin's direction
//! and input capacitance, and for each output-pin timing arc the four NLDM
//! tables (`cell_rise` / `cell_fall` / `rise_transition` / `fall_transition`).
//! `Table::lookup(slew, load)` does clamped bilinear interpolation over
//! (index_1 = input_net_transition, index_2 = total_output_net_capacitance).
//!
//! Tolerant of both the `vyges-char` emitter's form and foundry libs: cell and
//! template names may be quoted or bare. Pure std — fully unit-tested offline.

use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    In,
    Out,
    Other,
}

#[derive(Debug, Clone, Default)]
pub struct Table {
    pub index_1: Vec<f64>, // input slews
    pub index_2: Vec<f64>, // output loads
    pub values: Vec<Vec<f64>>, // values[i][j] over (slew_i, load_j)
}

#[derive(Debug, Clone)]
pub struct Arc {
    pub related_pin: String,
    pub sense: String,
    pub cell_rise: Table,
    pub cell_fall: Table,
    pub rise_transition: Table,
    pub fall_transition: Table,
}

#[derive(Debug, Clone)]
pub struct Pin {
    pub name: String,
    pub direction: Dir,
    pub capacitance: f64,
    pub clock: bool,         // `clock : true` — the cell's clock pin
    pub setup: Option<f64>,  // setup constraint (ns) on a data pin, vs the clock
    pub hold: Option<f64>,   // hold constraint (ns) on a data pin, vs the clock
    pub arcs: Vec<Arc>,      // delay arcs (e.g. CK->Q on a flop output)
}

#[derive(Debug, Clone)]
pub struct Cell {
    pub name: String,
    pub pins: BTreeMap<String, Pin>,
    pub is_seq: bool,                // has an `ff`/`latch` group
    pub clock_pin: Option<String>,   // the pin marked `clock : true`
}

#[derive(Debug, Clone, Default)]
pub struct Lib {
    pub cells: BTreeMap<String, Cell>,
}

#[derive(Debug)]
pub struct LibError(pub String);
impl std::fmt::Display for LibError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "liberty error: {}", self.0)
    }
}
impl std::error::Error for LibError {}

impl Table {
    /// Clamped bilinear interpolation; edge-clamps rather than extrapolating.
    pub fn lookup(&self, slew: f64, load: f64) -> f64 {
        if self.values.is_empty() {
            return 0.0;
        }
        if self.index_1.is_empty() || self.index_2.is_empty() {
            return self.values[0][0];
        }
        let (i0, i1, tx) = bracket(&self.index_1, slew);
        let (j0, j1, ty) = bracket(&self.index_2, load);
        let v = |i: usize, j: usize| self.values[i][j];
        let a = v(i0, j0) * (1.0 - tx) + v(i1, j0) * tx;
        let b = v(i0, j1) * (1.0 - tx) + v(i1, j1) * tx;
        a * (1.0 - ty) + b * ty
    }
}

/// Return (lo, hi, frac) bracketing `v` in ascending grid `g`; clamps at edges.
fn bracket(g: &[f64], v: f64) -> (usize, usize, f64) {
    let n = g.len();
    if n == 1 {
        return (0, 0, 0.0);
    }
    if v <= g[0] {
        return (0, 1, 0.0);
    }
    if v >= g[n - 1] {
        return (n - 2, n - 1, 1.0);
    }
    for k in 0..n - 1 {
        if v <= g[k + 1] {
            let t = (v - g[k]) / (g[k + 1] - g[k]);
            return (k, k + 1, t);
        }
    }
    (n - 2, n - 1, 1.0)
}

// ---- parser ---------------------------------------------------------------

fn matching(b: &[u8], mut i: usize) -> usize {
    let mut depth = 0i32;
    while i < b.len() {
        match b[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return i;
                }
            }
            _ => {}
        }
        i += 1;
    }
    b.len()
}

fn is_ident(c: u8) -> bool {
    c.is_ascii_alphanumeric() || c == b'_'
}

/// Next `kw ( args ) { body }` at/after `from`. Returns (args, body, after_idx).
fn next_block(s: &str, from: usize, kw: &str) -> Option<(String, String, usize)> {
    let b = s.as_bytes();
    let mut p = from;
    loop {
        let hit = s[p..].find(kw)? + p;
        // token boundary before kw
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + kw.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b'(' {
            let close_paren = s[q..].find(')')? + q;
            let args = s[q + 1..close_paren].trim().trim_matches('"').to_string();
            let mut r = close_paren + 1;
            while r < b.len() && b[r].is_ascii_whitespace() {
                r += 1;
            }
            if r < b.len() && b[r] == b'{' {
                let end = matching(b, r);
                return Some((args, s[r + 1..end].to_string(), end + 1));
            }
        }
        p = hit + kw.len();
    }
}

fn simple_attr(body: &str, key: &str) -> Option<String> {
    // matches `key : value ;`
    let b = body.as_bytes();
    let mut p = 0;
    loop {
        let hit = body[p..].find(key)? + p;
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + key.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b':' {
            let semi = body[q..].find(';')? + q;
            return Some(body[q + 1..semi].trim().trim_matches('"').to_string());
        }
        p = hit + key.len();
    }
}

fn floats(s: &str) -> Vec<f64> {
    s.split(',').filter_map(|t| t.trim().parse::<f64>().ok()).collect()
}

fn parse_table(body: &str) -> Table {
    // index_1/index_2 use paren+quote form: `index_1 ("0.01, 0.04");`
    let idx = |kw: &str| {
        next_paren_after(body, kw).map(|s| floats(&s.replace('"', ""))).unwrap_or_default()
    };
    let index_1 = idx("index_1");
    let index_2 = idx("index_2");
    // values ( "a, b", "c, d" ) — collect each quoted row
    let values = next_paren_after(body, "values")
        .map(|v| {
            let mut rows = Vec::new();
            let mut rest = v.as_str();
            while let Some(start) = rest.find('"') {
                let after = &rest[start + 1..];
                if let Some(endq) = after.find('"') {
                    rows.push(floats(&after[..endq]));
                    rest = &after[endq + 1..];
                } else {
                    break;
                }
            }
            rows
        })
        .unwrap_or_default();
    Table { index_1, index_2, values }
}

/// Content of the `( ... )` following `kw` (paren-matched), e.g. `values ( ... )`.
fn next_paren_after(s: &str, kw: &str) -> Option<String> {
    let b = s.as_bytes();
    let mut p = 0;
    loop {
        let hit = s[p..].find(kw)? + p;
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + kw.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b'(' {
            // paren-match
            let mut depth = 0i32;
            let mut r = q;
            while r < b.len() {
                match b[r] {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth == 0 {
                            return Some(s[q + 1..r].to_string());
                        }
                    }
                    _ => {}
                }
                r += 1;
            }
            return None;
        }
        p = hit + kw.len();
    }
}

fn parse_arc(timing_body: &str) -> Arc {
    let tbl = |name: &str| {
        next_block(timing_body, 0, name).map(|(_, body, _)| parse_table(&body)).unwrap_or_default()
    };
    Arc {
        related_pin: simple_attr(timing_body, "related_pin").unwrap_or_default(),
        sense: simple_attr(timing_body, "timing_sense").unwrap_or_else(|| "non_unate".into()),
        cell_rise: tbl("cell_rise"),
        cell_fall: tbl("cell_fall"),
        rise_transition: tbl("rise_transition"),
        fall_transition: tbl("fall_transition"),
    }
}

/// Largest value in a constraint timing group (rise/fall_constraint) — a
/// representative setup/hold number (interpolating at the real slews is later).
fn constraint_max(timing_body: &str) -> f64 {
    let mut m = 0.0f64;
    for name in ["rise_constraint", "fall_constraint"] {
        if let Some((_, tb, _)) = next_block(timing_body, 0, name) {
            for row in &parse_table(&tb).values {
                for &v in row {
                    if v > m {
                        m = v;
                    }
                }
            }
        }
    }
    m
}

fn parse_pin(name: String, body: &str) -> Pin {
    let direction = match simple_attr(body, "direction").as_deref() {
        Some("input") => Dir::In,
        Some("output") => Dir::Out,
        _ => Dir::Other,
    };
    let capacitance =
        simple_attr(body, "capacitance").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let clock = simple_attr(body, "clock").as_deref() == Some("true");
    let mut arcs = Vec::new();
    let mut setup: Option<f64> = None;
    let mut hold: Option<f64> = None;
    let mut at = 0;
    while let Some((_, tbody, after)) = next_block(body, at, "timing") {
        match simple_attr(&tbody, "timing_type").as_deref() {
            Some(tt) if tt.starts_with("setup") => {
                let v = constraint_max(&tbody);
                setup = Some(setup.map_or(v, |s| s.max(v)));
            }
            Some(tt) if tt.starts_with("hold") => {
                let v = constraint_max(&tbody); // ns the data must stay stable post-edge
                hold = Some(hold.map_or(v, |h| h.max(v)));
            }
            _ => arcs.push(parse_arc(&tbody)), // delay arc (incl. rising_edge CK->Q)
        }
        at = after;
    }
    Pin { name, direction, capacitance, clock, setup, hold, arcs }
}

fn parse_cell(name: String, body: &str) -> Cell {
    let mut pins = BTreeMap::new();
    let mut at = 0;
    while let Some((pname, pbody, after)) = next_block(body, at, "pin") {
        let pin = parse_pin(pname.clone(), &pbody);
        pins.insert(pname, pin);
        at = after;
    }
    let is_seq = next_block(body, 0, "ff").is_some() || next_block(body, 0, "latch").is_some();
    let clock_pin = pins.iter().find(|(_, p)| p.clock).map(|(n, _)| n.clone());
    Cell { name, pins, is_seq, clock_pin }
}

impl Lib {
    pub fn parse(text: &str) -> Result<Lib, LibError> {
        let mut cells = BTreeMap::new();
        let mut at = 0;
        while let Some((cname, cbody, after)) = next_block(text, at, "cell") {
            cells.insert(cname.clone(), parse_cell(cname, &cbody));
            at = after;
        }
        if cells.is_empty() {
            return Err(LibError("no cells found".into()));
        }
        Ok(Lib { cells })
    }

    pub fn load(path: &str) -> Result<Lib, LibError> {
        let text = std::fs::read_to_string(path).map_err(|e| LibError(format!("{path}: {e}")))?;
        Lib::parse(&text)
    }

    pub fn cell(&self, name: &str) -> Option<&Cell> {
        self.cells.get(name)
    }
}
