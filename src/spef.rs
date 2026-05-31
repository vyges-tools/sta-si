//! Minimal SPEF reader — per-net total capacitance + resistance for STA.
//!
//! STA needs two things from the parasitics: the **net capacitance** that loads
//! the driver, and the **interconnect delay** to the sinks. v1 reads the
//! per-net total cap (the `*D_NET` value) and the summed `*RES`, enough for a
//! lumped Elmore net delay (`R·C`) and a wire-cap load adder. Units are assumed
//! fF / Ω (what `vyges-extract` emits); the name map + `*D_NET … *END` records
//! are parsed, the detailed `*CAP`/`*CONN` topology is not (yet).
//!
//! Pure std — fully unit-tested offline.

use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, Default)]
pub struct NetRc {
    pub cap_ff: f64,      // total cap (grounded + coupling), from *D_NET
    pub res_ohm: f64,     // summed *RES
    pub coupling_ff: f64, // coupling cap to neighbours (for crosstalk delta)
}

#[derive(Debug, Clone, Default)]
pub struct Spef {
    pub nets: BTreeMap<String, NetRc>,
}

#[derive(Debug)]
pub struct SpefError(pub String);
impl std::fmt::Display for SpefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "spef error: {}", self.0)
    }
}
impl std::error::Error for SpefError {}

impl Spef {
    pub fn parse(text: &str) -> Spef {
        let mut names: BTreeMap<usize, String> = BTreeMap::new();
        let mut nets: BTreeMap<String, NetRc> = BTreeMap::new();
        let mut coupling: BTreeMap<String, f64> = BTreeMap::new(); // per-net coupling total
        let mut cur: Option<(String, NetRc)> = None;
        let mut in_namemap = false;
        let mut in_res = false;
        let mut in_cap = false;

        let finish = |cur: &mut Option<(String, NetRc)>, nets: &mut BTreeMap<String, NetRc>| {
            if let Some((name, rc)) = cur.take() {
                if !name.is_empty() {
                    nets.insert(name, rc);
                }
            }
        };
        let netname = |tok: &str, names: &BTreeMap<usize, String>| -> Option<String> {
            // a net node ref like `*2` -> net name; pin refs `*3:Y` are not nets
            let body = tok.trim_start_matches('*');
            if body.contains(':') {
                return None;
            }
            body.parse::<usize>().ok().and_then(|i| names.get(&i).cloned())
        };

        for raw in text.lines() {
            let t = raw.trim();
            if t == "*NAME_MAP" {
                in_namemap = true;
                continue;
            }
            if t.starts_with("*D_NET") {
                in_namemap = false;
                in_res = false;
                in_cap = false;
                finish(&mut cur, &mut nets);
                let toks: Vec<&str> = t.split_whitespace().collect();
                let id = toks.get(1).and_then(|s| s.trim_start_matches('*').parse::<usize>().ok());
                let cap = toks.get(2).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                let name = id.and_then(|i| names.get(&i).cloned()).unwrap_or_default();
                cur = Some((name, NetRc { cap_ff: cap, res_ohm: 0.0, coupling_ff: 0.0 }));
                continue;
            }
            match t {
                "*RES" => {
                    in_res = true;
                    in_cap = false;
                    continue;
                }
                "*CAP" => {
                    in_cap = true;
                    in_res = false;
                    continue;
                }
                "*CONN" => {
                    in_res = false;
                    in_cap = false;
                    continue;
                }
                "*END" => {
                    in_res = false;
                    in_cap = false;
                    finish(&mut cur, &mut nets);
                    continue;
                }
                _ => {}
            }
            if in_namemap && t.starts_with('*') {
                let toks: Vec<&str> = t.split_whitespace().collect();
                if let (Some(idtok), Some(name)) = (toks.first(), toks.get(1)) {
                    if let Ok(id) = idtok.trim_start_matches('*').parse::<usize>() {
                        names.insert(id, name.to_string());
                    }
                }
            } else if in_res {
                // `<idx> *a *b <ohm>` — accumulate the trailing resistance value
                if let Some(ohm) = t.split_whitespace().last().and_then(|s| s.parse::<f64>().ok()) {
                    if let Some((_, rc)) = cur.as_mut() {
                        rc.res_ohm += ohm;
                    }
                }
            } else if in_cap {
                // two-node entry `<idx> *A *B <ff>` is a coupling cap between nets
                let toks: Vec<&str> = t.split_whitespace().collect();
                if toks.len() >= 4 && toks[1].starts_with('*') && toks[2].starts_with('*') {
                    if let (Some(a), Some(b), Ok(v)) =
                        (netname(toks[1], &names), netname(toks[2], &names), toks[3].parse::<f64>())
                    {
                        *coupling.entry(a).or_default() += v;
                        *coupling.entry(b).or_default() += v;
                    }
                }
            }
        }
        finish(&mut cur, &mut nets);
        for (name, rc) in nets.iter_mut() {
            rc.coupling_ff = coupling.get(name).copied().unwrap_or(0.0);
        }
        Spef { nets }
    }

    pub fn load(path: &str) -> Result<Spef, SpefError> {
        let text = std::fs::read_to_string(path).map_err(|e| SpefError(format!("{path}: {e}")))?;
        Ok(Spef::parse(&text))
    }

    /// Extra driver load from wire capacitance, in pF (SPEF cap is fF).
    pub fn wire_load_pf(&self, net: &str) -> f64 {
        self.nets.get(net).map(|rc| rc.cap_ff / 1000.0).unwrap_or(0.0)
    }

    /// Lumped Elmore interconnect delay for a net, in ns (R[Ω]·C[fF] → ns).
    pub fn net_delay_ns(&self, net: &str) -> f64 {
        self.nets.get(net).map(|rc| rc.res_ohm * rc.cap_ff * 1e-6).unwrap_or(0.0)
    }
}
