//! SPEF reader — per-net RC network for STA (lumped fallback + per-pin Elmore).
//!
//! STA needs the net capacitance loading the driver and the interconnect delay
//! to each sink. This reads the per-net total cap (`*D_NET`), the `*RES`
//! resistors, the grounded `*CAP` entries, the two-node `*CAP` coupling entries,
//! and the `*CONN` instance pins. From that it offers a lumped Elmore (`R·C`)
//! and a true **per-pin tree Elmore** (delay to each sink = Σ over the
//! driver→sink path of `R · downstream-cap`). Units are assumed fF / Ω (what
//! `vyges-extract` emits).
//!
//! Pure std — fully unit-tested offline.

use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, Default)]
pub struct NetRc {
    pub cap_ff: f64,                  // total cap (grounded + coupling), from *D_NET
    pub res_ohm: f64,                 // summed *RES (lumped fallback)
    pub coupling_ff: f64,             // total coupling cap (sum over neighbours)
    pub coupling: Vec<(String, f64)>, // per-aggressor coupling (net, Cc) for window-aware SI
    // RC network (for per-pin tree Elmore):
    pub net_node: String,                 // node where coupling attaches (the net node)
    pub ground: Vec<(String, f64)>,       // (node, grounded cap fF)
    pub res: Vec<(String, String, f64)>,  // (node a, node b, ohm)
    pub pins: Vec<(String, String, String)>, // (instance, pin, node) from *CONN
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

fn node_tok(t: &str) -> String {
    t.trim_start_matches('*').to_string()
}

impl NetRc {
    /// Per-node Elmore delays (ns) for the net's RC tree rooted at `driver`,
    /// with `xtalk_cap_ff` added at the net node (the Miller crosstalk load).
    /// Returns `None` if the network is not a tree reachable from the driver
    /// (caller falls back to the lumped delay).
    pub fn elmore(&self, driver: &str, xtalk_cap_ff: f64) -> Option<BTreeMap<String, f64>> {
        if self.res.is_empty() {
            return None;
        }
        // node capacitances
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (node, c) in &self.ground {
            *cap.entry(node.as_str()).or_default() += c;
        }
        *cap.entry(self.net_node.as_str()).or_default() += xtalk_cap_ff;
        // adjacency
        let mut adj: HashMap<&str, Vec<(&str, f64)>> = HashMap::new();
        for (a, b, r) in &self.res {
            adj.entry(a).or_default().push((b, *r));
            adj.entry(b).or_default().push((a, *r));
            cap.entry(a.as_str()).or_default();
            cap.entry(b.as_str()).or_default();
        }
        if !adj.contains_key(driver) {
            return None;
        }
        // BFS tree from the driver; record parent + parent-edge R, in visit order
        let mut parent: HashMap<&str, (&str, f64)> = HashMap::new();
        let mut order: Vec<&str> = vec![driver];
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        seen.insert(driver);
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for &(v, r) in adj.get(u).map(|x| x.as_slice()).unwrap_or(&[]) {
                if !seen.insert(v) {
                    if parent.get(u).map(|p| p.0) != Some(v) {
                        return None; // a cycle reached an already-visited node -> not a tree
                    }
                    continue;
                }
                parent.insert(v, (u, r));
                order.push(v);
            }
        }
        // subtree caps: reverse BFS order accumulates child caps into parents
        let mut sub: HashMap<&str, f64> = HashMap::new();
        for &nd in &order {
            *sub.entry(nd).or_default() += cap.get(nd).copied().unwrap_or(0.0);
        }
        for &nd in order.iter().skip(1).rev() {
            let (p, _) = parent[nd];
            let add = sub[nd];
            *sub.get_mut(p).unwrap() += add;
        }
        // delays: delay[child] = delay[parent] + R_edge * subtree_cap[child]
        let mut delay: BTreeMap<String, f64> = BTreeMap::new();
        delay.insert(driver.to_string(), 0.0);
        for &nd in order.iter().skip(1) {
            let (p, r) = parent[nd];
            let d = delay[p] + r * sub[nd] * 1e-6; // R[Ω]·C[fF] -> ns
            delay.insert(nd.to_string(), d);
        }
        Some(delay)
    }

    /// SPEF node token for an instance pin, if present in `*CONN`.
    pub fn pin_node(&self, inst: &str, pin: &str) -> Option<&str> {
        self.pins.iter().find(|(i, p, _)| i == inst && p == pin).map(|(_, _, n)| n.as_str())
    }
}

impl Spef {
    pub fn parse(text: &str) -> Spef {
        let mut names: BTreeMap<usize, String> = BTreeMap::new();
        let mut nets: BTreeMap<String, NetRc> = BTreeMap::new();
        let mut coupling: BTreeMap<String, f64> = BTreeMap::new();
        let mut coupling_list: BTreeMap<String, Vec<(String, f64)>> = BTreeMap::new();
        let mut cur: Option<(String, String, NetRc)> = None; // (name, net_node_token, rc)
        let mut sect = ""; // "", "namemap", "conn", "cap", "res"

        let finish = |cur: &mut Option<(String, String, NetRc)>,
                      nets: &mut BTreeMap<String, NetRc>| {
            if let Some((name, _, rc)) = cur.take() {
                if !name.is_empty() {
                    nets.insert(name, rc);
                }
            }
        };
        let netname = |tok: &str, names: &BTreeMap<usize, String>| -> Option<String> {
            let body = tok.trim_start_matches('*');
            if body.contains(':') {
                return None;
            }
            body.parse::<usize>().ok().and_then(|i| names.get(&i).cloned())
        };
        // resolve a pin token "iid:pin" -> (instance name, pin)
        let pin_of = |tok: &str, names: &BTreeMap<usize, String>| -> Option<(String, String)> {
            let body = tok.trim_start_matches('*');
            let (ids, pin) = body.split_once(':')?;
            let inst = ids.parse::<usize>().ok().and_then(|i| names.get(&i).cloned())?;
            Some((inst, pin.to_string()))
        };

        for raw in text.lines() {
            let t = raw.trim();
            if t == "*NAME_MAP" {
                sect = "namemap";
                continue;
            }
            if t.starts_with("*D_NET") {
                sect = "";
                finish(&mut cur, &mut nets);
                let toks: Vec<&str> = t.split_whitespace().collect();
                let idtok = toks.get(1).copied().unwrap_or("");
                let id = idtok.trim_start_matches('*').parse::<usize>().ok();
                let cap = toks.get(2).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
                let name = id.and_then(|i| names.get(&i).cloned()).unwrap_or_default();
                let net_node = node_tok(idtok);
                cur = Some((
                    name,
                    net_node.clone(),
                    NetRc { cap_ff: cap, net_node, ..Default::default() },
                ));
                continue;
            }
            match t {
                "*CONN" => {
                    sect = "conn";
                    continue;
                }
                "*CAP" => {
                    sect = "cap";
                    continue;
                }
                "*RES" => {
                    sect = "res";
                    continue;
                }
                "*END" => {
                    sect = "";
                    finish(&mut cur, &mut nets);
                    continue;
                }
                _ => {}
            }
            let toks: Vec<&str> = t.split_whitespace().collect();
            match sect {
                "namemap" if t.starts_with('*') => {
                    if let (Some(idtok), Some(name)) = (toks.first(), toks.get(1)) {
                        if let Ok(id) = idtok.trim_start_matches('*').parse::<usize>() {
                            names.insert(id, name.to_string());
                        }
                    }
                }
                "conn" if toks.first() == Some(&"*I") => {
                    if let Some(node) = toks.get(1) {
                        if let Some((inst, pin)) = pin_of(node, &names) {
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.pins.push((inst, pin, node_tok(node)));
                            }
                        }
                    }
                }
                "res" => {
                    // `<idx> *a *b <ohm>`
                    if toks.len() >= 4 {
                        if let Ok(r) = toks[3].parse::<f64>() {
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.res_ohm += r;
                                rc.res.push((node_tok(toks[1]), node_tok(toks[2]), r));
                            }
                        }
                    }
                }
                "cap" => {
                    if toks.len() >= 4 && toks[1].starts_with('*') && toks[2].starts_with('*') {
                        // two-node coupling cap `<idx> *A *B <ff>`
                        if let (Some(a), Some(b), Ok(v)) =
                            (netname(toks[1], &names), netname(toks[2], &names), toks[3].parse::<f64>())
                        {
                            *coupling.entry(a.clone()).or_default() += v;
                            *coupling.entry(b.clone()).or_default() += v;
                            coupling_list.entry(a.clone()).or_default().push((b.clone(), v));
                            coupling_list.entry(b).or_default().push((a, v));
                        }
                    } else if toks.len() >= 3 && toks[1].starts_with('*') {
                        // grounded cap `<idx> *node <ff>`
                        if let Ok(v) = toks[2].parse::<f64>() {
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.ground.push((node_tok(toks[1]), v));
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        finish(&mut cur, &mut nets);
        for (name, rc) in nets.iter_mut() {
            rc.coupling_ff = coupling.get(name).copied().unwrap_or(0.0);
            rc.coupling = coupling_list.get(name).cloned().unwrap_or_default();
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
