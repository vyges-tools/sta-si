// SPDX-License-Identifier: Apache-2.0
//! `set_disable_timing` / `unset_disable_timing`: the arcs and pins the constraints take out of
//! the timing graph, as the reference keeps and consults them.
//!
//! What a command disables, by the kind of object it names (`set_disable_timing` in the
//! reference's SDC Tcl):
//! - an INSTANCE: its arcs (`DisabledPorts`) — all of them without `-from`/`-to`, else those from
//!   a `-from` port, into a `-to` port, or between the pair when both are given;
//! - a LIBRARY CELL: the same, for every instance of the cell;
//! - a PIN, PORT or LIBRARY PIN: the pin itself (`disabled_pins_`, `disabled_lib_ports_`), with
//!   `-from`/`-to` ignored (Warning 429).
//!
//! How the search and the delay calculation consult them (`SearchPred0`, `DcalcPred`,
//! `VisitPathEnds::checkEdgeEnabled`):
//! - nothing leaves a disabled pin: no arc FROM it is timed or has a delay calculated
//!   (`isDisabledConstraint(from_pin)`); an arc INTO it still arrives there;
//! - a disabled arc is neither searched nor calculated, so it sets no slew at its output;
//! - "all arcs" of an instance or cell does not include its timing checks
//!   (`DisabledPorts::isDisabled`: `all_ && !role->isTimingCheck()`); a `-from`/`-to` names
//!   arcs of every role, checks included;
//! - a check whose clock pin is disabled is not made (`searchFrom` of the check's from vertex).

use crate::liberty::Lib;
use crate::netlist::Netlist;
use crate::sdc::ArcCmd;
use std::collections::{HashMap, HashSet};

/// The arcs one instance or cell has disabled (`DisabledPorts`): all, from a port, into a port,
/// or a pair.
#[derive(Debug, Default, Clone)]
struct Ports {
    all: bool,
    from: HashSet<String>,
    to: HashSet<String>,
    from_to: HashSet<(String, String)>,
}

impl Ports {
    /// `DisabledPorts::isDisabled`.
    fn disabled(&self, from: &str, to: &str, check: bool) -> bool {
        (self.all && !check) || self.from.contains(from) || self.to.contains(to) || self.from_to.contains(&(from.to_string(), to.to_string()))
    }
    /// `-from`/`-to` set (`add`) or taken back (`!add`), as `set_disable_timing_instance` /
    /// `unset_…` expand them: each port a list of its bits (`port_members`), every pair of a
    /// `-from` and a `-to` bit when both are given.
    fn apply(&mut self, from: Option<&[String]>, to: Option<&[String]>, add: bool) {
        match (from, to) {
            (None, None) => self.all = add,
            (None, Some(ts)) => {
                for t in ts {
                    set(&mut self.to, t.clone(), add);
                }
            }
            (Some(fs), None) => {
                for f in fs {
                    set(&mut self.from, f.clone(), add);
                }
            }
            (Some(fs), Some(ts)) => {
                for f in fs {
                    for t in ts {
                        set(&mut self.from_to, (f.clone(), t.clone()), add);
                    }
                }
            }
        }
    }
}

fn set<T: std::hash::Hash + Eq>(s: &mut HashSet<T>, v: T, add: bool) {
    if add {
        s.insert(v);
    } else {
        s.remove(&v);
    }
}

/// Every disable the constraints leave in force — see the module documentation.
#[derive(Debug, Default, Clone)]
pub struct Disables {
    /// Instance pins (`inst/pin`) and top-level ports, by name.
    pins: HashSet<String>,
    /// Library pins, `(cell, port)`: the pin of every instance of the cell.
    lib_ports: HashSet<(String, String)>,
    insts: HashMap<String, Ports>,
    cells: HashMap<String, Ports>,
}

/// Tcl `string match` for `*` and `?` (the reference's pattern match without `-regexp`).
fn glob(pat: &str, s: &str) -> bool {
    if !pat.contains(['*', '?']) {
        return pat == s;
    }
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut i, mut j, mut star, mut mark) = (0usize, 0usize, None, 0usize);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(s) = star {
            i = s + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    p[i..].iter().all(|c| *c == '*')
}

/// One instance or cell a disable names, with its `-from`/`-to` ports as bits.
struct Target {
    is_inst: bool,
    name: String,
    from: Option<Vec<String>>,
    to: Option<Vec<String>>,
}

/// The objects one disable names, by kind.
#[derive(Default)]
struct Named {
    cells: Vec<String>,
    lib_ports: Vec<(String, String)>,
    insts: Vec<String>,
    pins: Vec<String>,
}

impl Disables {
    /// The disables `cmds` leave in force (`set_disable_timing` and `unset_disable_timing`, in file
    /// order), resolved on `nl` and `lib`. A name that matches nothing is said in `warnings`, as
    /// the reference warns (`object '…' not found`).
    pub fn new(cmds: &[ArcCmd], nl: &Netlist, lib: &Lib, warnings: &mut Vec<String>) -> Disables {
        let mut d = Disables::default();
        for c in cmds {
            let add = c.cmd == "set_disable_timing";
            let key = |k: &str| c.keys.iter().rev().find(|(n, _)| n == k).map(|(_, v)| v.as_str());
            let (from, to) = (key("-from"), key("-to"));
            let named = resolve(c, nl, lib, warnings);
            // `parse_disable_inst_ports` / `parse_disable_cell_ports`: each port is looked up on
            // the instance's (cell's) library cell, and one that is not there is an error that
            // ends the command (431 for an instance, 432 for a cell) — refused whole here.
            let mut ports: Vec<Target> = Vec::new();
            let mut missing = None;
            let kinds = named.cells.iter().map(|c| (false, c.clone(), Some(c.clone()))).chain(
                named.insts.iter().map(|i| (true, i.clone(), nl.insts.iter().find(|x| x.name == *i).map(|x| x.cell.clone()))),
            );
            for (is_inst, name, cell) in kinds {
                let cell = cell.as_deref().and_then(|c| lib.cell(c));
                let bits = |p: Option<&str>| -> Result<Option<Vec<String>>, String> {
                    let Some(p) = p else { return Ok(None) };
                    let members = cell.map(|c| port_members(c, p)).unwrap_or_default();
                    if members.is_empty() {
                        let code = if is_inst { 431 } else { 432 };
                        return Err(format!("Error {code}: pin '{name}/{p}' not found"));
                    }
                    Ok(Some(members))
                };
                match (bits(from), bits(to)) {
                    (Ok(from), Ok(to)) => ports.push(Target { is_inst, name, from, to }),
                    (Err(e), _) | (_, Err(e)) => {
                        missing = Some(e);
                        break;
                    }
                }
            }
            if let Some(e) = missing {
                warnings.push(format!("{}: {e}; not applied", c.cmd));
                continue;
            }
            for t in ports {
                let map = if t.is_inst { &mut d.insts } else { &mut d.cells };
                map.entry(t.name).or_default().apply(t.from.as_deref(), t.to.as_deref(), add);
            }
            for p in named.pins {
                set(&mut d.pins, p, add);
            }
            for lp in named.lib_ports {
                set(&mut d.lib_ports, lp, add);
            }
        }
        d
    }

    /// Nothing is disabled.
    pub fn is_empty(&self) -> bool {
        self.pins.is_empty() && self.lib_ports.is_empty() && self.insts.values().chain(self.cells.values()).all(|p| !p.all && p.from.is_empty() && p.to.is_empty() && p.from_to.is_empty())
    }

    /// `Sdc::isDisabledConstraint(pin)`: pin `port` of instance `inst` (of library cell `cell`),
    /// or the top-level port `port` when `inst` is `None`.
    pub fn pin(&self, inst: Option<&str>, cell: Option<&str>, port: &str) -> bool {
        match inst {
            None => self.pins.contains(port),
            Some(i) => self.pins.contains(&format!("{i}/{port}")) || cell.is_some_and(|c| self.lib_ports.contains(&(c.to_string(), port.to_string()))),
        }
    }

    /// `Sdc::isDisabled(inst, from, to, role)`: the arc `from` → `to` of instance `inst` (of
    /// library cell `cell`), a timing check when `check`.
    pub fn arc(&self, inst: &str, cell: &str, from: &str, to: &str, check: bool) -> bool {
        self.insts.get(inst).is_some_and(|p| p.disabled(from, to, check)) || self.cells.get(cell).is_some_and(|p| p.disabled(from, to, check))
    }
}

/// The objects `c` names, by kind: the accessor says which; a bare name is found as the
/// reference's `get_object_args` finds it — a library cell, a library pin, an instance, a port,
/// then a pin, the first kind that has it.
fn resolve(c: &ArcCmd, nl: &Netlist, lib: &Lib, warnings: &mut Vec<String>) -> Named {
    let mut n = Named::default();
    let accessor = c.written.accessor.as_str();
    // A query whose flags change what its names match (`-hierarchical`, `-regexp`, `-nocase`, a
    // `-filter`) is not matched here by a plain glob: refused, said, never guessed.
    if !c.written.flags.is_empty() || !c.written.keys.is_empty() {
        warnings.push(format!("{}: [{accessor} {}] is not matched here; not applied", c.cmd, c.written.flags.join(" ")));
        return n;
    }
    let ports = || nl.inputs.iter().chain(&nl.outputs).chain(&nl.inouts);
    for name in &c.objects {
        let found = match accessor {
            "get_cells" => insts(nl, name, &mut n),
            "get_lib_cells" => lib_cells(lib, name, &mut n),
            "get_lib_pins" => lib_pins(lib, name, &mut n),
            "get_pins" => inst_pins(nl, name, &mut n),
            "get_ports" => port_names(ports(), name, &mut n),
            _ => match name.as_str() {
                "*INPUTS*" => port_names(nl.inputs.iter(), "*", &mut n),
                "*OUTPUTS*" => port_names(nl.outputs.iter(), "*", &mut n),
                _ => lib_cells(lib, name, &mut n) || lib_pins(lib, name, &mut n) || insts(nl, name, &mut n) || port_names(ports(), name, &mut n) || inst_pins(nl, name, &mut n),
            },
        };
        if !found {
            warnings.push(format!("{}: object '{name}' not found", c.cmd));
        }
    }
    n
}

/// `port_members`: the port named `p` of `cell`, or the bits of the bus `p` (`p[0]`, `p[1]`, …);
/// empty when the cell has neither.
fn port_members(cell: &crate::liberty::Cell, p: &str) -> Vec<String> {
    if cell.pins.contains_key(p) {
        return vec![p.to_string()];
    }
    let prefix = format!("{p}[");
    cell.pins.keys().filter(|k| k.starts_with(&prefix) && k.ends_with(']')).cloned().collect()
}

fn insts(nl: &Netlist, pat: &str, n: &mut Named) -> bool {
    let before = n.insts.len();
    n.insts.extend(nl.insts.iter().filter(|i| glob(pat, &i.name)).map(|i| i.name.clone()));
    n.insts.len() > before
}

/// `lib/cell` or `cell`: the cell part is matched (one library set is read here).
fn lib_cells(lib: &Lib, pat: &str, n: &mut Named) -> bool {
    let cell = pat.rsplit('/').next().unwrap_or(pat);
    if pat.matches('/').count() > 1 {
        return false;
    }
    let before = n.cells.len();
    n.cells.extend(lib.cells.keys().filter(|c| glob(cell, c)).cloned());
    n.cells.len() > before
}

/// `lib/cell/port` or `cell/port`.
fn lib_pins(lib: &Lib, pat: &str, n: &mut Named) -> bool {
    let mut parts: Vec<&str> = pat.rsplit('/').collect();
    if parts.len() < 2 || parts.len() > 3 {
        return false;
    }
    let (port, cell) = (parts.remove(0), parts.remove(0));
    let before = n.lib_ports.len();
    for (cname, c) in lib.cells.iter().filter(|(k, _)| glob(cell, k)) {
        n.lib_ports.extend(c.pins.keys().filter(|p| glob(port, p)).map(|p| (cname.clone(), p.clone())));
    }
    n.lib_ports.len() > before
}

/// `inst/pin`: the pin part after the last divider, the instance before it.
fn inst_pins(nl: &Netlist, pat: &str, n: &mut Named) -> bool {
    let Some((ip, pp)) = pat.rsplit_once('/') else { return false };
    let before = n.pins.len();
    for i in nl.insts.iter().filter(|i| glob(ip, &i.name)) {
        let pins = i.conns.iter().map(|(p, _)| p).chain(i.ties.iter().map(|(p, _)| p));
        n.pins.extend(pins.filter(|p| glob(pp, p)).map(|p| format!("{}/{p}", i.name)));
    }
    n.pins.len() > before
}

fn port_names<'a>(ports: impl Iterator<Item = &'a String>, pat: &str, n: &mut Named) -> bool {
    let before = n.pins.len();
    n.pins.extend(ports.filter(|p| glob(pat, p)).cloned());
    n.pins.len() > before
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`DisabledPorts::isDisabled`): "all" spares timing checks; `-from`/`-to` do not.
    #[test]
    fn all_spares_checks_and_a_port_does_not() {
        let mut p = Ports::default();
        let v = |x: &str| vec![x.to_string()];
        p.apply(None, None, true);
        assert!(p.disabled("A", "Z", false));
        assert!(!p.disabled("CK", "D", true));
        let mut p = Ports::default();
        p.apply(Some(&v("CK")), None, true);
        assert!(p.disabled("CK", "D", true));
        assert!(!p.disabled("A", "Z", false));
        p.apply(Some(&v("CK")), None, false);
        assert!(!p.disabled("CK", "D", true), "unset takes it back");
        let mut p = Ports::default();
        p.apply(Some(&v("A")), Some(&v("Z")), true);
        assert!(p.disabled("A", "Z", false) && !p.disabled("B", "Z", false));
    }
}
