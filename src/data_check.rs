// SPDX-License-Identifier: Apache-2.0
//! `set_data_check` / `unset_data_check`: a setup or hold check between two DATA pins, as the
//! reference keeps them (`DataCheck`: margins by from-edge, to-edge and setup/hold) and times
//! them (`VisitPathEnds::visitDataCheckEnd`, `PathEndDataCheck`).
//!
//! The command (the reference's SDC Tcl): `-from` / `-rise_from` / `-fall_from` name ONE pin or
//! port (Error 128 for more, 425 for none), `-to` / `-rise_to` / `-fall_to` the same (426); a
//! plain `-from` / `-to` sets both edges; `-setup` or `-hold` alone sets that side, neither (or
//! both) sets both. A later command overwrites what it names; `unset_data_check` removes it.
//!
//! The timing, at the to-pin (the endpoint), for each from-edge and to-edge with a margin:
//! - setup is ZERO-cycle (`CycleAccting`: "data check setup checks are zero cycle"): required =
//!   the from-pin's EARLY arrival − the setup uncertainty − the margin;
//! - hold is checked against the from-pin's LATE arrival one cycle earlier — the to-pin's data
//!   is taken from the next launch (`src_cycle + 1`): slack = early arrival + period − (late
//!   from-arrival + the hold uncertainty + the margin).

use crate::sdc::{KeptArg, KeptCmd};

/// Margins of one from-pin → to-pin check: `[setup, hold][from edge][to edge]` (edge 0 rise).
pub type Margins = [[[Option<f64>; 2]; 2]; 2];

/// One data check: the from-pin (the reference) and the to-pin (the checked one), as first
/// written (`inst/pin` or a port name) with the accessor that named each, and the pins they
/// resolved to (`nodes`: from, to) — what the check is keyed on, as the reference keys a
/// `DataCheck` on its `Pin*`s.
#[derive(Debug, Clone, PartialEq)]
pub struct DataCheck {
    pub from: (String, String),
    pub to: (String, String),
    pub nodes: (usize, usize),
    pub margins: Margins,
}

impl DataCheck {
    /// Whether any margin of `side` (0 setup, 1 hold) is set.
    pub fn has(&self, side: usize) -> bool {
        self.margins[side].iter().flatten().any(Option::is_some)
    }
}

/// The edges a key names: `-rise_*` rise, `-fall_*` fall, the plain key both.
fn edges(key: &str) -> &'static [usize] {
    if key.starts_with("-rise") {
        &[0]
    } else if key.starts_with("-fall") {
        &[1]
    } else {
        &[0, 1]
    }
}

/// A pin as written, `(accessor, name)`, and the edges its key names.
type End = ((String, String), &'static [usize]);

/// The one pin a `-from` / `-to` key names, with its accessor, and the key's edges.
fn pin_of(c: &KeptCmd, keys: [&'static str; 3]) -> Result<Option<End>, String> {
    // the reference takes the first key of the three it finds, in this order
    let Some((k, v)) = keys.iter().find_map(|k| c.key(k).map(|v| (*k, v))) else { return Ok(None) };
    let (accessor, names) = match v {
        KeptArg::Objects(o) => (o.accessor.clone(), o.names.clone()),
        KeptArg::Word(w) => (String::new(), w.split_whitespace().map(String::from).collect()),
    };
    match names.as_slice() {
        [one] => Ok(Some(((accessor, one.clone()), edges(k)))),
        _ => Err(format!("{k} must be a single port or pin (Error 128)")),
    }
}

/// The data checks `cmds` leave in force, in the order first set. A command that cannot be
/// applied is said in `warnings` and skipped.
///
/// `resolve` finds the ONE pin an end names (`(accessor, name)`), or none. A check is keyed on
/// the pins (`Sdc::setDataCheck` / `unsetDataCheck` look the check up by its `Pin*`s), so a later
/// command that names the same pin another way — `a/A` and `[get_pins a/?]` — overwrites or
/// takes back the same check. An end that names no pin (or several) is said and not applied.
pub fn data_checks(cmds: &[KeptCmd], warnings: &mut Vec<String>, resolve: &dyn Fn(&(String, String)) -> Option<usize>) -> Vec<DataCheck> {
    let mut out: Vec<DataCheck> = Vec::new();
    for c in cmds.iter().filter(|c| c.cmd == "set_data_check" || c.cmd == "unset_data_check") {
        let set = c.cmd == "set_data_check";
        let ends = (
            pin_of(c, ["-from", "-rise_from", "-fall_from"]),
            pin_of(c, ["-to", "-rise_to", "-fall_to"]),
        );
        let ((from, from_rf), (to, to_rf)) = match ends {
            (Ok(Some(f)), Ok(Some(t))) => (f, t),
            (Err(e), _) | (_, Err(e)) => {
                warnings.push(format!("{}: {e}; not applied", c.cmd));
                continue;
            }
            _ => continue, // the reader refused it already (Errors 425-428)
        };
        if c.key("-clock").is_some() {
            warnings.push(format!("{}: -clock (a check for one clock only) is not modelled; not applied", c.cmd));
            continue;
        }
        // SDC's `-rise` / `-fall` restrict the check to one edge of the data; applied as a plain
        // check it would also constrain the other edge
        if let Some(edge) = ["-rise", "-fall"].into_iter().find(|f| c.has(f)) {
            warnings.push(format!("{}: {edge} (a check on one data edge) is not modelled; not applied", c.cmd));
            continue;
        }
        let sides: &[usize] = match (c.has("-setup"), c.has("-hold")) {
            (true, false) => &[0],
            (false, true) => &[1],
            _ => &[0, 1],
        };
        let value = if set {
            match c.number(0) {
                Some(v) => Some(v),
                None => {
                    warnings.push(format!("{}: the margin is not a number; not applied", c.cmd));
                    continue;
                }
            }
        } else {
            None
        };
        let nodes = match (resolve(&from), resolve(&to)) {
            (Some(f), Some(t)) => (f, t),
            _ => {
                warnings.push(format!("{}: {} -> {}: pin not found, or not one pin; not applied", c.cmd, from.1, to.1));
                continue;
            }
        };
        let at = match out.iter().position(|d| d.nodes == nodes) {
            Some(i) => i,
            None if set => {
                out.push(DataCheck { from: from.clone(), to: to.clone(), nodes, margins: Margins::default() });
                out.len() - 1
            }
            None => continue, // nothing to take back
        };
        for &s in sides {
            for &f in from_rf {
                for &t in to_rf {
                    out[at].margins[s][f][t] = value;
                }
            }
        }
    }
    out.retain(|d| d.has(0) || d.has(1));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdc::Sdc;

    /// Pins `a/A`, `a/B`, `a/C` (nodes 0, 1, 2); `[get_pins a/?X]` stands for a pattern that
    /// resolves to the pin `a/X`, so a test can name one pin two ways.
    fn resolve((_, name): &(String, String)) -> Option<usize> {
        let name = name.replace('?', "");
        ["a/A", "a/B", "a/C"].iter().position(|p| *p == name)
    }

    fn checks(text: &str) -> (Vec<DataCheck>, Vec<String>) {
        let s = Sdc::parse(text).expect("reads");
        let mut w = Vec::new();
        (data_checks(&s.kept, &mut w, &resolve), w)
    }

    /// Rule (`Sdc::setDataCheck` / `unsetDataCheck`, keyed on the `Pin*`s): a check is the
    /// pins', not the spelling's — a later command naming the same pins another way overwrites
    /// it or takes it back; an end naming no pin is said and not applied.
    #[test]
    fn a_check_is_keyed_on_its_pins_not_their_spelling() {
        let (d, w) = checks("set_data_check -from [get_pins a/A] -to [get_pins a/B] 0.1\nset_data_check -from a/?A -to [get_pins a/?B] -setup 0.2\n");
        assert!(w.is_empty(), "{w:?}");
        assert_eq!(d.len(), 1, "one check: {d:?}");
        assert_eq!((d[0].margins[0][0][0], d[0].margins[1][0][0]), (Some(0.2), Some(0.1)), "the setup side overwritten");
        let (d, _) = checks("set_data_check -from [get_pins a/A] -to [get_pins a/B] 0.1\nunset_data_check -from [get_pins a/?A] -to [get_pins a/?B]\n");
        assert!(d.is_empty(), "taken back by another spelling: {d:?}");
        let (d, w) = checks("set_data_check -from [get_pins a/A] -to [get_pins a/Z] 0.1\n");
        assert!(d.is_empty() && w.len() == 1 && w[0].contains("pin not found"), "{d:?} {w:?}");
    }

    /// Rule (`set_data_check`, `DataCheck::setMargin`): a plain `-from`/`-to` sets both edges, a
    /// side flag one side, a later command overwrites, `unset_data_check` removes.
    #[test]
    fn margins_by_edge_and_side_as_the_reference_keeps_them() {
        let (d, w) = checks(
            "set_data_check -from [get_pins a/A] -to [get_pins a/B] 0.1\nset_data_check -rise_from [get_pins a/A] -to [get_pins a/B] -hold 0.3\nunset_data_check -from [get_pins a/A] -fall_to [get_pins a/B] -setup\n",
        );
        assert!(w.is_empty(), "{w:?}");
        assert_eq!(d.len(), 1);
        let m = d[0].margins;
        assert_eq!(m[0][0], [Some(0.1), None], "setup: the fall-to side taken back");
        assert_eq!(m[0][1], [Some(0.1), None]);
        assert_eq!(m[1][0], [Some(0.3), Some(0.3)], "hold from rise overwritten");
        assert_eq!(m[1][1], [Some(0.1), Some(0.1)]);
    }

    /// SDC 1.4+ `set_data_check -rise` / `-fall` checks one edge of the data only. That is not
    /// modelled, so the command is skipped with a warning, never applied to both edges.
    #[test]
    fn a_one_edge_check_is_skipped_not_applied_to_both_edges() {
        for edge in ["-rise", "-fall"] {
            let (d, w) = checks(&format!("set_data_check {edge} -from [get_pins a/A] -to [get_pins a/B] 0.1\n"));
            assert!(d.is_empty(), "{edge}: applied: {d:?}");
            assert_eq!(w.len(), 1, "{edge}: {w:?}");
            assert!(w[0].contains(edge), "{w:?}");
        }
    }

    /// Rule (`get_port_pin_arg`, Warning/Error 128): one pin per side; `-clock` is not modelled.
    #[test]
    fn several_pins_or_a_clock_are_refused() {
        let (d, w) = checks("set_data_check -from {a/A a/C} -to [get_pins a/B] 0.1\nset_data_check -from [get_pins a/A] -to [get_pins a/B] -clock clk 0.1\n");
        assert!(d.is_empty());
        assert_eq!(w.len(), 2, "{w:?}");
    }
}
