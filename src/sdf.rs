//! SDF (Standard Delay Format) writer — back-annotation output.
//!
//! Emits the timing `sta-si` computes as an SDF `DELAYFILE`: per-instance
//! **IOPATH** cell delays (rise/fall from the Liberty NLDM arcs at the net load)
//! and **TIMINGCHECK** SETUP/HOLD for sequential cells, plus top-level
//! **INTERCONNECT** net delays from the SPEF parasitics (Elmore). This is the
//! standard hand-off a gate-level / back-annotated simulator consumes — the same
//! Liberty + SPEF the rest of Loom already reads, serialized to SDF.
//!
//! Scope (honest): single-corner (min=typ=max); IOPATH uses the Liberty arc at a
//! nominal slew and the actual net load (full timer-propagated slew is the
//! accuracy upgrade); INTERCONNECT needs a SPEF (none → omitted). Pure std.

use std::collections::BTreeMap;

use crate::liberty::{Dir, Lib};
use crate::netlist::Netlist;
use crate::spef::Spef;

const SLEW: f64 = 0.05; // nominal input slew for the NLDM lookup

fn triple(v: f64) -> String {
    format!("({v:.6}:{v:.6}:{v:.6})")
}

/// Total downstream capacitive load on `net` (sum of sink input caps).
fn net_load(nl: &Netlist, lib: &Lib, net: &str) -> f64 {
    let mut load = 0.0;
    for inst in &nl.insts {
        if let Some(cell) = lib.cells.get(&inst.cell) {
            for (pin, n) in &inst.conns {
                if n == net && cell.pins.get(pin).map(|p| p.direction) == Some(Dir::In) {
                    load += cell.input_cap(pin);
                }
            }
        }
    }
    load
}

/// Emit a complete SDF `DELAYFILE` for the design.
pub fn emit(design: &str, nl: &Netlist, lib: &Lib, spef: Option<&Spef>) -> String {
    let mut s = String::new();
    s.push_str("(DELAYFILE\n");
    s.push_str("  (SDFVERSION \"3.0\")\n");
    s.push_str(&format!("  (DESIGN \"{design}\")\n"));
    s.push_str("  (VENDOR \"Vyges\")\n");
    s.push_str("  (PROGRAM \"vyges-sta-si\")\n");
    s.push_str(&format!("  (VERSION \"{}\")\n", crate::VERSION));
    s.push_str("  (DIVIDER /)\n");
    s.push_str("  (TIMESCALE 1ns)\n");

    // per-instance cell delays + setup/hold
    for inst in &nl.insts {
        let Some(cell) = lib.cells.get(&inst.cell) else { continue };
        let conn: BTreeMap<&str, &str> =
            inst.conns.iter().map(|(p, n)| (p.as_str(), n.as_str())).collect();

        let mut iopaths: Vec<String> = Vec::new();
        for out in cell.outputs() {
            let load = conn.get(out.name.as_str()).map_or(0.0, |n| net_load(nl, lib, n));
            for arc in &out.arcs {
                iopaths.push(format!(
                    "      (IOPATH {} {} {} {})",
                    arc.related_pin,
                    out.name,
                    triple(arc.cell_rise.lookup(SLEW, load)),
                    triple(arc.cell_fall.lookup(SLEW, load)),
                ));
            }
        }

        let mut checks: Vec<String> = Vec::new();
        if cell.is_seq {
            let clk = cell.clock_pin.clone().unwrap_or_default();
            for (pname, pin) in &cell.pins {
                if let Some(c) = pin.setup.first() {
                    checks.push(format!(
                        "      (SETUP {pname} (posedge {clk}) {})",
                        triple(c.eval(SLEW, SLEW))
                    ));
                }
                if let Some(c) = pin.hold.first() {
                    checks.push(format!(
                        "      (HOLD {pname} (posedge {clk}) {})",
                        triple(c.eval(SLEW, SLEW))
                    ));
                }
            }
        }

        if iopaths.is_empty() && checks.is_empty() {
            continue;
        }
        s.push_str("  (CELL\n");
        s.push_str(&format!("    (CELLTYPE \"{}\")\n", inst.cell));
        s.push_str(&format!("    (INSTANCE {})\n", inst.name));
        if !iopaths.is_empty() {
            s.push_str("    (DELAY (ABSOLUTE\n");
            s.push_str(&iopaths.join("\n"));
            s.push_str("\n    ))\n");
        }
        if !checks.is_empty() {
            s.push_str("    (TIMINGCHECK\n");
            s.push_str(&checks.join("\n"));
            s.push_str("\n    )\n");
        }
        s.push_str("  )\n");
    }

    // top-level interconnect delays from SPEF (driver pin -> each sink pin)
    if let Some(sp) = spef {
        let mut inter: Vec<String> = Vec::new();
        // net -> (driver (inst,pin), sinks [(inst,pin)])
        for inst in &nl.insts {
            let Some(cell) = lib.cells.get(&inst.cell) else { continue };
            for out in cell.outputs() {
                let Some(net) = inst.conns.iter().find(|(p, _)| *p == out.name).map(|(_, n)| n)
                else {
                    continue;
                };
                // sinks on this net
                let sinks: Vec<(&str, &str)> = nl
                    .insts
                    .iter()
                    .flat_map(|si| {
                        let sc = lib.cells.get(&si.cell);
                        si.conns.iter().filter_map(move |(p, n)| {
                            if n == net
                                && sc.and_then(|c| c.pins.get(p)).map(|pp| pp.direction)
                                    == Some(Dir::In)
                            {
                                Some((si.name.as_str(), p.as_str()))
                            } else {
                                None
                            }
                        })
                    })
                    .collect();
                if sinks.is_empty() {
                    continue;
                }
                // per-sink Elmore from the net's RC (fall back to lumped net delay)
                let netrc = sp.nets.get(net);
                let elmore = netrc
                    .and_then(|rc| rc.pin_node(&inst.name, &out.name))
                    .and_then(|d| netrc.unwrap().elmore(d, 0.0));
                let lumped = sp.net_delay_ns(net);
                for (si, sp_pin) in sinks {
                    let d = match (&elmore, netrc.and_then(|rc| rc.pin_node(si, sp_pin))) {
                        (Some(m), Some(n)) => m.get(n).copied().unwrap_or(lumped),
                        _ => lumped,
                    };
                    inter.push(format!(
                        "      (INTERCONNECT {}/{} {}/{} {})",
                        inst.name, out.name, si, sp_pin, triple(d)
                    ));
                }
            }
        }
        if !inter.is_empty() {
            s.push_str("  (CELL\n");
            s.push_str(&format!("    (CELLTYPE \"{design}\")\n"));
            s.push_str("    (INSTANCE)\n");
            s.push_str("    (DELAY (ABSOLUTE\n");
            s.push_str(&inter.join("\n"));
            s.push_str("\n    ))\n");
            s.push_str("  )\n");
        }
    }

    s.push_str(")\n");
    s
}
