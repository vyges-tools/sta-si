//! Experimental: export sta-si timing data as **timing-IR** records.
//!
//! Emits, as JSON, the per-cell delay arcs and per-flop setup/hold checks that
//! `sta-si` computes from a Liberty library + a gate-level netlist — the timing
//! interchange a back-annotation or simulation flow consumes. Cell-delay and
//! setup/hold records are Liberty-derived and cross-check exactly against an
//! OpenSTA-generated reference on a small `aigpdk` design.
//!
//! Run:
//!   cargo run --example timing_ir_export -- <cells.lib> <design.v>
//!
//! Not yet emitted (honest scope): interconnect delays (need SPEF or SDF back-
//! annotation) and clock-arrival records. This is a capability probe, not a
//! supported output format.

use std::collections::BTreeMap;

use vyges_sta_si::liberty::{Dir, Lib};
use vyges_sta_si::netlist;

const SLEW: f64 = 0.05; // nominal input slew for the NLDM lookup

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: timing_ir_export <liberty.lib> <netlist.v>");
        std::process::exit(2);
    }
    let lib = Lib::load(&args[1]).expect("load liberty");
    let nl = netlist::load(&args[2]).expect("load netlist");

    // downstream capacitive load on a net (sum of sink input caps)
    let net_load = |net: &str| -> f64 {
        let mut load = 0.0;
        for inst in &nl.insts {
            if let Some(cell) = lib.cells.get(&inst.cell) {
                for (pin, n) in &inst.conns {
                    if n == net {
                        if let Some(p) = cell.pins.get(pin) {
                            if p.direction == Dir::In {
                                load += cell.input_cap(pin);
                            }
                        }
                    }
                }
            }
        }
        load
    };

    let mut arcs: Vec<String> = Vec::new();
    let mut checks: Vec<String> = Vec::new();
    for inst in &nl.insts {
        let Some(cell) = lib.cells.get(&inst.cell) else {
            continue;
        };
        let conn: BTreeMap<&str, &str> = inst
            .conns
            .iter()
            .map(|(p, n)| (p.as_str(), n.as_str()))
            .collect();

        // delay arcs: input pin -> each output pin
        for out in cell.outputs() {
            let load = conn.get(out.name.as_str()).map_or(0.0, |n| net_load(n));
            for arc in &out.arcs {
                arcs.push(format!(
                    "    {{\"cell_instance\":\"{}\",\"driver_pin\":\"{}\",\"load_pin\":\"{}\",\
                     \"rise_delay\":{:.6},\"fall_delay\":{:.6}}}",
                    inst.name,
                    arc.related_pin,
                    out.name,
                    arc.cell_rise.lookup(SLEW, load),
                    arc.cell_fall.lookup(SLEW, load)
                ));
            }
        }

        // setup/hold checks on sequential cells
        if cell.is_seq {
            let clk = cell.clock_pin.clone().unwrap_or_default();
            for (pname, pin) in &cell.pins {
                if pin.setup.is_empty() && pin.hold.is_empty() {
                    continue;
                }
                checks.push(format!(
                    "    {{\"cell_instance\":\"{}\",\"d_pin\":\"{}\",\"clk_pin\":\"{}\",\
                     \"setup\":{:.6},\"hold\":{:.6}}}",
                    inst.name,
                    pname,
                    clk,
                    pin.setup.first().map_or(0.0, |c| c.eval(SLEW, SLEW)),
                    pin.hold.first().map_or(0.0, |c| c.eval(SLEW, SLEW))
                ));
            }
        }
    }

    println!("{{");
    println!("  \"generator\": \"vyges-sta-si timing_ir_export (experimental)\",");
    println!("  \"timing_arcs\": [\n{}\n  ],", arcs.join(",\n"));
    println!("  \"setup_hold_checks\": [\n{}\n  ],", checks.join(",\n"));
    println!("  \"interconnect_delays\": [],  // TODO: from SPEF / SDF");
    println!("  \"clock_arrivals\": []        // TODO");
    println!("}}");
    eprintln!(
        "emitted {} timing arcs + {} setup/hold checks",
        arcs.len(),
        checks.len()
    );
}
