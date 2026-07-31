// SPDX-License-Identifier: Apache-2.0
//! Yosys JSON netlists as a first-class input.
//!
//! Yosys is how most open flows produce a gate-level netlist, and `write_json` is one of its
//! primary outputs. Taking it directly means a user does not have to write Verilog back out and
//! have us re-parse it — a round trip that costs time and invites dialect drift.
//!
//! The contract these tests pin: **the reader is the only thing that differs.** Both loom
//! readers return the same `Netlist`, so timing must be bit-identical whichever way the design
//! arrived. `examples/top/top_yosys.json` is real `yosys write_json` output for
//! `examples/top/top.v`, not a hand-written approximation.
use vyges_sta_si::engine;
use vyges_sta_si::job::StaJob;

const VERILOG_JOB: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
const JSON_JOB: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top_json.sta");
const JSON_NETLIST: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top_yosys.json");
const V_NETLIST: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.v");

#[test]
fn a_yosys_json_netlist_loads() {
    let nl = engine::load_netlist(JSON_NETLIST).expect("yosys json netlist should load");
    assert_eq!(nl.module, "top");
    assert!(!nl.insts.is_empty(), "expected instances from the yosys netlist");
}

#[test]
fn the_reader_is_chosen_by_extension() {
    // .json -> the Yosys reader, anything else -> structural Verilog. Both land on the same
    // Netlist, so the two describe the same design.
    let from_json = engine::load_netlist(JSON_NETLIST).unwrap();
    let from_v = engine::load_netlist(V_NETLIST).unwrap();
    assert_eq!(from_json.module, from_v.module);
    assert_eq!(from_json.insts.len(), from_v.insts.len());
    assert_eq!(from_json.inputs.len(), from_v.inputs.len());
    assert_eq!(from_json.outputs.len(), from_v.outputs.len());
}

#[test]
fn timing_is_identical_from_either_netlist_format() {
    // The claim a user actually cares about: switching to the Yosys netlist changes nothing
    // about the answer. Same libs, same SPEF, same clock — only the netlist reader differs.
    let v = engine::analyze_job(&StaJob::load(VERILOG_JOB).unwrap()).unwrap();
    let j = engine::analyze_job(&StaJob::load(JSON_JOB).unwrap()).unwrap();
    assert_eq!(j.wns, v.wns, "WNS must not depend on the netlist format");
    assert_eq!(j.tns, v.tns, "TNS must not depend on the netlist format");
    assert_eq!(j.endpoints, v.endpoints);
    assert_eq!(j.worst_endpoint, v.worst_endpoint, "the critical path must be the same one");
    assert_eq!(j.worst_path.len(), v.worst_path.len());
}
