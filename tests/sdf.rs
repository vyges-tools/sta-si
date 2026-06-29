//! SDF writer: IOPATH (cell delay), TIMINGCHECK (setup/hold), INTERCONNECT (SPEF).

use vyges_sta_si::{liberty::Lib, netlist, sdf, spef::Spef};

#[test]
fn inv_chain_iopath_and_spef_interconnect() {
    let lib = Lib::load("examples/top/cells.lib").unwrap();
    let nl = netlist::load("examples/top/top.v").unwrap();
    let sp = Spef::load("examples/top/top.spef").unwrap();
    let out = sdf::emit("top", &nl, &lib, Some(&sp));

    assert!(out.starts_with("(DELAYFILE"));
    assert!(out.contains("(SDFVERSION \"3.0\")"));
    // one IOPATH A->Y per inverter (g1/g2/g3)
    assert!(out.contains("(INSTANCE g1)") && out.contains("(INSTANCE g3)"));
    assert!(out.matches("(IOPATH A Y").count() >= 3, "one IOPATH per INV:\n{out}");
    // interconnect comes from the SPEF (driver output pin -> sink input pin)
    assert!(out.contains("(INTERCONNECT g1/Y g2/A"), "spef interconnect n1:\n{out}");
    assert!(out.contains("(INTERCONNECT g2/Y g3/A"), "spef interconnect n2:\n{out}");
    assert!(out.trim_end().ends_with(")"));
}

#[test]
fn dff_emits_setup_hold_timingcheck() {
    let lib = Lib::load("examples/seq/seq.lib").unwrap();
    let nl = netlist::load("examples/seq/seq.v").unwrap();
    let out = sdf::emit("seq", &nl, &lib, None);

    assert!(out.contains("(IOPATH CK Q"), "DFF clk->Q IOPATH:\n{out}");
    assert!(out.contains("(SETUP D (posedge CK)"), "setup check:\n{out}");
    assert!(out.contains("(HOLD D (posedge CK)"), "hold check:\n{out}");
    // no SPEF given -> no interconnect block
    assert!(!out.contains("(INTERCONNECT"), "no SPEF -> no interconnect");
}
