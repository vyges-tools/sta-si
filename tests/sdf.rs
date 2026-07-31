//! SDF writer: IOPATH (cell delay), TIMINGCHECK (setup/hold), INTERCONNECT (SPEF).

use vyges_sta_si::{job::StaJob, liberty::Lib, netlist, sdf, spef::Spef, sta::Timer};

#[test]
fn inv_chain_iopath_and_spef_interconnect() {
    let lib = Lib::load("examples/top/cells.lib").unwrap();
    let nl = netlist::load("examples/top/top.v").unwrap();
    let sp = Spef::load("examples/top/top.spef").unwrap();
    let job = StaJob::load("examples/top/top.sta").unwrap();
    let t = Timer::build(&nl, &lib, &job, Some(&sp)).unwrap();
    let out = sdf::emit("top", &nl, &lib, Some(&sp), &t);

    assert!(out.starts_with("(DELAYFILE"));
    assert!(out.contains("(SDFVERSION \"3.0\")"));
    // one IOPATH A->Y per inverter (g1/g2/g3)
    assert!(out.contains("(INSTANCE g1)") && out.contains("(INSTANCE g3)"));
    assert!(
        out.matches("(IOPATH A Y").count() >= 3,
        "one IOPATH per INV:\n{out}"
    );
    // interconnect comes from the SPEF (driver output pin -> sink input pin)
    assert!(
        out.contains("(INTERCONNECT g1/Y g2/A"),
        "spef interconnect n1:\n{out}"
    );
    assert!(
        out.contains("(INTERCONNECT g2/Y g3/A"),
        "spef interconnect n2:\n{out}"
    );
    assert!(out.trim_end().ends_with(")"));
}

#[test]
fn dff_emits_setup_hold_timingcheck() {
    let lib = Lib::load("examples/seq/seq.lib").unwrap();
    let nl = netlist::load("examples/seq/seq.v").unwrap();
    let job = StaJob::load("examples/seq/seq.sta").unwrap();
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    let out = sdf::emit("seq", &nl, &lib, None, &t);

    assert!(out.contains("(IOPATH CK Q"), "DFF clk->Q IOPATH:\n{out}");
    assert!(out.contains("(SETUP D (posedge CK)"), "setup check:\n{out}");
    assert!(out.contains("(HOLD D (posedge CK)"), "hold check:\n{out}");
    // no SPEF given -> no interconnect block
    assert!(!out.contains("(INTERCONNECT"), "no SPEF -> no interconnect");
}

#[test]
fn iopath_delays_are_the_timers_own_numbers() {
    // The contract this file exists to enforce. The writer used to re-derive delay from a
    // FIXED nominal slew and a load that summed sink pin caps while ignoring SPEF wire
    // capacitance — both optimistic, and on a real sky130 block that came out ~39% under
    // OpenSTA's signoff SDF while the timer agreed to ~3.5%. SDF feeds back-annotated
    // gate-level simulation, so optimistic delays let a design pass a sim it should fail.
    //
    // So: an emitted IOPATH must equal a Liberty lookup at the TIMER's propagated input slew
    // and the TIMER's load. Not "close to" — equal, because it should be the same call.
    let lib = Lib::load("examples/top/cells.lib").unwrap();
    let nl = netlist::load("examples/top/top.v").unwrap();
    let sp = Spef::load("examples/top/top.spef").unwrap();
    let job = StaJob::load("examples/top/top.sta").unwrap();
    let t = Timer::build(&nl, &lib, &job, Some(&sp)).unwrap();
    let out = sdf::emit("top", &nl, &lib, Some(&sp), &t);

    let cell = lib.cells.get("INV").expect("INV");
    let o = cell.outputs().into_iter().next().expect("an output pin");
    let arc = o.arcs.first().expect("an arc");
    let islew = t.slew(t.pin("g2/A").expect("g2/A"));
    let load = t.load(t.pin("g2/Y").expect("g2/Y"));
    let want = format!(
        "(IOPATH {} {} ({:.6}:{:.6}:{:.6}) ({:.6}:{:.6}:{:.6}))",
        arc.related_pin,
        o.name,
        arc.cell_rise.lookup(islew, load),
        arc.cell_rise.lookup(islew, load),
        arc.cell_rise.lookup(islew, load),
        arc.cell_fall.lookup(islew, load),
        arc.cell_fall.lookup(islew, load),
        arc.cell_fall.lookup(islew, load),
    );
    assert!(out.contains(&want), "g2 IOPATH should be the timer's numbers.\nwant {want}\ngot:\n{out}");

    // and the load must actually carry the SPEF wire cap — otherwise the above could pass
    // against a pin-cap-only load and the regression would sail straight back in.
    let pin_caps_only: f64 = cell.input_cap("A");
    assert!(
        load > pin_caps_only,
        "timer load {load} should exceed the bare sink pin cap {pin_caps_only} (wire cap missing?)"
    );
}
