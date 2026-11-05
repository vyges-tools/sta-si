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
    let o = cell.outputs().next().expect("an output pin");
    let arc = o.arcs.first().expect("an arc");
    // Per EDGE. The INV is negative unate: its output RISE is driven by an input FALL, so the
    // rise delay is the rise table at the input's FALLING slew and the rise load, and the
    // fall delay the fall table at the RISING slew and the fall load. One slew for both (the
    // worst lane's, as this writer used to take) is right for one edge only: against a
    // reference timer's SDF on sky130 that put inverter and xor fall delays 10-25 % low.
    let a = t.pin("g2/A").expect("g2/A");
    let y = t.pin("g2/Y").expect("g2/Y");
    let (rise_in, fall_in) = (t.slew_edge(a, true), t.slew_edge(a, false));
    assert!(
        (rise_in - fall_in).abs() > 1e-6,
        "fixture must give g2/A different rise and fall slews, or this test cannot fail ({rise_in} vs {fall_in})"
    );
    let (r, f) = (
        arc.cell_rise.lookup(fall_in, t.load_edge(y, true)),
        arc.cell_fall.lookup(rise_in, t.load_edge(y, false)),
    );
    let want = format!(
        "(IOPATH {} {} ({r:.6}:{r:.6}:{r:.6}) ({f:.6}:{f:.6}:{f:.6}))",
        arc.related_pin, o.name,
    );
    assert!(out.contains(&want), "g2 IOPATH should be the timer's numbers.\nwant {want}\ngot:\n{out}");
    let load = t.load(y);

    // and the load must actually carry the SPEF wire cap — otherwise the above could pass
    // against a pin-cap-only load and the regression would sail straight back in.
    let pin_caps_only: f64 = cell.input_cap("A");
    assert!(
        load > pin_caps_only,
        "timer load {load} should exceed the bare sink pin cap {pin_caps_only} (wire cap missing?)"
    );
}

#[test]
fn timing_checks_carry_their_data_edge_when_rise_and_fall_differ() {
    // Reference rule (`SdfWriter::writeEdgeCheck`): a check whose rise and fall constraints are
    // equal is one line with a bare data port; one whose values differ is written once per data
    // edge, `(SETUP (posedge D) ...)` and `(SETUP (negedge D) ...)`, each with its own value.
    // This writer used to write one line holding the worse of the two, so the other edge was
    // stated wrong in every SDF of a real library (sky130 dfrtp setup: 0.14 rise, 0.29 fall).
    let text = std::fs::read_to_string("examples/seq/seq.lib").unwrap();
    // the setup group's fall_constraint is the first `values ( "0.05" )` after its rise one
    let at = text.find("fall_constraint (s) { index_1 (\"0.01\"); index_2 (\"0.01\"); values ( \"0.05\" ); }").unwrap();
    let text = format!("{}{}", &text[..at], text[at..].replacen("values ( \"0.05\" )", "values ( \"0.09\" )", 1));
    let lib = Lib::parse(&text).unwrap();
    let nl = netlist::load("examples/seq/seq.v").unwrap();
    let job = StaJob::load("examples/seq/seq.sta").unwrap();
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    let out = sdf::emit("seq", &nl, &lib, None, &t);

    assert!(out.contains("(SETUP (posedge D) (posedge CK) (0.050000:0.050000:0.050000))"), "rise setup:\n{out}");
    assert!(out.contains("(SETUP (negedge D) (posedge CK) (0.090000:0.090000:0.090000))"), "fall setup:\n{out}");
    assert!(!out.contains("(SETUP D "), "no edge-less setup once rise and fall differ:\n{out}");
    // hold stayed symmetric: still one line, no data edge
    assert!(out.contains("(HOLD D (posedge CK) (0.020000:0.020000:0.020000))"), "hold:\n{out}");
}

#[test]
fn timing_check_clock_edge_is_the_constraints_own() {
    // `setup_falling` / `hold_falling` check at the clock's FALLING edge: the reference writes
    // the clock port as `(negedge CLK)` (`writeCheck` prints `sdfEdge(arc->fromEdge())`). This
    // writer used to print `(posedge ...)` for every check, so a negative-edge register's
    // checks named the wrong edge.
    let lib = Lib::load("examples/fall_launch/fall_launch.lib").unwrap();
    let nl = netlist::load("examples/fall_launch/fall_launch.v").unwrap();
    let job = StaJob::parse(
        "design: fall_launch\nnetlist: fall_launch.v\nlib: fall_launch.lib\nclock: clk 2.0\n",
        "examples/fall_launch",
    )
    .unwrap();
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    let out = sdf::emit("fall_launch", &nl, &lib, None, &t);
    let block = |inst: &str| {
        let at = out.find(&format!("(INSTANCE {inst})")).unwrap_or_else(|| panic!("{inst} in:\n{out}"));
        out[at..].split("  (CELL").next().unwrap().to_string()
    };
    assert!(block("n1").contains("(SETUP D (negedge CLK)"), "DFFN setup at the falling edge:\n{}", block("n1"));
    assert!(block("n1").contains("(HOLD D (negedge CLK)"), "DFFN hold at the falling edge:\n{}", block("n1"));
    assert!(!block("n1").contains("posedge"), "no rising edge on a DFFN check:\n{}", block("n1"));
    assert!(block("r2").contains("(SETUP D (posedge CLK)"), "DFF setup at the rising edge:\n{}", block("r2"));
}
