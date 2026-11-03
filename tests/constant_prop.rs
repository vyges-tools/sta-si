// SPDX-License-Identifier: Apache-2.0
//! Constant (tie-cell) propagation.
//!
//! A net driven by a tie cell (`function : "1"` / `"0"`) can never switch, so it carries
//! no timing: nothing to launch, nothing to check. Treating the tie output as an ordinary
//! undriven node instead makes it a path source at t=0, which **manufactures hold
//! violations on wires that cannot toggle** — that was the sole cause of the only hold
//! violation sta-si reported on a block sign-off says is clean.
//!
//! The second test matters as much as the first: making constants untimed must not make
//! a gate with *one* constant input untimed as well.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/tie/");

fn timer() -> Timer {
    let job = StaJob::load(&format!("{D}tie.sta")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

#[test]
fn a_flop_fed_by_a_tie_cell_is_not_hold_checked() {
    let t = timer();
    let tied = t.pin("r_tied/D").expect("the pin still exists as a node");
    assert!(
        !t.report().hold_slacks.iter().any(|(i, _)| *i == tied),
        "a pin that can never switch must carry no hold check"
    );
    assert!(t.arrival(tied).is_infinite(), "and must be unreached, not seeded at t=0");
}

#[test]
fn a_gate_with_one_constant_input_still_times_through_the_other() {
    // The regression this could easily have introduced. g0.B is tied high; g0.A is real.
    // If constants made the whole gate untimed, the flop behind it would silently lose
    // its check — trading a false violation for a missing one, which is worse.
    let t = timer();
    let a = t.pin("g0/A").expect("g0/A");
    let x = t.pin("g0/X").expect("g0/X");
    let b = t.pin("g0/B").expect("g0/B");
    assert!(t.arrival(a).is_finite(), "the real input must be timed");
    assert!(t.arrival(b).is_infinite(), "the tied input must not be");
    assert!(t.arrival(x).is_finite(), "and the gate output must still be timed");
    assert!(t.arrival(x) > t.arrival(a), "through the real input's arc");

    let real = t.pin("r1/D").expect("r1/D");
    assert!(
        t.report().hold_slacks.iter().any(|(i, _)| *i == real),
        "the flop behind a partially-constant gate must KEEP its hold check"
    );
}

#[test]
fn the_tie_cell_does_not_become_the_worst_hold_endpoint() {
    // The end-to-end shape of the real-block bug: a tie net seeded at t=0 launched
    // nothing yet dominated WHS. This fixture does carry a genuine, marginal hold
    // violation on the REAL path — which is the useful control: it shows ordinary hold
    // checks still fire, so a passing test here cannot mean "hold checking is off".
    let t = timer();
    let worst = t.report().worst_hold_endpoint.clone();
    assert!(
        !worst.starts_with("r_tied/"),
        "worst hold endpoint should not be the tied flop: {worst:?}"
    );
    assert!(t.whs() < 0.0, "the real path here is genuinely marginal — control for the above");
    // and the tied flop contributes nothing to the totals
    let tied = t.pin("r_tied/D").unwrap();
    assert!(!t.report().hold_slacks.iter().any(|(i, _)| *i == tied));
}

// ---- case analysis: set_case_analysis / set_logic_* / netlist ties ---------------------------
//
// Reference rules (its `Sim`, then `SearchPred0` and `searchThruSimEdge`): a pin held at 0 or 1
// starts and receives no arrival; an arc whose output no longer depends on its input once the
// held pins are substituted is cut (a deselected mux input), and one that still does takes the
// substituted function's sense (an xor input held at 1 inverts the other); a constant reaches
// through combinational logic, not through a register; a pin held `rising` passes only rising
// transitions.

mod case {
    use vyges_sta_si::job::StaJob;
    use vyges_sta_si::sim::CaseSpec;
    use vyges_sta_si::sta::Timer;
    use vyges_sta_si::{liberty::Lib, netlist};

    const LIB: &str = r#"
library (k) {
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (BUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (X) { direction : output; function : "A";
      timing () { related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.3, 0.3", "0.3, 0.3"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.3, 0.3", "0.3, 0.3"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } } }
  }
  cell (MUX2) {
    pin (A0) { direction : input; capacitance : 0.001; }
    pin (A1) { direction : input; capacitance : 0.001; }
    pin (S) { direction : input; capacitance : 0.001; }
    pin (X) { direction : output; function : "(A0&!S)|(A1&S)";
      timing () { related_pin : "A0"; timing_sense : positive_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } }
      timing () { related_pin : "A1"; timing_sense : positive_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } }
      timing () { related_pin : "S"; timing_sense : non_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.2, 0.2", "0.2, 0.2"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.2, 0.2", "0.2, 0.2"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } } }
  }
  cell (XOR2) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (B) { direction : input; capacitance : 0.001; }
    pin (X) { direction : output; function : "A^B";
      timing () { related_pin : "A"; timing_sense : non_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.5, 0.5", "0.5, 0.5"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } }
      timing () { related_pin : "B"; timing_sense : non_unate;
        cell_rise (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.5, 0.5", "0.5, 0.5"); }
        cell_fall (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.1, 0.1", "0.1, 0.1"); }
        rise_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); }
        fall_transition (t) { index_1 ("0.01, 1"); index_2 ("0.001, 1"); values ("0.02, 0.02", "0.02, 0.02"); } } }
  }
}
"#;

    // a -> three buffers -> A1 (0.9 + 0.1); b -> A0 (0.1); s -> S (0.2); c, e -> xor -> z
    const NL: &str = "module k ( a, b, s, c, e, y, z ); input a, b, s, c, e; output y, z; wire n1, n2, n3;\n\
                      BUF u1 ( .A(a), .X(n1) );\n BUF u2 ( .A(n1), .X(n2) );\n BUF u3 ( .A(n2), .X(n3) );\n\
                      MUX2 m ( .A0(b), .A1(n3), .S(s), .X(y) );\n XOR2 x ( .A(c), .B(e), .X(z) );\nendmodule";

    fn timer(nl_text: &str, cases: &[(&str, &str, &str)]) -> Timer {
        let mut job = StaJob::parse("design: k\nnetlist: x\nlib: x\nclock: clk 4.0\ninput_slew: 0.02\noutput_load: 0.001\ninput_delay: 0.0\n", ".").unwrap();
        job.case_values = cases
            .iter()
            .map(|(accessor, value, name)| CaseSpec {
                cmd: "set_case_analysis".into(),
                value: value.to_string(),
                accessor: accessor.to_string(),
                narrowed: false,
                names: vec![name.to_string()],
            })
            .collect();
        let nl = netlist::parse(nl_text).unwrap();
        let lib = Lib::parse(LIB).unwrap();
        Timer::build(&nl, &lib, &job, None).unwrap()
    }

    fn at(t: &Timer, pin: &str) -> f64 {
        t.arrival(t.pin(pin).unwrap_or_else(|| panic!("{pin}")))
    }

    #[test]
    fn a_held_select_cuts_the_deselected_input_and_the_select_itself() {
        let free = timer(NL, &[]);
        assert!((at(&free, "y") - 1.0).abs() < 1e-6, "unheld: the long A1 path, {}", at(&free, "y"));
        let s0 = timer(NL, &[("get_ports", "0", "s")]);
        assert!((at(&s0, "y") - 0.1).abs() < 1e-6, "S=0: only A0 reaches y, {}", at(&s0, "y"));
        assert!(at(&s0, "s").is_infinite(), "a held port starts nothing");
        // held on the pin rather than the port: the same cut
        let pin = timer(NL, &[("get_pins", "zero", "m/S")]);
        assert!((at(&pin, "y") - 0.1).abs() < 1e-6);
        let s1 = timer(NL, &[("get_ports", "1", "s")]);
        assert!((at(&s1, "y") - 1.0).abs() < 1e-6, "S=1: only A1");
    }

    #[test]
    fn a_netlist_tie_holds_a_pin_as_case_analysis_does() {
        let tied = NL.replace(".S(s)", ".S(1'b0)");
        let t = timer(&tied, &[]);
        assert!((at(&t, "y") - 0.1).abs() < 1e-6, "S tied 0: only A0, {}", at(&t, "y"));
    }

    #[test]
    fn a_constant_reaches_through_logic_and_holds_its_output() {
        // both data inputs held: the mux output is a constant whatever S does
        let t = timer(NL, &[("get_ports", "0", "b"), ("get_ports", "0", "a")]);
        assert!(at(&t, "y").is_infinite(), "a constant output carries no arrival");
    }

    #[test]
    fn a_held_xor_input_makes_the_other_arc_unate_and_rising_passes_one_edge() {
        // c held rising: only c's rising edge enters the xor. Unheld e: non-unate, so a rising
        // c reaches z rising (0.5). e held 1: an inverter, so rising c gives z FALLING (0.1).
        let rise = timer(NL, &[("get_ports", "rising", "c")]);
        assert!((at(&rise, "z") - 0.5).abs() < 1e-6, "{}", at(&rise, "z"));
        let inv = timer(NL, &[("get_ports", "rising", "c"), ("get_ports", "1", "e")]);
        assert!((at(&inv, "z") - 0.1).abs() < 1e-6, "{}", at(&inv, "z"));
        // and without the rising hold both edges reach z again
        let both = timer(NL, &[("get_ports", "1", "e")]);
        assert!((at(&both, "z") - 0.5).abs() < 1e-6);
    }

    #[test]
    fn case_analysis_is_modelled_and_merged_from_the_sdc() {
        let sdc = vyges_sta_si::sdc::Sdc::parse("set_case_analysis 0 [get_ports s]\nset_logic_one [get_pins m/S]\n").unwrap();
        let mut job = StaJob::parse("design: k\nnetlist: x\nlib: x\nclock: clk 4.0\n", ".").unwrap();
        vyges_sta_si::job::merge_sdc_into(&sdc, &mut job);
        let cmds: Vec<&str> = job.case_values.iter().map(|c| c.cmd.as_str()).collect();
        // set_logic_* first, as the reference seeds them before the case values
        assert_eq!(cmds, ["set_logic_one", "set_case_analysis"]);
        assert_eq!(job.case_values[1].accessor, "get_ports");
        for c in ["set_case_analysis", "set_logic_one"] {
            assert!(vyges_sta_si::job::MODELLED_HERE.contains(&c), "{c} must not be listed as unmodelled");
        }
    }
}
