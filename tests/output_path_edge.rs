// An output port is scored per data edge like a register, so its reported path is the edge that
// set the slack, followed lane by lane. Collapsed, each node showed its own latest edge: on the
// reference's gcd test the path to `resp_msg[15]` printed `_414_/Q` rising (0.8028) inside a
// path the reference reports falling at that pin (0.7985), the slack equal either way.
//
// Here Q rises late (0.5) and falls early (0.1); the inverter's output rises slowly (1.0) and
// falls fast (0.1). The worst edge at `y` is the RISE (Q fall 0.1 + 1.0 = 1.1, against Q rise
// 0.5 + 0.1 = 0.6), so the path must show Q at 0.1 — its falling edge — not its latest, 0.5.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (e) {
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) { direction : input; capacitance : 0.001; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.5, 0.5", "0.5, 0.5" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "!A";
      timing () { related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "1.0, 1.0", "1.0, 1.0" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
}
"#;

const NL: &str = "module t ( clk, d, y ); input clk, d; output y; wire q;\n\
                  DFF r1 ( .CK(clk), .D(d), .Q(q) );\n\
                  INV g1 ( .A(q), .Y(y) );\n\
                  endmodule";

#[test]
fn an_output_path_is_the_edge_that_set_its_slack() {
    let job = StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    let rep = analyze_inputs(NL, LIB, &job).unwrap();
    assert_eq!(rep.worst_endpoint, "y");
    assert!((rep.wns - (5.0 - 1.1)).abs() < 1e-6, "slack from the rising output edge: {}", rep.wns);
    let q = rep.worst_path.iter().find(|p| p.label == "r1/Q").expect("r1/Q on the path");
    assert!((q.arrival - 0.1).abs() < 1e-6, "Q on the falling edge that feeds y's rise, got {}", q.arrival);
}

// Rule: `create_clock … [get_ports *clk*]` clocks every port the pattern matches (the reference's
// `get_ports`). Kept literal, `*clk*` named no port and every register went unchecked.
#[test]
fn a_clock_source_pattern_is_the_ports_it_matches() {
    let mut job = StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    job.clocks = vec![("clk".into(), "*cl*".into(), 5.0)];
    job.clock_port = "*cl*".into();
    let rep = analyze_inputs(NL, LIB, &job).unwrap();
    assert_eq!(rep.worst_endpoint, "y", "the register launches once its clock is found");
    assert!((rep.wns - (5.0 - 1.1)).abs() < 1e-6, "{}", rep.wns);
}

// Rule (reference scripts): an output with no output delay is UNCONSTRAINED — it ends no path.
// The `.sta` format keeps checking it at the default delay; the Tcl adapter sets this.
#[test]
fn an_output_without_an_output_delay_ends_no_path_in_reference_mode() {
    let mut job = StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    assert_eq!(analyze_inputs(NL, LIB, &job).unwrap().worst_endpoint, "y", ".sta: checked at the default");
    job.unconstrained_outputs = true;
    let rep = analyze_inputs(NL, LIB, &job).unwrap();
    // r1/D is fed by `d`, itself unconstrained (no input delay): nothing is left to check
    assert_eq!(rep.endpoints, 0, "{}", rep.worst_endpoint);
}
