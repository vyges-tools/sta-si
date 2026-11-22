// SPDX-License-Identifier: Apache-2.0
// The end reported when several ends share the worst slack.
//
// Rule (the reference's `PathEnd::cmp` with `cmp_slack`): slacks equal within the float tolerance
// (`cmpSlack` -> `delayEqual` -> `fuzzyEqual`) are a tie, broken by the pins' path names
// (`Path::cmpPinTrClk` -> `Network::pathNameLess`): a port's instance is the top one, so every port
// comes before every instance pin; instance pins by instance name, then pin name. It is NOT the first
// end visited. Measured on the reference at the pin with three outputs of one slack declared
// `z, a, m` (interleaved with the inputs): it reports `a`, and `report_checks -format end` lists
// `a, m, z`. Visiting in node (declaration) order reported `z`.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

const LIB: &str = r#"
library (e) {
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) { direction : input; capacitance : 0.001;
      timing () { related_pin : "CK"; timing_type : setup_rising;
        rise_constraint (t) { index_1 ("0.01, 0.6"); index_2 ("0.01, 0.6"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        fall_constraint (t) { index_1 ("0.01, 0.6"); index_2 ("0.01, 0.6"); values ( "0.1, 0.1", "0.1, 0.1" ); } }
      timing () { related_pin : "CK"; timing_type : hold_rising;
        rise_constraint (t) { index_1 ("0.01, 0.6"); index_2 ("0.01, 0.6"); values ( "0.0, 0.0", "0.0, 0.0" ); }
        fall_constraint (t) { index_1 ("0.01, 0.6"); index_2 ("0.01, 0.6"); values ( "0.0, 0.0", "0.0, 0.0" ); } } }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.2, 0.2", "0.2, 0.2" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
  cell (BUF2) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "A";
      timing () { related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
  cell (BUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "A";
      timing () { related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.3, 0.3", "0.3, 0.3" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
}
"#;

fn job() -> StaJob {
    StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\ninput_delay: 0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap()
}

#[test]
fn equal_slack_outputs_report_the_least_port_name_not_the_first_declared() {
    // the reference's own probe: outputs z, a, m of one slack, ports interleaved in the header
    let nl = "module t ( z, d, a, clk, m ); output z; input d; output a; input clk; output m;\n\
              BUF u1 ( .A(d), .Y(z) );\n\
              BUF u2 ( .A(d), .Y(a) );\n\
              BUF u3 ( .A(d), .Y(m) );\n\
              endmodule";
    let rep = analyze_inputs(nl, LIB, &job()).unwrap();
    assert_eq!(rep.endpoints, 3);
    assert_eq!(rep.worst_endpoint, "a", "the least name among equal slacks");
    assert!((rep.wns - (5.0 - 0.3)).abs() < 1e-6, "{}", rep.wns);
}

#[test]
fn equal_slack_registers_report_the_least_instance_name() {
    // three registers with the same path to D from one launch (src feeds itself through b1):
    // setup and hold each tie three ways
    let nl = "module t ( clk ); input clk; wire q, w2, w1;\n\
              DFF src ( .CK(clk), .D(w1), .Q(q) );\n\
              BUF b2 ( .A(q), .Y(w2) );\n\
              DFF r2 ( .CK(clk), .D(w2) );\n\
              BUF b1 ( .A(q), .Y(w1) );\n\
              DFF r1 ( .CK(clk), .D(w1) );\n\
              endmodule";
    let rep = analyze_inputs(nl, LIB, &job()).unwrap();
    assert_eq!(rep.worst_endpoint, "r1/D", "setup: r1 before r2 and src, though both are made first");
    assert_eq!(rep.worst_hold_endpoint, "r1/D", "hold: the same order");
}

#[test]
fn a_port_is_reported_before_an_instance_pin_of_equal_slack() {
    // y (port) and r1/D (register) share a slack: 5.0 - 0.3 to y; 5.0 - 0.1 - 0.2 to r1/D
    let nl = "module t ( clk, d, y ); input clk, d; output y; wire q;\n\
              DFF a0 ( .CK(clk), .D(d), .Q(q) );\n\
              DFF r1 ( .CK(clk), .D(q) );\n\
              BUF u1 ( .A(d), .Y(y) );\n\
              endmodule";
    let rep = analyze_inputs(nl, LIB, &job()).unwrap();
    assert!((rep.wns - 4.7).abs() < 1e-6, "{}", rep.wns);
    assert_eq!(rep.worst_endpoint, "y", "a port's empty instance path sorts first");
}

#[test]
fn an_incremental_update_reports_the_same_end_on_a_tie() {
    // the incremental path's own report (`IncGraph`), after a resize to an identical cell
    use vyges_sta_si::liberty::Lib;
    use vyges_sta_si::sta::Timer;
    let nl = "module t ( z, d, a, clk, m ); output z; input d; output a; input clk; output m;\n\
              BUF u1 ( .A(d), .Y(z) );\n\
              BUF u2 ( .A(d), .Y(a) );\n\
              BUF u3 ( .A(d), .Y(m) );\n\
              endmodule";
    let nl = vyges_sta_si::netlist::parse(nl).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let mut t = Timer::build(&nl, &lib, &job(), None).unwrap();
    let (inc0, _) = t.update_stats();
    assert!(t.resize("u1", "BUF2"), "an equivalent cell");
    let rep = t.update().unwrap();
    assert_eq!(rep.worst_endpoint, "a");
    assert!(t.update_stats().0 > inc0, "served by the incremental path");
}
