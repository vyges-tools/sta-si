// Rule (reference `makeTristateEnableArcs`): a three-state ENABLE arc launches on ONE input edge —
// positive_unate from the rise, negative_unate from the fall — into BOTH output edges (Z->1 takes
// the rise table, Z->0 the fall table). It is not a unate pair: paired by sense, the output's fall
// would launch from the enable's fall.
//
// Here Q rises early (0.1) and falls late (0.5) into the enable of a buffer that drives when EN is
// high (`three_state : "!EN"`, enable arc positive_unate, 1.0 both edges). Both of y's edges launch
// from Q's rise: y = 0.1 + 1.0 = 1.1. Paired by sense, y's fall would take Q's fall: 0.5 + 1.0 = 1.5.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::{DriveSpec, StaJob};
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::sta::Timer;

const LIB: &str = r#"
library (z) {
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) { direction : input; capacitance : 0.001; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CK"; timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.5, 0.5", "0.5, 0.5" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
  cell (EBUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (EN) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "A"; three_state : "!EN";
      timing () { related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } }
      timing () { related_pin : "EN"; timing_type : three_state_enable; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "1.0, 1.0", "1.0, 1.0" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "1.0, 1.0", "1.0, 1.0" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.5, 0.5" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.5, 0.5" ); } } }
  }
  cell (EBUF2) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (EN) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "A"; three_state : "!EN";
      timing () { related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } }
      timing () { related_pin : "EN"; timing_type : three_state_enable; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "2.0, 2.0", "2.0, 2.0" ); }
        cell_fall (t)       { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "2.0, 2.0", "2.0, 2.0" ); }
        rise_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.5, 0.5" ); }
        fall_transition (t) { index_1 ("0.01, 0.6"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.5, 0.5" ); } } }
  }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "!A";
      timing () { related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.05, 0.5"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "1.0, 1.0" ); }
        cell_fall (t)       { index_1 ("0.05, 0.5"); index_2 ("0.001, 0.01"); values ( "0.1, 0.1", "0.1, 0.1" ); }
        rise_transition (t) { index_1 ("0.05, 0.5"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        fall_transition (t) { index_1 ("0.05, 0.5"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); } } }
  }
}
"#;

const NL: &str = "module t ( clk, d, y ); input clk, d; output y; wire q;\n\
                  DFF r1 ( .CK(clk), .D(d), .Q(q) );\n\
                  EBUF g1 ( .A(q), .EN(q), .Y(y) );\n\
                  endmodule";

#[test]
fn a_tristate_enable_arc_launches_both_output_edges_from_one_input_edge() {
    let job = StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    let rep = analyze_inputs(NL, LIB, &job).unwrap();
    assert_eq!(rep.worst_endpoint, "y");
    assert!((rep.wns - (5.0 - 1.1)).abs() < 1e-6, "y = Q rise 0.1 + enable 1.0 on both edges, got WNS {}", rep.wns);
}

// The incremental retimer pairs the edges the same way: swap the buffer for one with a 2.0 enable
// arc, take the cone-local fast path, and y = 0.1 + 2.0 (paired by sense: 0.5 + 2.0).
#[test]
fn the_incremental_retimer_pairs_a_tristate_enable_arcs_edges_the_same_way() {
    let job = StaJob::parse("design: t\nnetlist: x\nlib: x\nclock: clk 5.0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    let (nl, lib) = (netlist::parse(NL).unwrap(), Lib::parse(LIB).unwrap());
    let mut t = Timer::build(&nl, &lib, &job, None).unwrap();
    assert!(t.resize("g1", "EBUF2"));
    t.update().unwrap();
    assert_eq!(t.update_stats().0, 1, "served by the incremental path");
    assert!((t.wns() - (5.0 - 2.1)).abs() < 1e-6, "y = Q rise 0.1 + enable 2.0, got WNS {}", t.wns());
}

// So does a `set_driving_cell` on the enable pin: the port's fall comes from the enable's RISE, at
// its input transition (0.01 → port slew 0.05), not from its fall (0.6 → 0.5). The inverter after
// it turns a 0.5 slew into a 1.0 delay on y's rise; at 0.05 it is 0.1.
#[test]
fn a_driving_cells_tristate_enable_arc_drives_both_port_edges_from_one_input_edge() {
    let nl = "module d ( a, y ); input a; output y;\n INV g2 ( .A(a), .Y(y) );\n endmodule";
    let mut job = StaJob::parse("design: d\nnetlist: x\nlib: x\nclock: clk 5.0\ninput_delay: 0\noutput_delay: 0\ninput_slew: 0.05\noutput_load: 0\n", ".").unwrap();
    job.driving_cells = vec![(
        "a".into(),
        DriveSpec { cell: "EBUF".into(), from_pin: Some("EN".into()), pin: Some("Y".into()), input_transition: [0.01, 0.6] },
    )];
    let rep = analyze_inputs(nl, LIB, &job).unwrap();
    assert_eq!(rep.worst_endpoint, "y");
    assert!((rep.wns - (5.0 - 0.1)).abs() < 1e-6, "both port edges at slew 0.05, got WNS {}", rep.wns);
}
