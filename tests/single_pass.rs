// Rules (`GraphDelayCalc::findDelays`: one levelized pass; per driver, `gateDelay` per arc, then
// that driver's `loadDelaySlew` for its net's loads at the driver's final slew). With no Miller
// increment (`miller ≤ 1`) a net's wires need only its own driver's slew, so the single pass
// computes each net's wires when its driver pops and runs no fixed point. It must land where the
// Jacobi fixed point it replaces converges (that one stops at a 1e-9 ns per-arc change), on a
// chain where the wires feed the next cell's slew: a port with no driving waveform behind an RC
// net (its wires read the DRIVER SLEW handed over at the pop), and a cell driver behind an RC
// net (its wires read the driver's waveforms, recorded by its own arcs before it pops).
use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::netlist;
use vyges_sta_si::spef::Spef;
use vyges_sta_si::sta::{analyze, with_fixed_point};

const SPEF: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 n1
*2 n2
*3 u1
*4 u2
*5 u3
*6 a
*D_NET *6 30.000000
*CONN
*P *6 I
*I *3:A I
*CAP
1 *6:1 30.000000
*RES
1 *6 *6:1 3000.000000
2 *6:1 *3:A 10.000000
*END
*D_NET *1 62.000000
*CONN
*I *3:Y O
*I *4:A I
*CAP
1 *3:Y 2.000000
2 *1:1 60.000000
*RES
1 *3:Y *1:1 4000.000000
2 *1:1 *4:A 50.000000
*END
*D_NET *2 20.000000
*CONN
*I *4:Y O
*I *5:A I
*CAP
1 *2:1 20.000000
*RES
1 *4:Y *2:1 2000.000000
2 *2:1 *5:A 10.000000
*END
"#;

const LIB: &str = r#"
library (d) {
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.08, 2.00", "0.30, 2.60" ); }
        cell_fall (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.07, 1.80", "0.28, 2.40" ); }
        rise_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.10, 0.50" ); }
        fall_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.09, 0.48" ); }
      }
    }
  }
}
"#;

const NL: &str = "module top ( a, y ); input a; output y; wire n1, n2;\n\
                  INV u1 ( .A(a), .Y(n1) ); INV u2 ( .A(n1), .Y(n2) ); INV u3 ( .A(n2), .Y(y) ); endmodule";

fn job(miller: f64) -> StaJob {
    StaJob {
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "top".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 5.0,
        clocks: vec![],
        input_slew: 0.02,
        input_slews: Vec::new(),
        output_load: 0.005,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller,
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions: vec![],
        async_groups: vec![],
        clock_groups: vec![],
        ideal_clocks: Default::default(),
        clock_waveforms: Default::default(),
        driving_cells: Default::default(),
        crpr: true,
        pba: false,
        input_delay: 0.0,
        output_delay: 0.0,
        output_delay_declared: false,
        unconstrained_outputs: false,
        io_input_delays: vec![],
        io_output_delays: vec![],
        io_input_all_but: vec![],
        io_output_all_but: vec![],
        case_values: Vec::new(),
        disabled_timing: Vec::new(),
        data_check_cmds: Vec::new(),
        max_time_borrow: Vec::new(),
        setup_uncertainty: 0.0,
        hold_uncertainty: 0.0,
        clock_uncertainty: Vec::new(),
        sdc: None,
        metadata: None,
        base_dir: String::new(),
    }
}

#[test]
fn the_single_pass_lands_where_the_fixed_point_converges() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let spef = Spef::parse(SPEF);
    let pass = analyze(&nl, &lib, &job(1.0), Some(&spef)).unwrap();
    let fixed = with_fixed_point(|| analyze(&nl, &lib, &job(1.0), Some(&spef)).unwrap());
    let near = |a: f64, b: f64, what: &str| assert!(a == b || (a - b).abs() <= 1e-6, "{what}: pass {a} fixed point {b}");
    near(pass.wns, fixed.wns, "wns");
    near(pass.whs, fixed.whs, "whs");
    assert_eq!(pass.worst_path.len(), fixed.worst_path.len());
    for (p, f) in pass.worst_path.iter().zip(&fixed.worst_path) {
        near(p.arrival, f.arrival, &format!("{} arrival", p.label));
        near(p.slew, f.slew, &format!("{} slew", p.label));
    }
    for (p, f) in pass.worst_hold_path.iter().zip(&fixed.worst_hold_path) {
        near(p.arrival, f.arrival, &format!("{} hold arrival", p.label));
        near(p.slew, f.slew, &format!("{} hold slew", p.label));
    }
    // the wires matter here: without the SPEF every arrival is earlier
    let ideal = analyze(&nl, &lib, &job(1.0), None).unwrap();
    assert!(pass.wns < ideal.wns - 0.05, "the RC nets delay the path: {} vs {}", pass.wns, ideal.wns);
}

// A Miller multiplier above 1 keeps the fixed point (its windows read other nets' arrivals): the
// switch changes nothing there.
#[test]
fn a_miller_increment_keeps_the_fixed_point() {
    let nl = netlist::parse(NL).unwrap();
    let lib = Lib::parse(LIB).unwrap();
    let spef = Spef::parse(SPEF);
    let a = analyze(&nl, &lib, &job(2.0), Some(&spef)).unwrap();
    let b = with_fixed_point(|| analyze(&nl, &lib, &job(2.0), Some(&spef)).unwrap());
    assert_eq!(a.wns.to_bits(), b.wns.to_bits());
    assert_eq!(a.whs.to_bits(), b.whs.to_bits());
}

// The EARLY pass, witnessed by a hold check: r1/Q drives an RC net into g1, g1 an RC net into r2/D,
// so the hold slack at r2/D reads the early wires at the early driver slews. g1's B arc (from r0) is
// fast with a sharp slew, its A arc slow and soft: the early analysis takes B's slew and waveform,
// the late one A's — the early wires differ from the late ones. The pass must land where
// the early fixed point converges.
const LIB_FF: &str = r#"
library (d) {
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.08, 2.00", "0.30, 2.60" ); }
        cell_fall (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.07, 1.80", "0.28, 2.40" ); }
        rise_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.10, 0.50" ); }
        fall_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.09, 0.48" ); }
      }
    }
  }
  cell (ND2) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (B) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      function : "!(A&B)";
      timing () {
        related_pin : "A";
        timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.08, 2.00", "0.30, 2.60" ); }
        cell_fall (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.07, 1.80", "0.28, 2.40" ); }
        rise_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.10, 0.50" ); }
        fall_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.09, 0.48" ); }
      }
      timing () {
        related_pin : "B";
        timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.05, 0.90", "0.15, 1.20" ); }
        cell_fall (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.04, 0.80", "0.14, 1.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.01, 0.08", "0.03, 0.12" ); }
        fall_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.01, 0.07", "0.03, 0.11" ); }
      }
    }
  }
  cell (DFF) {
    ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
    pin (CK) { direction : input; clock : true; capacitance : 0.001; }
    pin (D) {
      direction : input;
      capacitance : 0.001;
      timing () {
        related_pin : "CK";
        timing_type : setup_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.05" ); }
      }
      timing () {
        related_pin : "CK";
        timing_type : hold_rising;
        rise_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
        fall_constraint (s) { index_1 ("0.01"); index_2 ("0.01"); values ( "0.02" ); }
      }
    }
    pin (Q) {
      direction : output;
      timing () {
        related_pin : "CK";
        timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.10, 1.50", "0.20, 1.80" ); }
        cell_fall (t)       { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.09, 1.40", "0.19, 1.70" ); }
        rise_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.10, 0.50" ); }
        fall_transition (t) { index_1 ("0.01, 0.40"); index_2 ("0.001, 0.20"); values ( "0.03, 0.30", "0.09, 0.48" ); }
      }
    }
  }
}
"#;

const NL_FF: &str = "module top ( clk, din, dout ); input clk, din; output dout; wire q0, q1, n1, q2;\n\
                     DFF r1 ( .CK(clk), .D(din), .Q(q1) ); DFF r0 ( .CK(clk), .D(din), .Q(q0) ); ND2 g1 ( .A(q1), .B(q0), .Y(n1) );\n\
                     DFF r2 ( .CK(clk), .D(n1), .Q(q2) ); INV g2 ( .A(q2), .Y(dout) ); endmodule";

const SPEF_FF: &str = r#"
*SPEF "IEEE 1481-1999"
*C_UNIT 1 FF
*R_UNIT 1 OHM
*NAME_MAP
*1 q1
*2 n1
*3 r1
*4 g1
*5 r2
*D_NET *1 42.000000
*CONN
*I *3:Q O
*I *4:A I
*CAP
1 *3:Q 2.000000
2 *1:1 40.000000
*RES
1 *3:Q *1:1 3000.000000
2 *1:1 *4:A 20.000000
*END
*D_NET *2 31.000000
*CONN
*I *4:Y O
*I *5:D I
*CAP
1 *4:Y 1.000000
2 *2:1 30.000000
*RES
1 *4:Y *2:1 2500.000000
2 *2:1 *5:D 20.000000
*END
"#;

#[test]
fn the_early_pass_lands_where_the_early_fixed_point_converges() {
    let nl = netlist::parse(NL_FF).unwrap();
    let lib = Lib::parse(LIB_FF).unwrap();
    let spef = Spef::parse(SPEF_FF);
    let pass = analyze(&nl, &lib, &job(1.0), Some(&spef)).unwrap();
    let fixed = with_fixed_point(|| analyze(&nl, &lib, &job(1.0), Some(&spef)).unwrap());
    assert!(pass.whs.is_finite() && pass.hold_endpoints > 0, "a hold check is timed: whs {}", pass.whs);
    let near = |a: f64, b: f64, what: &str| assert!(a == b || (a - b).abs() <= 1e-6, "{what}: pass {a} fixed point {b}");
    near(pass.whs, fixed.whs, "whs");
    near(pass.wns, fixed.wns, "wns");
    // every hold endpoint, r2/D (behind both RC nets) among them
    assert_eq!(pass.hold_slacks.len(), fixed.hold_slacks.len());
    assert!(pass.hold_slacks.len() >= 2, "r1/D and r2/D are hold endpoints");
    for ((pp, ps), (fp, fs)) in pass.hold_slacks.iter().zip(&fixed.hold_slacks) {
        assert_eq!(pp, fp);
        near(*ps, *fs, &format!("{pp:?} hold slack"));
    }
}
