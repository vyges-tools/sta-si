// CRPR (clock-reconvergence pessimism removal). With OCV derates, the launch
// clock (late) and capture clock (early) derive the SHARED clock path two ways —
// unphysical pessimism. When both flops sit behind the same clock buffer the whole
// clock path is common, so CRPR must cancel it entirely: the buffered design then
// matches the zero-latency design under the same derates.
use vyges_sta_si::engine::analyze_inputs;
use vyges_sta_si::job::StaJob;

// flop Q tables are slew-independent (rows equal) so only the OCV derate — not a
// clock-slew change — separates the buffered and unbuffered cases.
const LIB: &str = r#"
library (crpr) {
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
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
    }
    pin (Q) {
      direction : output;
      timing () {
        related_pin : "CK";
        timing_type : rising_edge;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
}
"#;

// no clock buffer: both flops directly on clk (zero insertion delay)
const NL_FLAT: &str = "module c ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       DFF r1 ( .CK(clk), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(clk), .D(n1), .Q(q2) );\n\
                       endmodule";

// both flops behind the SAME 2-inverter clock buffer -> fully shared clock path
const NL_SHARED: &str = "module c ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1, ck1, ckg;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       INV cb1 ( .A(clk), .Y(ck1) );\n\
                       INV cb2 ( .A(ck1), .Y(ckg) );\n\
                       DFF r1 ( .CK(ckg), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(ckg), .D(n1), .Q(q2) );\n\
                       endmodule";

fn job() -> StaJob {
    StaJob {
        design: "c".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
        clocks: vec![],
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.10, // OCV: clock buffer is late ×1.10 on launch...
        early_derate: 0.90, // ...and early ×0.90 on capture -> shared-path pessimism
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
        xtalk_window: 0.0,
        scenarios: vec![],
        exceptions: vec![],
        crpr: true,
        base_dir: String::new(),
    }
}

#[test]
fn crpr_removes_shared_clock_pessimism() {
    let flat = analyze_inputs(NL_FLAT, LIB, &job()).unwrap();

    let mut off = job();
    off.crpr = false;
    let shared_off = analyze_inputs(NL_SHARED, LIB, &off).unwrap();
    let shared_on = analyze_inputs(NL_SHARED, LIB, &job()).unwrap();

    // without CRPR the shared clock path is derived late+early -> false skew that
    // eats setup slack relative to the zero-latency design.
    assert!(shared_off.wns < flat.wns, "off {} should be < flat {}", shared_off.wns, flat.wns);
    // CRPR adds it back -> the fully-shared clock cancels and slack matches flat.
    assert!(
        (shared_on.wns - flat.wns).abs() < 1e-9,
        "CRPR did not fully cancel shared clock: on {} vs flat {}",
        shared_on.wns,
        flat.wns
    );
    // and CRPR is a credit, never a penalty
    assert!(shared_on.wns > shared_off.wns, "CRPR should relax: {} !> {}", shared_on.wns, shared_off.wns);
}

#[test]
fn crpr_also_relaxes_hold() {
    let mut off = job();
    off.crpr = false;
    let shared_off = analyze_inputs(NL_SHARED, LIB, &off).unwrap();
    let shared_on = analyze_inputs(NL_SHARED, LIB, &job()).unwrap();
    // hold uses late capture / early launch on the shared path -> CRPR relaxes it too
    assert!(shared_on.whs >= shared_off.whs, "CRPR hold {} !>= {}", shared_on.whs, shared_off.whs);
}
