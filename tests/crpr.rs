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
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
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
const NL_FLAT: &str =
    "module c ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       DFF r1 ( .CK(clk), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(clk), .D(n1), .Q(q2) );\n\
                       endmodule";

// both flops behind the SAME 2-inverter clock buffer -> fully shared clock path
const NL_SHARED: &str =
    "module c ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1, ck1, ckg;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       INV cb1 ( .A(clk), .Y(ck1) );\n\
                       INV cb2 ( .A(ck1), .Y(ckg) );\n\
                       DFF r1 ( .CK(ckg), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(ckg), .D(n1), .Q(q2) );\n\
                       endmodule";

fn job() -> StaJob {
    StaJob {
        rc_model: "elmore".into(),
        input_delay_declared: true,
        design: "c".into(),
        netlist: "x".into(),
        libs: vec!["x".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 2.0,
        clocks: vec![],
        input_slew: 0.02,
        input_slews: Vec::new(),
        output_load: 0.005,
        late_derate: 1.10,  // OCV: clock buffer is late ×1.10 on launch...
        early_derate: 0.90, // ...and early ×0.90 on capture -> shared-path pessimism
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
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
fn crpr_removes_shared_clock_pessimism() {
    let flat = analyze_inputs(NL_FLAT, LIB, &job()).unwrap();

    let mut off = job();
    off.crpr = false;
    let shared_off = analyze_inputs(NL_SHARED, LIB, &off).unwrap();
    let shared_on = analyze_inputs(NL_SHARED, LIB, &job()).unwrap();

    // without CRPR the shared clock path is derived late+early -> false skew that
    // eats setup slack relative to the zero-latency design.
    assert!(
        shared_off.wns < flat.wns,
        "off {} should be < flat {}",
        shared_off.wns,
        flat.wns
    );
    // CRPR adds it back -> the fully-shared clock cancels and slack matches flat (to float: the
    // reference takes each path's spread in float, `CheckCrpr::crprArrivalDiff`).
    assert!(
        (shared_on.wns - flat.wns).abs() < 1e-6,
        "CRPR did not fully cancel shared clock: on {} vs flat {}",
        shared_on.wns,
        flat.wns
    );
    // and CRPR is a credit, never a penalty
    assert!(
        shared_on.wns > shared_off.wns,
        "CRPR should relax: {} !> {}",
        shared_on.wns,
        shared_off.wns
    );
}

#[test]
fn crpr_also_relaxes_hold() {
    let mut off = job();
    off.crpr = false;
    let shared_off = analyze_inputs(NL_SHARED, LIB, &off).unwrap();
    let shared_on = analyze_inputs(NL_SHARED, LIB, &job()).unwrap();
    // hold uses late capture / early launch on the shared path -> CRPR relaxes it too
    assert!(
        shared_on.whs >= shared_off.whs,
        "CRPR hold {} !>= {}",
        shared_on.whs,
        shared_off.whs
    );
}

// The credit is each path's spread on its OWN edge at the common pin (`CheckCrpr::findCrpr1`,
// `same_pin` mode): its arrival against the other analysis's arrival of that edge, the smaller of
// the launch's and the capture's. With no derate the two analyses agree edge for edge, so a clock
// buffer far slower rising (0.30) than falling (0.10) leaves NOTHING to credit. Taken from the
// node's collapsed arrivals (latest edge late, earliest early) it credited the 0.20 rise/fall
// spread: post-CTS sky130 reports read 0.0642 of pessimism removal where the reference reads 0.
const LIB_ASYM: &str = r#"
library (crpr_asym) {
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (CLKBUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (X) {
      direction : output;
      timing () {
        related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.30, 0.30", "0.30, 0.30" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
}
"#;

const NL_ASYM: &str =
    "module c ( clk, din, dout ); input clk, din; output dout; wire q1, q2, n1, ckg;\n\
                       INV ob ( .A(din), .Y(dout) );\n\
                       CLKBUF cb ( .A(clk), .X(ckg) );\n\
                       DFF r1 ( .CK(ckg), .D(q2), .Q(q1) );\n\
                       INV g1 ( .A(q1),   .Y(n1) );\n\
                       DFF r2 ( .CK(ckg), .D(n1), .Q(q2) );\n\
                       endmodule";

#[test]
fn a_rise_fall_spread_on_the_common_clock_is_not_credited() {
    let lib = format!("{}{}", &LIB[..LIB.rfind('}').unwrap()], &LIB_ASYM[LIB_ASYM.find("cell (CLKBUF)").unwrap()..]);
    let mut on = job();
    on.late_derate = 1.0;
    on.early_derate = 1.0;
    let mut off = on.clone();
    off.crpr = false;
    let (a, b) = (analyze_inputs(NL_ASYM, &lib, &on).unwrap(), analyze_inputs(NL_ASYM, &lib, &off).unwrap());
    assert!((a.wns - b.wns).abs() < 1e-9, "setup: crpr on {} off {}", a.wns, b.wns);
}

// A hold check is scored per launch that CRPR tells apart, not only from the earliest arrival.
// The reference keeps a path per CRPR clock pin at every pin and drops one only when the best
// arrival, moved by that best path's largest possible credit (`CheckCrpr::maxCrpr`: the clock
// spread at its own launching clock pin), is still earlier (`ArrivalVisitor::pruneCrprArrivals`).
// Here r1's own Q->D loop reaches r1/D first, with its WHOLE clock path as credit (launch and
// capture are the same pin), while r2's path arrives 0.027 later but shares only the root buffer
// with r1, so it gets 0.024 of credit instead of 0.12: r2's is the worse hold, by 0.069. Scored
// from the earliest arrival alone the check read the self loop's 0.115, optimistic. Measured on
// CoralNPU: 52.923 here against the reference's 47.237 at one register-to-register end.
const LIB_LOOP: &str = r#"
library (crpr_loop) {
  lu_table_template (s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  cell (BUF) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
  cell (DLY) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.12, 0.12", "0.12, 0.12" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
  cell (AND2) {
    pin (A) { direction : input; capacitance : 0.001; }
    pin (B) { direction : input; capacitance : 0.001; }
    pin (Y) {
      direction : output; function : "A&B";
      timing () {
        related_pin : "A"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
      timing () {
        related_pin : "B"; timing_sense : positive_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.05, 0.05", "0.05, 0.05" ); }
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
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.10, 0.10", "0.10, 0.10" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.03", "0.03, 0.03" ); }
      }
    }
  }
}
"#;

// clk -> b0 (shared) -> a1..a4 -> r1/CK ; b0 -> c1..c4 -> dly -> r2/CK. r1/D = q1 & q2.
// `with_r2` false ties r1/D's B to q1 too: the self loop alone.
fn nl_loop(with_r2: bool) -> String {
    nl_loop_dly(with_r2, false)
}

// `dly_on_r1`: the delay cell on r1's branch instead of r2's, so the self loop arrives LAST.
fn nl_loop_dly(with_r2: bool, dly_on_r1: bool) -> String {
    let b = if with_r2 { "q2" } else { "q1" };
    let (k1, k2) = if dly_on_r1 { ("k2", "c4") } else { ("a4", "k2") };
    let dly_in = if dly_on_r1 { "a4" } else { "c4" };
    format!(
        "module c ( clk ); input clk; wire k0, a1, a2, a3, a4, c1, c2, c3, c4, k2, q1, q2, n2, d1;\n\
         BUF b0 ( .A(clk), .Y(k0) );\n\
         BUF ba1 ( .A(k0), .Y(a1) ); BUF ba2 ( .A(a1), .Y(a2) ); BUF ba3 ( .A(a2), .Y(a3) ); BUF ba4 ( .A(a3), .Y(a4) );\n\
         BUF bc1 ( .A(k0), .Y(c1) ); BUF bc2 ( .A(c1), .Y(c2) ); BUF bc3 ( .A(c2), .Y(c3) ); BUF bc4 ( .A(c3), .Y(c4) );\n\
         DLY dl ( .A({dly_in}), .Y(k2) );\n\
         AND2 g ( .A(q1), .B({b}), .Y(d1) );\n\
         DFF r1 ( .CK({k1}), .D(d1), .Q(q1) );\n\
         INV i2 ( .A(q2), .Y(n2) );\n\
         DFF r2 ( .CK({k2}), .D(n2), .Q(q2) );\n\
         endmodule"
    )
}

#[test]
fn a_later_launch_with_less_crpr_credit_sets_the_hold() {
    let alone = analyze_inputs(&nl_loop(false), LIB_LOOP, &job()).unwrap();
    let both = analyze_inputs(&nl_loop(true), LIB_LOOP, &job()).unwrap();
    // the self loop alone: early r1/CK 0.54, Q 0.09, AND 0.045; capture late 0.66 less the whole
    // 0.12 spread at r1/CK; hold 0.02 (r2's own loop holds by more)
    assert!((alone.whs - 0.115).abs() < 1e-6, "self loop alone {}", alone.whs);
    // r2's launch: 0.027 later, credit only b0's 0.024 spread
    assert!((both.whs - 0.046).abs() < 1e-6, "whs {}", both.whs);
    let p = both.check_paths.iter().find(|p| !p.max).unwrap_or_else(|| panic!("no hold path: {:?}", both.check_paths.iter().map(|p| (&p.startpoint, &p.endpoint)).collect::<Vec<_>>()));
    assert_eq!(p.endpoint, "r1", "{}", p.endpoint);
    assert_eq!(p.startpoint, "r2", "the reported hold path is the launch that set the slack");
    assert!((p.slack - 0.046).abs() < 1e-6, "reported {}", p.slack);
}

// The setup mirror (same rule, `pruneCrprArrivals` on the max side): r1's own loop is now the
// LATEST arrival at r1/D, with its whole clock path (0.126) as credit, and r2's launch 0.033
// earlier shares only b0 (0.024): r2's is the worse setup, by 0.069. Scored from the latest
// arrival alone the check read the self loop's slack, optimistic.
#[test]
fn an_earlier_launch_with_less_crpr_credit_sets_the_setup() {
    use vyges_sta_si::{liberty::Lib, netlist, sta::Timer};
    let lib = Lib::parse(LIB_LOOP).unwrap();
    let r1d = |with_r2: bool| {
        let nl = netlist::parse(&nl_loop_dly(with_r2, true)).unwrap();
        let t = Timer::build(&nl, &lib, &job(), None).unwrap();
        let s = t.endpoint_slacks().into_iter().find(|(p, _)| t.pin_label(*p) == "r1/D").unwrap().1;
        let path = t.report().check_paths.iter().find(|p| p.max && p.endpoint == "r1").map(|p| (p.startpoint.clone(), p.slack));
        (s, path)
    };
    // late r1/CK 0.693, Q 0.11, AND 0.055; capture early 0.567 + 2.0 + the whole 0.126; setup 0.05
    let (alone, _) = r1d(false);
    assert!((alone - 1.785).abs() < 1e-6, "self loop alone {alone}");
    // r2: late r2/CK 0.66, credit only b0's 0.024
    let (both, path) = r1d(true);
    assert!((both - 1.716).abs() < 1e-6, "r1/D with r2 {both}");
    if let Some((start, slack)) = path {
        assert_eq!(start, "r2", "the reported setup path is the launch that set the slack");
        assert!((slack - both).abs() < 1e-9, "reported {slack}");
    }
}
