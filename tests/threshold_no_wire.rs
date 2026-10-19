// SPDX-License-Identifier: Apache-2.0
//! A load's slew is re-expressed at ITS library's thresholds even where no wire model exists.
//!
//! **The rule.** With no parasitics a load's wire delay is 0 and its slew the driver's; the
//! reference passes both through `thresholdAdjust` into the load library's thresholds, as it
//! does for an RC wire (`LumpedCapDelayCalc::makeResult`): the slew scaled by the two libraries'
//! slew spans, the delay shifted by the slew times the difference of their delay thresholds.
//! Found on a memory macro (10/90 slews) driving sky130 cells (20/80) before any wire existed:
//! the load slew was the macro's, and the next gate 1.7 ps late.
//!
//! **Witness.** `examples/thresholds/th_nowire.tcl` (th.tcl without its SPEF), the reference at
//! its pin (`instruments/sta-si/tcl-witness.sh … lib`, `-fields {slew input_pins}`): every node
//! of the setup path below, slack 0.7499 (0.7569 before the rule), hold 0.1378.
const SCRIPT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/thresholds/th_nowire.tcl");

#[test]
fn every_node_of_the_setup_path_is_the_references() {
    let job = vyges_sta_si::tcl::adapt(SCRIPT).unwrap().job;
    let (nl, lib, spef) = vyges_sta_si::engine::load_inputs(&job, Default::default()).unwrap();
    let t = vyges_sta_si::sta::Timer::build(&nl, &lib, &job, spef.as_ref()).unwrap();
    let r = t.report();
    // (node, arrival, slew) as the reference prints them, 4 digits
    let reference = [
        ("r1/Q", 0.1006, 0.0318),
        ("g1/A", 0.1059, 0.0424), // the slew at INV_B's thresholds; the delay shifted +0.0053
        ("g1/Y", 0.2045, 0.0346),
        ("r2/D", 0.2001, 0.0260), // back at DFF's: −0.0043
    ];
    let path: Vec<(String, String, String)> =
        r.worst_path.iter().map(|p| (p.label.clone(), format!("{:.4}", p.arrival), format!("{:.4}", p.slew))).collect();
    for (node, a, s) in reference {
        let row = (node.to_string(), format!("{a:.4}"), format!("{s:.4}"));
        assert!(path.contains(&row), "{row:?} not on the path {path:?}");
    }
    assert_eq!(format!("{:.4}", r.wns), "0.7499");
    assert_eq!(format!("{:.4}", r.whs), "0.1378");
}
