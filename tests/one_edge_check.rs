// SPDX-License-Identifier: Apache-2.0
//! A check exists only on the data edges its library gives a table for.
//!
//! **The rule.** The reference builds one check arc per data edge that has a model
//! (`LibertyBuilder::makeFromTransitionArcs`): a `recovery_rising` group with a `rise_constraint`
//! and no `fall_constraint` checks the rising data edge and nothing else. sky130's reset pins are
//! written that way — the release of an active-low reset is checked, its assertion is not.
//! Read as a zero constraint, the missing table invented a check on the other edge.
//!
//! **Witness.** `examples/one_edge_check`, the reference at its pin, report identical line for
//! line: recovery 1.3400 (the rising reset, 0.55 + 0.11 against 2.0), removal 0.4800. The
//! falling reset arrives later (0.80) and, checked against an invented zero, read 1.2000.
//! Found on sky130 flow designs, whose reported recovery path was a falling reset at 0.0000.
use std::process::Command;

#[test]
fn a_reset_is_checked_on_its_release_edge_only() {
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si"))
        .args(["tcl", "examples/one_edge_check/one_edge.tcl"])
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout).to_string();
    assert!(text.contains("Endpoint: r0 (recovery check against rising-edge clock clk)"), "{text}");
    assert!(text.contains("  -0.1100    1.8900   library recovery time"), "{text}");
    assert!(text.contains("             1.3400   slack (MET)"), "{text}");
    assert!(text.contains("             0.4800   slack (MET)"), "{text}");
    assert!(!text.contains(" v rb ") && !text.contains(" v r0/RESET_B"), "the falling reset is not checked:\n{text}");
}
