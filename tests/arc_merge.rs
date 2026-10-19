// A gate with two arcs onto one output edge, on a parasitic net (examples/arc_merge). Reference
// rules, each measured on the reference with this script:
//  * each input port keeps its own `set_input_transition` (in1 0.02, in2 0.35);
//  * the driver's slew merges over its arcs on its own — the worse for the analysis — not the
//    slew of the arc that wins the arrival (u1/Y 0.242773 on the path through B, whose own arc
//    gives 0.1685);
//  * every wire from the driver merges the same way over the arcs' waveforms (u1/Y -> u2/A
//    0.054700, slew 0.263237).
// (u2/A, a load pin, is printed only with `-fields input_pins`; its numbers are pinned by the
// golden and by the report being the reference's line for line.)
// Reference, setup: u1/Y 0.168994 (slew 0.242773), u2/A 0.223694 (0.263237), u2/Y 0.257854
// (0.030055), slack 1.742146.
//  * an output port with `set_output_delay` is hold-checked too: required = edge − output delay.
// Reference, hold: u1/Y 0.140331 (0.159597), u2/A 0.193390 (0.188296), u2/Y 0.219601 (0.019890),
// slack 0.219601.
use std::process::Command;

#[test]
fn two_arcs_merge_slew_and_wire_as_the_reference() {
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si"))
        .args(["tcl", "examples/arc_merge/arc_merge.tcl"])
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&out.stdout).to_string();
    // the reference's report: `Slew Delay Time edge pin (cell)` rows, max path then min path
    let setup = &text[text.find("Path Type: max").expect("setup path")..];
    let hold = &text[text.find("Path Type: min").expect("an output port is hold-checked")..];
    // (time, slew) of a pin's row, as printed (6 digits)
    let at = |block: &str, pin: &str| -> (String, String) {
        let row = block.lines().find(|l| l.split_whitespace().nth(4) == Some(pin)).unwrap_or_else(|| panic!("{pin} in\n{block}"));
        let f: Vec<&str> = row.split_whitespace().collect();
        (f[2].to_string(), f[0].to_string())
    };
    assert_eq!(at(setup, "in2"), ("0.000000".into(), "0.350000".into()));
    assert_eq!(at(setup, "u1/Y"), ("0.168994".into(), "0.242773".into()));
    assert_eq!(at(setup, "u2/Y"), ("0.257854".into(), "0.030055".into()));
    assert!(setup.contains("1.742146   slack (MET)"), "{setup}");

    assert_eq!(at(hold, "u1/Y"), ("0.140331".into(), "0.159597".into()));
    assert_eq!(at(hold, "u2/Y"), ("0.219601".into(), "0.019890".into()));
    assert!(hold.contains("0.219601   slack (MET)"), "{hold}");
}
