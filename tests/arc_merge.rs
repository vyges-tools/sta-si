// A gate with two arcs onto one output edge, on a parasitic net (examples/arc_merge). Reference
// rules, each measured on the reference with this script:
//  * each input port keeps its own `set_input_transition` (in1 0.02, in2 0.35);
//  * the driver's slew merges over its arcs on its own — the worse for the analysis — not the
//    slew of the arc that wins the arrival (u1/Y 0.242773 on the path through B, whose own arc
//    gives 0.1685);
//  * every wire from the driver merges the same way over the arcs' waveforms (u1/Y -> u2/A
//    0.054700, slew 0.263237).
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
    let setup = &text[text.find("max (Setup)").expect("setup path")..];
    // `arrival slew node` rows of the setup path
    let at = |pin: &str| -> (String, String) {
        let row = setup.lines().find(|l| l.split_whitespace().nth(2) == Some(pin)).unwrap_or_else(|| panic!("{pin} in\n{setup}"));
        let f: Vec<&str> = row.split_whitespace().collect();
        (f[0].to_string(), f[1].to_string())
    };
    assert_eq!(at("in2"), ("0.0000".into(), "0.3500".into()));
    assert_eq!(at("u1/Y"), ("0.1690".into(), "0.2428".into()));
    assert_eq!(at("u2/A"), ("0.2237".into(), "0.2632".into()));
    assert_eq!(at("u2/Y"), ("0.2579".into(), "0.0301".into()));
    assert!(setup.contains("slack 1.7421"), "{setup}");

    let hold = &text[text.find("min (Hold)").expect("an output port is hold-checked")..];
    let at = |pin: &str| -> (String, String) {
        let row = hold.lines().find(|l| l.split_whitespace().nth(2) == Some(pin)).unwrap_or_else(|| panic!("{pin} in\n{hold}"));
        let f: Vec<&str> = row.split_whitespace().collect();
        (f[0].to_string(), f[1].to_string())
    };
    assert_eq!(at("u1/Y"), ("0.1403".into(), "0.1596".into()));
    assert_eq!(at("u2/A"), ("0.1934".into(), "0.1883".into()));
    assert_eq!(at("u2/Y"), ("0.2196".into(), "0.0199".into()));
    assert!(hold.contains("slack 0.2196"), "{hold}");
}
