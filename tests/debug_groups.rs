// SPDX-License-Identifier: Apache-2.0
//! Debug groups and stage lines print to stderr and never change the report.
//!
//! **The rule** (the engines' debug-flag standard): `VYGES_DEBUG=sta-si:<group>=<level>` turns a
//! group on, as the reference's `set_debug_level sta <group> <level>` does; an old switch keeps
//! working as an alias with a deprecation note; an unknown group is a warning naming the valid
//! ones; `VYGES_STAGE_TIMES=1` adds one line per stage. None of it may change a result: the
//! report is the same byte for byte with every group on.
//!
//! **Witness.** `examples/top` (a SPEF-annotated input-to-flop path): each group's lines appear
//! when it is on and none when it is off.
use std::process::{Command, Output};

const JOB: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.sta");
const VARS: &[&str] = &["VYGES_DEBUG", "VYGES_STAGE_TIMES", "VYGES_STA_DMP_TRACE", "VYGES_STA_RC_TRACE", "VYGES_STA_RC_NET", "VYGES_STA_PROBE_END"];

fn run(env: &[(&str, &str)]) -> Output {
    let mut c = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si"));
    c.args(["run", JOB]);
    for v in VARS {
        c.env_remove(v);
    }
    for (k, v) in env {
        c.env(k, v);
    }
    let out = c.output().expect("the engine binary runs");
    assert!(out.status.success(), "exit {:?}\n{}", out.status, String::from_utf8_lossy(&out.stderr));
    out
}

fn err(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).to_string()
}

#[test]
fn groups_print_when_on_are_silent_when_off_and_never_change_the_report() {
    let off = run(&[]);
    let e = err(&off);
    for marker in ["dmp|", "drive|", "rc-trace:", "VYGES-STAGE", "stage=", "deprecated"] {
        assert!(!e.contains(marker), "off, yet `{marker}` printed:\n{e}");
    }

    let on = run(&[("VYGES_DEBUG", "sta-si:dmp_ceff=1,sta-si:vy_rc_model=1"), ("VYGES_STAGE_TIMES", "1")]);
    let e = err(&on);
    assert_eq!(on.stdout, off.stdout, "a debug group changed the report");
    assert!(e.contains("dmp|"), "dmp_ceff on, no `dmp|` line:\n{e}");
    assert!(e.contains("rc-trace:"), "vy_rc_model on, no `rc-trace:` line:\n{e}");
    for s in ["readVerilog", "readLiberty", "readSpef", "makeGraph", "findDelays", "findArrivals", "findPathEnds", "reportPathEnds"] {
        assert!(e.contains(&format!("stage={s} ")), "no stage line for {s}:\n{e}");
    }
    assert!(!e.contains("deprecated"), "{e}");
}

#[test]
fn the_old_switches_are_aliases_with_a_note() {
    let off = run(&[]);
    let on = run(&[("VYGES_STA_DMP_TRACE", "1"), ("VYGES_STA_RC_TRACE", "1")]);
    let e = err(&on);
    assert_eq!(on.stdout, off.stdout, "an old switch changed the report");
    assert!(e.contains("dmp|") && e.contains("rc-trace:"), "{e}");
    assert!(e.contains("VYGES_STA_DMP_TRACE is deprecated; use VYGES_DEBUG=sta-si:dmp_ceff=1"), "{e}");
    assert!(e.contains("VYGES_STA_RC_TRACE is deprecated; use VYGES_DEBUG=sta-si:vy_rc_model=1"), "{e}");
}

#[test]
fn an_unknown_group_is_a_warning_naming_the_valid_ones() {
    let on = run(&[("VYGES_DEBUG", "sta-si:no_such_group=1")]);
    let e = err(&on);
    assert!(e.contains("unknown debug group sta-si:no_such_group; valid groups for sta-si: dmp_ceff, vy_rc_model, vy_probe_end"), "{e}");
}

#[test]
fn describe_lists_the_groups() {
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si")).arg("--describe").output().expect("the engine binary runs");
    let text = String::from_utf8_lossy(&out.stdout);
    assert!(text.contains("\n  \"debug_groups\": {\"env\":\"VYGES_DEBUG\""), "{text}");
    assert!(text.contains("\"group\":\"dmp_ceff\"") && text.contains("\"group\":\"vy_rc_model\""), "{text}");
    assert!(text.contains("\"reference\":\"sta dmp_ceff\""), "{text}");
    assert!(text.ends_with("}\n}\n"), "one object, closed once:\n{text}");
}

/// The rule: `VYGES_STA_PROBE_END=<pin>` names an end and `vy_probe_end` prints its own check paths
/// in `report_checks`' full format, whatever the job's report (in a `.sta` job they were computed
/// and never printed). The pin alone prints only a note saying how; a pin that ends no check says
/// so; the report never changes.
#[test]
fn the_probe_end_group_prints_that_ends_path() {
    let off = run(&[]);
    let pin_only = run(&[("VYGES_STA_PROBE_END", "y")]);
    let e = err(&pin_only);
    assert!(!e.contains("probe|"), "{e}");
    assert!(e.contains("its paths print with VYGES_DEBUG=sta-si:vy_probe_end=1"), "{e}");
    let on = run(&[("VYGES_STA_PROBE_END", "y"), ("VYGES_DEBUG", "sta-si:vy_probe_end=1")]);
    let e = err(&on);
    assert_eq!(on.stdout, off.stdout, "the probe changed the report");
    for l in ["probe| Endpoint: y (output port", "probe| Path Group: probe", "probe| Path Type: max", "slack ("] {
        assert!(e.contains(l), "no `{l}`:\n{e}");
    }
    let none = run(&[("VYGES_STA_PROBE_END", "no_such_pin"), ("VYGES_DEBUG", "sta-si:vy_probe_end=1")]);
    assert!(err(&none).contains("probe| no_such_pin: no timed setup or hold check ends there"), "{}", err(&none));
}

/// An output port's HOLD path prints too, as a register's does: its `-min` output delay is a check
/// of its own (examples/top has none, so examples/nonunate_clk, whose y has both).
#[test]
fn the_probe_end_group_prints_an_output_ports_hold_path() {
    let job = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/nonunate_clk/nonunate_clk.sta");
    let mut c = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si"));
    c.args(["run", job]);
    for v in VARS {
        c.env_remove(v);
    }
    c.env("VYGES_STA_PROBE_END", "y").env("VYGES_DEBUG", "sta-si:vy_probe_end=1");
    let e = err(&c.output().expect("the engine binary runs"));
    for l in ["probe| Endpoint: y (output port", "probe| Path Type: max", "probe| Path Type: min"] {
        assert!(e.contains(l), "no `{l}`:\n{e}");
    }
}
