// SPDX-License-Identifier: Apache-2.0
//! A timing constraint the file holds and this timer does not apply is never dropped silently:
//! the run warns, once per kind and with its count (`STA-SDC-NOT-APPLIED`). A clock sense is the
//! case that prompted it: read and kept by the SDC reader, applied by no timer here.
use std::process::Command;

#[test]
fn a_clock_sense_not_applied_is_a_warning_with_its_count() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_not_applied");
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["one_edge.lib", "one_edge.v"] {
        std::fs::copy(format!("examples/one_edge_check/{f}"), dir.join(f)).unwrap();
    }
    std::fs::write(
        dir.join("t.tcl"),
        "read_liberty one_edge.lib\nread_verilog one_edge.v\nlink_design one_edge\n\
         create_clock -name clk -period 2.0 [get_ports clk]\nset_input_delay 0.5 -clock clk [get_ports rst_n]\n\
         set_sense -stop_propagation [get_pins r0/Q]\nset_sense -stop_propagation [get_pins r0/Q]\n\
         set_max_area 0\nreport_checks -path_delay max -digits 4\n",
    )
    .unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si")).arg("tcl").arg("t.tcl").current_dir(&dir).output().unwrap();
    let text = String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr);
    let warns: Vec<&str> = text.lines().filter(|l| l.contains("\"code\":\"STA-SDC-NOT-APPLIED\"")).collect();
    assert_eq!(warns.len(), 1, "{text}");
    assert!(warns[0].contains("\"severity\":\"warn\""), "{}", warns[0]);
    assert!(warns[0].contains("sdc: `set_sense` (2 commands): kept and written back; not applied by this timer"), "{}", warns[0]);
    // the README quotes this message
    let readme = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/README.md")).unwrap();
    let flat = readme.split_whitespace().collect::<Vec<_>>().join(" ");
    assert!(flat.contains("`` sdc: `set_sense` (2 commands): kept and written back; not applied by this timer``"), "README");
    // set_max_area moves no slack: noted with the commands outside the subset, not a warning
    assert!(text.contains("(sdc) set_max_area"), "{text}");
}

/// The same warning on a job file's run, whose constraints come from its `sdc:` file.
#[test]
fn a_job_files_sdc_warns_the_same() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_not_applied_job");
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["clk_edge.lib", "clk_edge.v"] {
        std::fs::copy(format!("examples/clk_edge/{f}"), dir.join(f)).unwrap();
    }
    std::fs::write(dir.join("t.sdc"), "create_clock -name clk_i -period 4 [get_ports clk_i]\ngroup_path -name g -to [get_ports *]\n").unwrap();
    std::fs::write(dir.join("t.sta"), "design: clk_edge\nnetlist: clk_edge.v\nlib: clk_edge.lib\nsdc: t.sdc\n").unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_vyges-sta-si")).arg("run").arg(dir.join("t.sta")).output().unwrap();
    let text = String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr);
    let warns: Vec<&str> = text.lines().filter(|l| l.contains("\"code\":\"STA-SDC-NOT-APPLIED\"")).collect();
    assert_eq!(warns.len(), 1, "{text}");
    assert!(warns[0].contains("sdc: `group_path`: kept and written back; not applied by this timer"), "{}", warns[0]);
}
