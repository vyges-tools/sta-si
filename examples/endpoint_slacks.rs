// SPDX-License-Identifier: Apache-2.0
//! Diagnostic: every setup and hold endpoint's slack, by pin, for a per-endpoint comparison with a
//! reference timer's end lists (`report_checks -format end`), plus each path group's worst path.
//!
//! With `VYGES_STA_PROBE_END=<pin>` set, that end's own setup and hold path is reported too, in
//! a path group named `probe`, printed in full (every data path line, the clock terms, CRPR).
//!
//! Usage: `cargo run --release --example endpoint_slacks -- JOB.sta > ends.txt`
//! Lines: `S <pin> <setup slack>`, `H <pin> <hold slack>`, `C <max> <group> <start> <end> <slack>`,
//! and `PATH ...` blocks for the probe end. NLDM only (CCS skipped), as `run --liberty-nldm-only`.

use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;

fn main() {
    let p = std::env::args().nth(1).expect("usage: endpoint_slacks JOB.sta");
    let job = StaJob::load(&p).expect("load job");
    let (nl, lib, spef) =
        vyges_sta_si::engine::load_inputs(&job, vyges_sta_si::liberty::LibOpts { skip_ccs: true }).expect("load inputs");
    let t = Timer::build(&nl, &lib, &job, spef.as_ref()).expect("build timer");
    let r = t.report();
    eprintln!("WNS {} WHS {}", r.wns, r.whs);
    for (p, s) in &r.hold_slacks {
        println!("H {} {:.4}", t.pin_label(*p), s);
    }
    for (p, s) in t.endpoint_slacks() {
        println!("S {} {:.4}", t.pin_label(p), s);
    }
    for c in &r.check_paths {
        println!("C {} {} {} {} {:.4}", c.max, c.group, c.startpoint, c.endpoint, c.slack);
        if c.group == "probe" {
            println!("PATH {} start {} ({}) end {} ({})", if c.max { "max" } else { "min" }, c.startpoint, c.start_reason, c.endpoint, c.end_reason);
            println!("PATH src_clock {:?} src_network_delay {:.4} input_delay {:?}", c.src_clock, c.src_network_delay, c.input_delay);
            for l in &c.lines {
                println!("PATH line {:.4} slew {:.4} cap {:?} {} {}", l.time, l.slew, l.cap, if l.rise { "^" } else { "v" }, l.desc);
            }
            println!(
                "PATH arrival {:.4} tgt_clock {:?} tgt_network_delay {:.4} uncertainty {:.4} crpr {:.4} tgt_pin {:?} {} required {:.4} slack {:.4}",
                c.arrival, c.tgt_clock, c.tgt_network_delay, c.uncertainty, c.crpr, c.tgt_pin, c.margin_what, c.required, c.slack
            );
        }
    }
}
