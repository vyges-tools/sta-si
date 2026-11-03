// SPDX-License-Identifier: Apache-2.0
//! `set_disable_timing` / `unset_disable_timing`, rule by rule, on the two-flop ring
//! `r1.Q -[g1]-> r2.D`, `r2.Q (= y) -> r1.D`. Each rule is the reference's; every one was
//! also witnessed against it on gcd (sky130hd), 16 variants, WNS/TNS/worst slack both ways.
use vyges_sta_si::job::{merge_sdc_into, StaJob};
use vyges_sta_si::sdc::Sdc;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/seq/");

fn timer(sdc: &str) -> Timer {
    let mut job = StaJob::load(&format!("{D}seq.sta")).unwrap();
    merge_sdc_into(&Sdc::parse(sdc).unwrap(), &mut job);
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let lib = Lib::load(&job.resolve(&job.libs[0])).unwrap();
    Timer::build(&nl, &lib, &job, None).unwrap()
}

/// The hold-checked data pins, by name.
fn hold_checked(t: &Timer) -> Vec<String> {
    let mut v: Vec<String> = ["r1/D", "r2/D"]
        .into_iter()
        .filter(|p| t.pin(p).is_some_and(|id| t.report().hold_slacks.iter().any(|(i, _)| *i == id)))
        .map(String::from)
        .collect();
    v.sort();
    v
}

#[test]
fn the_ring_checks_both_flops_without_a_disable() {
    assert_eq!(hold_checked(&timer("")), ["r1/D", "r2/D"]);
}

/// Rule (`DisabledPorts::isDisabled`: `all_ && !role->isTimingCheck()`): an instance's "all
/// arcs" takes out its CK->Q arc, not its checks. r1 still checks its D; nothing launches into
/// r2/D any more.
#[test]
fn an_instance_disable_spares_its_checks() {
    assert_eq!(hold_checked(&timer("set_disable_timing [get_cells r1]\n")), ["r1/D"]);
}

/// Rule: a `-from` names arcs of every role, checks included — `-from CK` takes out r1's CK->Q
/// and its CK->D checks.
#[test]
fn a_from_port_takes_the_checks_too() {
    assert!(hold_checked(&timer("set_disable_timing -from CK [get_cells r1]\n")).is_empty());
}

/// Rule (`SearchPred0::searchFrom`): nothing leaves a disabled pin, arcs into it still arrive.
/// g1/A disabled: r2/D is cut off; r1/D, fed by r2/Q, is not.
#[test]
fn nothing_leaves_a_disabled_pin() {
    assert_eq!(hold_checked(&timer("set_disable_timing [get_pins g1/A]\n")), ["r1/D"]);
    let lib_pin = timer("set_disable_timing [get_lib_pins */INV/A]\n");
    assert_eq!(hold_checked(&lib_pin), ["r1/D"], "a library pin is that pin of every instance");
}

/// Rule: `unset_disable_timing` takes back what the same objects and ports set.
#[test]
fn unset_takes_a_disable_back() {
    let t = timer("set_disable_timing [get_cells r1]\nunset_disable_timing [get_cells r1]\n");
    assert_eq!(hold_checked(&t), ["r1/D", "r2/D"]);
}

/// Rule (`parse_disable_inst_ports`, Error 431): a `-from`/`-to` port the instance's cell does
/// not have ends the command, so none of it applies.
#[test]
fn a_port_the_cell_lacks_refuses_the_command() {
    let t = timer("set_disable_timing -to Q [get_cells {r1 g1}]\n");
    assert_eq!(hold_checked(&t), ["r1/D", "r2/D"]);
}
