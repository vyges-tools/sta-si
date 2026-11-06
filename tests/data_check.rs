// SPDX-License-Identifier: Apache-2.0
//! `set_data_check`, rule by rule, on the two-flop ring `r1.Q -[g1]-> r2.D`. Each rule is the
//! reference's (`VisitPathEnds::visitDataCheckEnd`, `PathEndDataCheck`, `CycleAccting`); all of
//! them were also witnessed against it on gcd (sky130hd), 18 variants, WNS/TNS/worst slack both
//! ways, the data check setting the worst slack on each side.
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

const CHECK: &str = "set_data_check -from [get_pins g1/A] -to [get_pins g1/Y]";

/// Rule: a data check makes its to-pin an endpoint, and its margin enters the required time
/// one for one — subtracted on setup (required = the from-pin's early arrival − margin), added
/// on hold (required = the from-pin's late arrival + margin).
#[test]
fn the_to_pin_is_checked_and_the_margin_enters_one_for_one() {
    let base = timer("");
    let s1 = timer(&format!("{CHECK} -setup 5\n"));
    let s2 = timer(&format!("{CHECK} -setup 6\n"));
    assert_eq!(s1.report().endpoints, base.report().endpoints + 1, "g1/Y is a setup endpoint now");
    assert!((s1.report().wns - s2.report().wns - 1.0).abs() < 1e-9, "{} {}", s1.report().wns, s2.report().wns);
    assert_eq!(s1.report().hold_endpoints, base.report().hold_endpoints, "-setup alone makes no hold check");

    let h1 = timer(&format!("{CHECK} -hold 5\n"));
    let h2 = timer(&format!("{CHECK} -hold 6\n"));
    assert_eq!(h1.report().hold_endpoints, base.report().hold_endpoints + 1);
    assert!((h1.report().whs - h2.report().whs - 1.0).abs() < 1e-9, "{} {}", h1.report().whs, h2.report().whs);
}

/// Rule (`CycleAccting`): setup is ZERO-cycle and hold one cycle back. The ring's period is 1:
/// with no margin, setup slack is the from-pin's early arrival minus the to-pin's late one — the
/// gate delay, negated, whatever the period — and hold slack is a period plus the to-pin's early
/// arrival minus the from-pin's late one.
#[test]
fn setup_is_zero_cycle_and_hold_one_cycle_back() {
    let t = timer(&format!("{CHECK} 0\n"));
    let r = t.report();
    assert!(r.wns < 0.0 && r.wns > -1.0, "a zero-cycle check fails by the gate delay: {}", r.wns);
    let y = t.pin("g1/Y").unwrap();
    let hold = r.hold_slacks.iter().find(|(i, _)| *i == y).map(|(_, s)| *s).expect("g1/Y hold-checked");
    assert!(hold > 0.0, "a period back, the check is met: {hold}");
    assert!((hold + r.wns - 1.0).abs() < 0.1, "setup and hold are the same gate delay a period apart: {hold} {}", r.wns);
}

/// `unset_data_check` takes a check back; one with a clock is refused (not modelled).
#[test]
fn unset_takes_it_back_and_a_clock_is_refused() {
    let base = timer("");
    let u = timer(&format!("{CHECK} 0.1\nunset_data_check -from [get_pins g1/A] -to [get_pins g1/Y]\n"));
    assert_eq!(u.report().endpoints, base.report().endpoints);
    let c = timer(&format!("{CHECK} -clock clk 0.1\n"));
    assert_eq!(c.report().endpoints, base.report().endpoints);
}

/// Rule (`SdcNetwork::findPinsMatching`, `set_data_check` naming ONE pin, Error 128): a
/// `[get_pins …]` pattern is found as the reference finds pins, so `g?/A` is the check on `g1/A`;
/// a pattern naming two pins (`g1/*`) is not applied.
#[test]
fn a_pin_pattern_names_the_pin_the_reference_finds() {
    let base = timer("");
    let lit = timer(&format!("{CHECK} -setup 5\n"));
    let pat = timer("set_data_check -from [get_pins g?/A] -to [get_pins g1/Y] -setup 5\n");
    assert_eq!(pat.report().endpoints, base.report().endpoints + 1);
    assert_eq!(pat.report().wns, lit.report().wns);
    let two = timer("set_data_check -from [get_pins g1/*] -to [get_pins g1/Y] -setup 5\n");
    assert_eq!(two.report().endpoints, base.report().endpoints, "a pattern naming two pins: not applied");
}
