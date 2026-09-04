// SPDX-License-Identifier: Apache-2.0
//! A register whose clock pin no DECLARED clock reaches carries no check at all.
//!
//! **The upstream rule.** OpenSTA seeds clock-tagged paths only at pins the SDC names as
//! a clock source (`Sdc::isLeafPinClock` gating `seedClkArrivals` in
//! `Search::seedArrival`); every other start is a DATA arrival from `seedInputArrival`.
//! `VisitPathEnds::visitCheckEnd` then builds a setup / hold / recovery / removal
//! `PathEndCheck` only for a target clock path whose `isClock()` is true — with none it
//! leaves `check_clked` false and falls through to `visitCheckEndUnclked`, which emits
//! nothing but an explicit `set_max_delay` / `set_min_delay`. So such a register is an
//! **unconstrained endpoint, not a passing one**.
//!
//! **Why it matters.** On `fft_top` — whose SDC creates a clock on `clk_i` and leaves
//! `pclk_i` an ordinary data input with a 12 ns `set_input_delay` — this engine
//! propagated that 12 ns through the clock buffers and used it as a CAPTURE CLOCK
//! ARRIVAL for 194 of 730 registers. WHS came out −1.0682 where OpenSTA and sign-off
//! both say +0.88, and a hold-fix ECO acted on it: **599 delay cells into a design that
//! is already hold-clean by 0.88 ns**. An incomplete SDC must produce silence and a
//! count, never numbers.
//!
//! This is the clock-side half of the rule landed port-side at `4a578af`.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/undeclared_clk/");

fn timer() -> Timer {
    let job = StaJob::load(&format!("{D}undeclared_clk.sta")).unwrap();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

/// The fixture must actually pose the question: r2's CLK has to carry a LATE arrival
/// from the undeclared tree, and a later one than its own data. Without this the
/// tests below could pass on a fixture that never manufactured anything.
#[test]
fn the_fixture_really_puts_a_late_data_arrival_on_the_undeclared_clock_pin() {
    let t = timer();
    let ck2 = t.pin("r2/CLK").expect("r2/CLK is a node");
    let d2 = t.pin("r2/D").expect("r2/D is a node");
    let ck1 = t.pin("r1/CLK").expect("r1/CLK is a node");
    let a = |p| t.setup_arrival(p);
    assert!(
        a(ck2) > 12.0,
        "r2/CLK should carry pclk_i's 12 ns input delay, got {}",
        a(ck2)
    );
    assert!(
        a(ck2) > a(d2),
        "the undeclared clock tree must be DEEPER than r2's data path — that is what \
         turns into a fake hold violation ({} vs {})",
        a(ck2),
        a(d2)
    );
    assert!(
        a(ck1) < 12.0,
        "r1's declared clock tree carries no input delay: {}",
        a(ck1)
    );
}

/// Setup and RECOVERY: `flop_d` carries both, so declining there covers both.
#[test]
fn an_undeclared_clock_leaves_its_registers_out_of_the_setup_and_recovery_set() {
    let t = timer();
    for pin in ["r2/D", "r2/RESET_B"] {
        let p = t.pin(pin).unwrap_or_else(|| panic!("{pin} is a node"));
        assert!(
            t.slack(p).is_none(),
            "{pin} is clocked by the undeclared pclk_i and must carry no setup/recovery \
             check — OpenSTA builds no PathEndCheck for it, got slack {:?}",
            t.slack(p)
        );
    }
    // ...and the declared side is untouched. A rule that silenced everything would be
    // just as wrong as one that silenced nothing.
    for pin in ["r1/D", "r1/RESET_B"] {
        let p = t.pin(pin).unwrap_or_else(|| panic!("{pin} is a node"));
        assert!(
            t.slack(p).is_some(),
            "{pin} is on the DECLARED clock and must still be checked"
        );
    }
}

/// Hold and REMOVAL: `flop_hold` carries both.
#[test]
fn an_undeclared_clock_leaves_its_registers_out_of_the_hold_and_removal_set() {
    let t = timer();
    let r = t.report();
    let held = |p| r.hold_slacks.iter().any(|(i, _)| *i == p);
    for pin in ["r2/D", "r2/RESET_B"] {
        let p = t.pin(pin).unwrap();
        assert!(!held(p), "{pin} must carry no hold/removal check");
    }
    for pin in ["r1/D", "r1/RESET_B"] {
        let p = t.pin(pin).unwrap();
        assert!(held(p), "{pin} is on the DECLARED clock and must still be checked");
    }
}

/// The consequence, stated as a number: WHS is the declared clock's, and it is MET.
/// Before this rule the fake capture arrival on r2 set WHS and it was negative — the
/// reading that drove 599 delay cells into a hold-clean `fft_top`.
#[test]
fn whs_comes_from_the_declared_clock_and_is_met() {
    let t = timer();
    let r = t.report();
    assert!(
        r.whs > 0.0,
        "WHS must not be manufactured by an undeclared clock's data arrival, got {}",
        r.whs
    );
    let worst = t.pin(&r.worst_hold_endpoint);
    let r2_pins: Vec<_> = ["r2/D", "r2/RESET_B"].iter().filter_map(|p| t.pin(p)).collect();
    assert!(
        worst.is_none_or(|w| !r2_pins.contains(&w)),
        "the worst hold endpoint must not be a register on the undeclared clock, got {}",
        r.worst_hold_endpoint
    );
}

/// Guard: with NO clock source resolved we cannot tell a missing `create_clock` from a
/// clock port this model failed to locate, so the rule must not fire. Declining every
/// check on a model gap would turn it into a silently clean design — the exact failure
/// the rule exists to prevent, inverted.
#[test]
fn a_job_whose_clock_source_resolves_to_nothing_still_checks_its_registers() {
    let mut job = StaJob::load(&format!("{D}undeclared_clk.sta")).unwrap();
    job.clocks = vec![("nope".into(), "no_such_port".into(), 2.0)];
    job.clock_port = "no_such_port".into();
    let nl = netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    let t = Timer::build(&nl, &lib, &job, None).unwrap();
    let p = t.pin("r1/D").unwrap();
    assert!(
        t.slack(p).is_some(),
        "no clock source resolved: the rule must stand down rather than silence the design"
    );
}

/// The other half of the same rule: a register on an undeclared clock does not LAUNCH.
///
/// `Search.cc`'s `regClkToQ` branch propagates only when `from_tag->isClock()` and sets
/// `to_tag = nullptr` otherwise — "Do not propagate paths from input ports with default
/// input arrival clk thru CLK->Q edges". So nothing leaves r2 at all, and the output it
/// drives is not an endpoint.
///
/// ⛔ Leaving this out was worth **83 of 790 setup endpoints out by 8.9–12.6 ns** on
/// `fft_top` — the undeclared domain's 12 ns input delay reaching the data cone of
/// registers that ARE declared. Setup rms 3.6447 where hold was 0.033, and WNS was 1.9 %
/// out and showed none of it.
#[test]
fn a_register_on_an_undeclared_clock_does_not_launch_either() {
    let t = timer();
    let q = t.pin("r2/Q").expect("r2/Q is a node");
    assert!(
        !t.setup_arrival(q).is_finite(),
        "r2 is clocked by the undeclared pclk_i and must launch nothing, got {}",
        t.setup_arrival(q)
    );
    let out = t.pin("pq_o").expect("pq_o is a node");
    assert!(
        t.slack(out).is_none(),
        "and the output it drives is then not a timed endpoint"
    );
    // the declared side still launches
    let q1 = t.pin("r1/Q").unwrap();
    assert!(t.setup_arrival(q1).is_finite(), "r1 is on the DECLARED clock and must launch");
}
