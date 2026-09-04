// SPDX-License-Identifier: Apache-2.0
//! A clock pin's EARLY and LATE arrival must be equal when nothing derates them.
//!
//! **The invariant.** Early (hold) and late (setup) arrivals may differ only because
//! something *makes* them differ — derating, OCV, or separate min/max corners. With one
//! library, flat derate and no OCV, a register clock pin has exactly one arrival: the
//! time the LAUNCHING EDGE gets there. `create_clock` fixes that edge (rising), and a
//! rising edge stays rising through a non-inverting buffer, so both passes must report
//! the rise arrival.
//!
//! **What OpenSTA does.** Clock paths carry a `ClkInfo` with a `ClockEdge`, and
//! `Search::clkPathArrival` returns that path's own arrival — there is no minimum taken
//! over rise and fall at a clock pin. On `fft_top` it reports **0.8411** at the launch
//! CLK pin of a MIN (hold) path, which is the value our LATE pass already computes
//! (0.8405, 0.6 ps apart). Our EARLY pass says 0.6429.
//!
//! **What we do instead.** `sta.rs`'s per-lane collapse picks a lane by EXTREMUM —
//! `max` for late, `min` for early — rather than by clock edge. On a clock tree whose
//! rise is slower than its fall, the early pass therefore reports the FALLING edge's
//! arrival as the launch time, and everything downstream of the CLK->Q arc inherits it.
//!
//! **Measured cost, `fft_top` @ pin `7d490b8`:** all **536** checked CK pins carry a
//! late-minus-early spread of 0.102-0.199 ns (p50 0.186) that should be zero. It is the
//! attributed cause of the residual hold gap — WHS +0.6157 against OpenSTA's +0.88 — and
//! it accounts for BOTH halves of it: 0.198 ns of launch clock plus 0.149 ns of data path
//! re-derived from the wrong edge. See `vyges-tools-internal/docs/resize/rsz-audit.md`.
use vyges_sta_si::job::StaJob;
use vyges_sta_si::liberty::Lib;
use vyges_sta_si::sta::Timer;

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/examples/clk_edge/");

fn timer() -> Timer {
    let job = StaJob::load(&format!("{D}clk_edge.sta")).unwrap();
    let nl = vyges_sta_si::netlist::load(&job.resolve(&job.netlist)).unwrap();
    let mut lib = Lib::default();
    for l in &job.libs {
        lib.cells.extend(Lib::load(&job.resolve(l)).unwrap().cells);
    }
    Timer::build(&nl, &lib, &job, None).unwrap()
}

/// The fixture must really pose the question. If CLKBUF ever became symmetric the
/// invariant below would hold for the wrong reason and prove nothing.
#[test]
fn the_fixture_really_has_an_asymmetric_clock_buffer() {
    let job = StaJob::load(&format!("{D}clk_edge.sta")).unwrap();
    let lib = Lib::load(&job.resolve(&job.libs[0])).unwrap();
    let arc = &lib.cells["CLKBUF"].pins["X"].arcs[0];
    let (r, f) = (arc.cell_rise.lookup(0.02, 0.001), arc.cell_fall.lookup(0.02, 0.001));
    assert!(
        (r - f).abs() > 0.15,
        "CLKBUF must stay rise/fall asymmetric or this suite is vacuous: rise {r}, fall {f}"
    );
}

/// ⛔ FAILS TODAY — this is the attributed residual, pinned so the fix is measured.
/// Un-ignore it when the clock-edge selection lands; it should then pass unchanged.
#[test]
#[ignore = "attributed defect: the early pass reports the FALLING edge at a clock pin. \
            Un-ignore when clock-edge selection replaces the min-over-lanes collapse."]
fn a_clock_pin_has_one_arrival_when_nothing_derates_it() {
    let t = timer();
    let ck = t.pin("r1/CLK").expect("r1/CLK is a node");
    let (late, early) = (t.arrival(ck), t.arrival_min(ck));
    // Two CLKBUFs at 0.30 rise / 0.10 fall: the launching rise edge arrives at ~0.60.
    assert!(
        (late - 0.60).abs() < 1e-6,
        "the late pass should already carry the rise edge: {late}"
    );
    assert!(
        (early - late).abs() < 1e-9,
        "one corner and no derating, so a CLK pin has ONE arrival — the launching rise \
         edge at {late}. Got early {early}, which is the FALLING edge (2 x 0.10); the \
         spread of {} ns is invented by the per-lane collapse.",
        late - early
    );
}
