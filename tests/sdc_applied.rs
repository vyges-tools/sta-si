// SPDX-License-Identifier: Apache-2.0
//! Each SDC command loom's command registry says this timer applies (`applied_by` naming
//! `sta-si`), shown to change a timing result here: the same constraints with and without the
//! command time differently, and an `unset_*` command puts back the result of neither. A command
//! read but not applied would leave the two runs equal, and its test fails.
//!
//! The fixture (`tests/applied/app.v`): `a -[ga]-> r1/D`; `r1 -[g1]-> r2/D` on `clk`; `r1 -> r3/D`
//! on `clk2`; `r1 -> r4/D` with r4 clocked by `rd/Q`, a divide-by-two of `clk`; outputs `y`
//! (r2/Q), `z`, `w`. Library: `examples/seq/seq.lib`.
use vyges_sta_si::job::{merge_sdc_into, StaJob};
use vyges_sta_si::sdc::Sdc;
use vyges_sta_si::sta::Timer;
use vyges_sta_si::{liberty::Lib, netlist};

const D: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/applied/");

/// The clock on `clk` and the I/O delays every test starts from.
const CLOCK: &str = "create_clock -name c -period 2 [get_ports clk]\n";
const IO: &str = "set_input_delay 0.1 -clock c [get_ports a]\nset_output_delay 0.1 -clock c [get_ports {y z w}]\n";

fn timer(sdc: &str) -> Timer {
    let mut job = StaJob::load(&format!("{D}app.sta")).expect("fixture job");
    merge_sdc_into(&Sdc::parse(sdc).unwrap_or_else(|e| panic!("{sdc}: {e:?}")), &mut job);
    let nl = netlist::load(&job.resolve(&job.netlist)).expect("fixture netlist");
    let lib = Lib::load(&job.resolve(&job.libs[0])).expect("fixture library");
    Timer::build(&nl, &lib, &job, None).expect("timer")
}

/// Every setup and hold endpoint with its slack, by name: the whole result two runs compare.
fn slacks(t: &Timer) -> Vec<(String, f64)> {
    let mut v: Vec<(String, f64)> = t.endpoint_slacks().into_iter().map(|(p, s)| (format!("setup {}", t.pin_label(p)), s)).collect();
    v.extend(t.hold_endpoint_slacks().into_iter().map(|(p, s)| (format!("hold {}", t.pin_label(p)), s)));
    v.sort_by(|a, b| a.0.cmp(&b.0));
    v
}

/// The setup slack at `pin`, if it is a setup endpoint.
fn setup(t: &Timer, pin: &str) -> Option<f64> {
    slacks(t).into_iter().find(|(n, _)| n == &format!("setup {pin}")).map(|(_, s)| s)
}

/// `base` and `base` + `cmd` time differently, in the slack at `pin`: `(without, with)`.
fn moved(base: &str, cmd: &str, pin: &str) -> (Option<f64>, Option<f64>) {
    let (without, with) = (timer(base), timer(&format!("{base}{cmd}\n")));
    let (a, b) = (setup(&without, pin), setup(&with, pin));
    assert_ne!(a, b, "{cmd}: the setup slack at {pin} did not move\n{:?}", slacks(&with));
    (a, b)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// `create_clock`: a register on a port no clock is defined on is not checked; the clock makes
/// it an endpoint.
#[test]
fn create_clock_clocks_its_registers() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "create_clock -name c2 -period 2 [get_ports clk2]", "r3/D");
    assert!(without.is_none() && with.is_some(), "{without:?} {with:?}");
}

/// `create_generated_clock`: a clock derived from a master at a pin clocks what that pin
/// drives; without it, r4 (clocked by a register's output) is not checked.
#[test]
fn create_generated_clock_clocks_what_its_pin_drives() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins rd/Q]", "r4/D");
    assert!(without.is_none() && with.is_some(), "{without:?} {with:?}");
}

/// `set_false_path`: a path it names is not timed.
#[test]
fn set_false_path_removes_the_check() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_false_path -to [get_pins r2/D]", "r2/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `set_input_delay`: an input with no delay starts no path; with one, the path from it is
/// checked, and its value moves the slack one for one.
#[test]
fn set_input_delay_starts_a_path_at_its_value() {
    let (without, with) = moved(CLOCK, "set_input_delay 0.1 -clock c [get_ports a]", "r1/D");
    assert!(without.is_none(), "{without:?}");
    let later = setup(&timer(&format!("{CLOCK}set_input_delay 0.3 -clock c [get_ports a]\n")), "r1/D").expect("checked");
    assert!(close(with.expect("checked") - later, 0.2), "{with:?} {later}");
}

/// `set_multicycle_path -setup N`: the setup check moves N - 1 periods later.
#[test]
fn set_multicycle_path_moves_the_setup_check() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_multicycle_path 2 -setup -to [get_pins r2/D]", "r2/D");
    assert!(close(with.expect("checked") - without.expect("checked"), 2.0), "{without:?} {with:?}");
}

/// `set_output_delay`: the output's required time is the capture edge less the delay, so its
/// value moves the slack one for one. (An output with no delay is checked here against the
/// job's default delay, 0 in this fixture, not left unchecked.)
#[test]
fn set_output_delay_checks_the_output_at_its_value() {
    let base = format!("{CLOCK}set_input_delay 0.1 -clock c [get_ports a]\n");
    let (without, with) = moved(&base, "set_output_delay 0.1 -clock c [get_ports y]", "y");
    assert!(close(without.expect("checked") - with.expect("checked"), 0.1), "{without:?} {with:?}");
    let later = setup(&timer(&format!("{base}set_output_delay 0.4 -clock c [get_ports y]\n")), "y").expect("checked");
    assert!(close(with.expect("checked") - later, 0.3), "{with:?} {later}");
}

/// `set_case_analysis`: a constant pin starts and carries no path.
#[test]
fn set_case_analysis_holds_its_pin_constant() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_case_analysis 0 [get_ports a]", "r1/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `set_clock_groups -asynchronous`: paths between the groups are not timed.
#[test]
fn set_clock_groups_cuts_paths_between_groups() {
    let base = format!("{CLOCK}create_clock -name c2 -period 2 [get_ports clk2]\n{IO}");
    let (without, with) = moved(&base, "set_clock_groups -asynchronous -group c -group c2", "r3/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `set_clock_latency -source` with `set_input_delay -source_latency_included`: the input delay
/// already contains the clock's source latency, which then comes out of its arrival.
#[test]
fn set_clock_latency_comes_out_of_a_delay_that_includes_it() {
    let base = format!("{CLOCK}set_input_delay 0.5 -clock c -source_latency_included [get_ports a]\n");
    let (without, with) = moved(&base, "set_clock_latency -source 0.3 [get_clocks c]", "r1/D");
    assert!(close(with.expect("checked") - without.expect("checked"), 0.3), "{without:?} {with:?}");
}

/// `set_clock_transition`: an ideal clock's slew at the register clock pins, which the
/// clock-to-output delay depends on.
#[test]
fn set_clock_transition_sets_the_ideal_clock_slew() {
    moved(&format!("{CLOCK}{IO}"), "set_clock_transition 0.08 [get_clocks c]", "r2/D");
}

/// `set_clock_uncertainty -setup`: subtracted from every setup required time of the clock.
#[test]
fn set_clock_uncertainty_tightens_the_setup_check() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_clock_uncertainty -setup 0.1 [get_clocks c]", "r2/D");
    assert!(close(without.expect("checked") - with.expect("checked"), 0.1), "{without:?} {with:?}");
}

/// `set_driving_cell`: the input is driven through that cell's arc, not an ideal edge.
#[test]
fn set_driving_cell_drives_the_input_through_the_cell() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_driving_cell -lib_cell INV [get_ports a]", "r1/D");
    assert!(with.expect("checked") < without.expect("checked"), "{without:?} {with:?}");
}

/// `set_input_transition`: the input's slew, which the first gate's delay depends on.
#[test]
fn set_input_transition_sets_the_input_slew() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_input_transition 0.08 [get_ports a]", "r1/D");
    assert!(with.expect("checked") < without.expect("checked"), "{without:?} {with:?}");
}

/// `set_load`: the capacitance on an output, which the driving register's delay depends on.
#[test]
fn set_load_loads_the_output() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_load 0.01 [get_ports y]", "y");
    assert!(with.expect("checked") < without.expect("checked"), "{without:?} {with:?}");
}

/// `set_logic_zero`: a constant 0 on the pin, which then starts and carries no path.
#[test]
fn set_logic_zero_holds_its_pin_constant() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_logic_zero [get_ports a]", "r1/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `set_logic_one`: likewise a constant 1.
#[test]
fn set_logic_one_holds_its_pin_constant() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_logic_one [get_ports a]", "r1/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `set_logic_dc`: X is a constraint value too. After `set_logic_zero` on the same port it
/// replaces the 0, so the port is no longer constant and its path is timed again.
#[test]
fn set_logic_dc_replaces_a_constant_with_a_dont_care() {
    let base = format!("{CLOCK}{IO}set_logic_zero [get_ports a]\n");
    let (without, with) = moved(&base, "set_logic_dc [get_ports a]", "r1/D");
    assert!(without.is_none() && with.is_some(), "{without:?} {with:?}");
}

/// `set_propagated_clock`: the clock's arrival at the register clock pins is computed from its
/// source, not taken as an ideal edge with the `set_clock_transition` slew.
#[test]
fn set_propagated_clock_replaces_the_ideal_clock() {
    let base = format!("{CLOCK}{IO}set_clock_transition 0.08 [get_clocks c]\n");
    moved(&base, "set_propagated_clock [get_clocks c]", "r2/D");
}

/// `set_timing_derate -late`: late (setup data) cell delays are scaled by it.
#[test]
fn set_timing_derate_scales_late_delays() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_timing_derate -late 1.2", "r2/D");
    assert!(with.expect("checked") < without.expect("checked"), "{without:?} {with:?}");
}

const DATA_CHECK: &str = "set_data_check -from [get_pins g1/A] -to [get_pins g1/Y] -setup 0.1";

/// `set_data_check`: the to-pin becomes a setup endpoint, checked against the from-pin.
#[test]
fn set_data_check_makes_the_to_pin_an_endpoint() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), DATA_CHECK, "g1/Y");
    assert!(without.is_none() && with.is_some(), "{without:?} {with:?}");
}

/// `unset_data_check`: the same from/to takes the check back; the result is the one without it.
#[test]
fn unset_data_check_takes_the_check_back() {
    let base = format!("{CLOCK}{IO}");
    let set = slacks(&timer(&format!("{base}{DATA_CHECK}\n")));
    let back = slacks(&timer(&format!("{base}{DATA_CHECK}\nunset_data_check -from [get_pins g1/A] -to [get_pins g1/Y]\n")));
    assert_ne!(set, back, "the unset changed nothing");
    assert_eq!(back, slacks(&timer(&base)));
}

/// `set_disable_timing` on an instance: its arcs carry no path, so what only it feeds is not
/// reached.
#[test]
fn set_disable_timing_cuts_the_arcs() {
    let (without, with) = moved(&format!("{CLOCK}{IO}"), "set_disable_timing [get_cells g1]", "r2/D");
    assert!(without.is_some() && with.is_none(), "{without:?} {with:?}");
}

/// `unset_disable_timing`: the same objects take the disable back; the result is the one
/// without it.
#[test]
fn unset_disable_timing_takes_the_disable_back() {
    let base = format!("{CLOCK}{IO}");
    let set = slacks(&timer(&format!("{base}set_disable_timing [get_cells g1]\n")));
    let back = slacks(&timer(&format!("{base}set_disable_timing [get_cells g1]\nunset_disable_timing [get_cells g1]\n")));
    assert_ne!(set, back, "the unset changed nothing");
    assert_eq!(back, slacks(&timer(&base)));
}

/// A generated clock whose `-source` no clock is defined on takes its master from the design, after
/// the read (`Sta::updateGeneratedClks`; `Genclks::ensureMaster` traces the source back through
/// the netlist to the clock defined there): `rd/CK` is driven by the port `c` is on, so `g` is
/// `c` divided by 2, period 4. Read without the design, `g` would have no master and period 0.
#[test]
fn a_generated_clock_takes_its_master_from_the_design() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_applied_genclk_master");
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let sdc = dir.join("g.sdc");
    std::fs::write(&sdc, format!("{CLOCK}create_generated_clock -name g -source [get_pins rd/CK] -divide_by 2 [get_pins rd/Q]\n"))
        .expect("write sdc");
    let sta = dir.join("g.sta");
    std::fs::write(
        &sta,
        format!("design: app\nnetlist: {D}app.v\nlib: {D}../../examples/seq/seq.lib\nsdc: {}\n", sdc.display()),
    )
    .expect("write job");
    let job = StaJob::load(&sta.to_string_lossy()).expect("job");
    let g: Vec<f64> = job.clocks.iter().filter(|c| c.0 == "g").map(|c| c.2).collect();
    assert_eq!(g, vec![4.0], "{:?}", job.clocks);
}
