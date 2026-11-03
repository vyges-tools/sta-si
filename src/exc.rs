//! Timing exceptions as this timer matches them: each side's objects BY KIND.
//!
//! The reader keeps every `-from` / `-to` object with the accessor that named it
//! ([`Exception::args`]); [`Exception::from`] / [`Exception::to`] are those objects reduced to
//! names, which cannot say whether `a` was a clock, a cell or a port. The reference keeps the
//! kinds apart (an exception side holds pins, instances and clocks) and matches each kind against
//! a different property of the path:
//!
//! - a CLOCK on the from side matches a path launched by that clock — at a register, the clock
//!   that reaches its clock pin; at an input port, the clock its input delay is relative to;
//! - a CLOCK on the to side matches a path captured by that clock;
//! - an instance, pin or port matches the path's start (end) object, as before.
//!
//! A side matches when ANY of its objects does; a side with no objects matches every path.
//!
//! ⛔ An object of a kind this timer cannot match is never quietly inert or quietly "any": the
//! exception is REFUSED ([`refuse_unmatchable`]) so the run's constraint health reports it.

use crate::sdc::{Exception, ExcKind, Objects, Sdc};

/// One object on an exception side, by the kind the reference reads it as.
#[derive(Debug, Clone, PartialEq)]
pub enum Obj {
    /// A clock, by name or pattern (`[get_clocks a]`, `[all_clocks]` = `*`, or a bare name that
    /// is a clock's).
    Clock(String),
    /// An instance or a port, by name or pattern (a pin reduces to its instance).
    Inst(String),
    /// `[all_inputs]`: every input port.
    Inputs,
    /// `[all_outputs]`: every output port.
    Outputs,
    /// `[all_registers]`: every register.
    Registers,
}

/// One end of a timed path as an exception sees it.
#[derive(Debug, Clone, Copy)]
pub struct PathPoint<'a> {
    /// The instance (a register) or the port the path starts or ends at; empty when there is
    /// none (a path from a tie cell).
    pub name: &'a str,
    /// The clock that launched (captured) it, when one did.
    pub clock: Option<&'a str>,
    /// The point is a top-level port rather than a register.
    pub port: bool,
}

/// An exception with its two sides read by kind. `None` for a side: every path.
#[derive(Debug, Clone)]
pub struct Matcher<'a> {
    pub exc: &'a Exception,
    from: Option<Vec<Obj>>,
    to: Option<Vec<Obj>>,
    /// The checks it is written for (`-setup` / `-hold`; neither or both is both).
    min_max: MinMax,
    /// A multicycle counts the capture clock's cycles (`-end`), else the launch clock's.
    end_clk: bool,
    /// The reference's priority for how specific the sides are (`fromThruToPriority`).
    specificity: i32,
}

/// The checks an exception is written for: `-setup` only, `-hold` only, or both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MinMax {
    All,
    Max,
    Min,
}

const FROM_FLAGS: [&str; 3] = ["-from", "-rise_from", "-fall_from"];
const TO_FLAGS: [&str; 3] = ["-to", "-rise_to", "-fall_to"];

/// Tcl `string match` for `*` and `?`.
fn glob(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut i, mut j, mut star, mut mark) = (0usize, 0usize, None, 0usize);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(st) = star {
            i = st + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    while i < p.len() && p[i] == '*' {
        i += 1;
    }
    i == p.len()
}

pub(crate) fn named(pat: &str, name: &str) -> bool {
    pat == "*" || pat == name || (pat.contains(['*', '?']) && glob(pat, name))
}

/// A pin names its instance (`r1/Q` -> `r1`); a port or an instance is itself.
fn instance_part(name: &str) -> String {
    name.rsplit_once('/').map_or(name, |(inst, _)| inst).to_string()
}

/// The objects one written object stands for, or why this timer cannot match it.
/// `clocks`: every clock's name — a bare name the reference looks up as a clock FIRST, and only
/// then as an instance, a port or a pin.
fn objects_of(o: &Objects, clocks: &[&str], out: &mut Vec<Obj>) -> Result<(), String> {
    match o.accessor.as_str() {
        "list" | "concat" => {
            for it in &o.items {
                objects_of(it, clocks, out)?;
            }
            Ok(())
        }
        "get_clocks" => {
            out.extend(o.names.iter().map(|n| Obj::Clock(n.clone())));
            Ok(())
        }
        "all_clocks" => {
            out.push(Obj::Clock("*".to_string()));
            Ok(())
        }
        "all_inputs" if o.keys.is_empty() && o.flags.iter().all(|f| f == "-no_clocks") => {
            out.push(Obj::Inputs);
            Ok(())
        }
        "all_outputs" if o.keys.is_empty() && o.flags.is_empty() => {
            out.push(Obj::Outputs);
            Ok(())
        }
        // the pin selectors keep every register; a `-clock` / edge selector narrows the set to
        // registers of one clock or kind, which is not matched here
        "all_registers" if o.keys.is_empty() && o.flags.iter().all(|f| ["-clock_pins", "-data_pins", "-output_pins", "-cells"].contains(&f.as_str())) => {
            out.push(Obj::Registers);
            Ok(())
        }
        "get_ports" | "get_pins" | "get_cells" => {
            out.extend(o.names.iter().map(|n| Obj::Inst(instance_part(n))));
            Ok(())
        }
        "" => {
            for n in &o.names {
                match n.as_str() {
                    "*INPUTS*" => out.push(Obj::Inputs),
                    "*OUTPUTS*" => out.push(Obj::Outputs),
                    "*REGS*" => out.push(Obj::Registers),
                    _ if clocks.iter().any(|c| named(n, c)) => out.push(Obj::Clock(n.clone())),
                    _ => out.push(Obj::Inst(instance_part(n))),
                }
            }
            Ok(())
        }
        other => Err(format!("an exception object `[{other} …]` this timer cannot match")),
    }
}

/// One side's objects (`flags`: the side's keys), `None` when the side names none. An exception
/// built without its written objects (a `.sta` job's) keeps its reduced names, read as instances.
fn side(e: &Exception, flags: &[&str], reduced: &[String], clocks: &[&str]) -> Result<Option<Vec<Obj>>, String> {
    let written: Vec<&Objects> = e.args.iter().filter(|(k, _)| flags.contains(&k.as_str())).map(|(_, o)| o).collect();
    if written.is_empty() {
        if !e.args.is_empty() || reduced.is_empty() || reduced.iter().any(|n| n == "*") {
            return Ok(None);
        }
        return Ok(Some(reduced.iter().map(|n| Obj::Inst(n.clone())).collect()));
    }
    let mut out = Vec::new();
    for o in written {
        objects_of(o, clocks, &mut out)?;
    }
    Ok(Some(out))
}

/// Why this timer cannot match exception `e`, when it cannot — an object of a kind it does not
/// match (a net, a library cell, a register set selected by clock).
pub fn unmatchable(e: &Exception, clocks: &[&str]) -> Option<String> {
    side(e, &FROM_FLAGS, &e.from, clocks).and(side(e, &TO_FLAGS, &e.to, clocks)).err()
}

/// Refuse, on the reader's own model, every exception this timer cannot match — so the health
/// line counts it with the others it does not apply. Run before the exceptions are used and
/// before the health line is written.
pub fn refuse_unmatchable(sdc: &mut Sdc) {
    let clocks: Vec<String> = sdc.clocks.iter().map(|c| c.name.clone()).collect();
    let clocks: Vec<&str> = clocks.iter().map(String::as_str).collect();
    for e in sdc.exceptions.iter_mut().filter(|e| e.refused.is_none()) {
        e.refused = unmatchable(e, &clocks);
    }
}

impl<'a> Matcher<'a> {
    /// Read `e`'s sides by kind against the design's clock names. An exception that cannot be
    /// read is one that matches nothing (it is refused at load, see [`refuse_unmatchable`]).
    pub fn new(e: &'a Exception, clocks: &[&str]) -> Matcher<'a> {
        let (from, to) = match (side(e, &FROM_FLAGS, &e.from, clocks), side(e, &TO_FLAGS, &e.to, clocks)) {
            (Ok(from), Ok(to)) => (from, to),
            _ => (Some(Vec::new()), Some(Vec::new())),
        };
        // `set_multicycle_path` / `set_false_path`: `-setup` alone is max, `-hold` alone min,
        // neither or both all; a multicycle counts the end clock unless `-start`, or it is a
        // `-hold` one without `-end`. A job-file exception (no flags written) keeps its fields.
        let has = |f: &str| e.flags.iter().any(|x| x == f);
        let (setup, hold) = if e.flags.is_empty() && e.hold { (false, true) } else { (has("-setup"), has("-hold")) };
        let min_max = match (setup, hold) {
            (true, false) => MinMax::Max,
            (false, true) => MinMax::Min,
            _ => MinMax::All,
        };
        let end_clk = if e.flags.is_empty() { !e.start } else { !(has("-start") || (min_max == MinMax::Min && !has("-end"))) };
        let objs = |s: &Option<Vec<Obj>>, clock: bool| s.as_ref().is_some_and(|v| v.iter().any(|o| matches!(o, Obj::Clock(_)) == clock));
        let mut specificity = 0;
        for (on, bit) in [(objs(&from, false), 6), (objs(&to, false), 5), (objs(&from, true), 3), (objs(&to, true), 2)] {
            if on {
                specificity |= 1 << bit;
            }
        }
        Matcher { exc: e, from, to, min_max, end_clk, specificity }
    }

    /// Is it written for the check `hold`? `exactly`: a multicycle written for setup only does
    /// not count for hold — without it, every multicycle reaches the hold check, because the
    /// setup multiplier decides the hold edge (`MultiCyclePath::matches`).
    fn matches_check(&self, hold: bool, exactly: bool) -> bool {
        let written = match self.min_max {
            MinMax::All => true,
            MinMax::Max => !hold,
            MinMax::Min => hold,
        };
        match self.exc.kind {
            ExcKind::FalsePath => written,
            ExcKind::Multicycle(_) => written || (!exactly && hold),
        }
    }

    /// The reference's priority of the exception for the check (`hold`): its type (a false path
    /// over a multicycle), how specific its sides are, and for a multicycle whether it was
    /// written for this check alone (+2), for both (+1) or for the other one (+0).
    fn priority(&self, hold: bool) -> i32 {
        match self.exc.kind {
            ExcKind::FalsePath => 4000 + self.specificity,
            ExcKind::Multicycle(_) => {
                let mm = match (self.min_max, hold) {
                    (MinMax::All, _) => 1,
                    (MinMax::Max, false) | (MinMax::Min, true) => 2,
                    _ => 0,
                };
                2000 + self.specificity + mm
            }
        }
    }

    /// The multiplier for the check (`hold`): a multicycle written for both checks counts 0 on
    /// the hold side (`MultiCyclePath::pathMultiplier`).
    fn multiplier(&self, hold: bool) -> f64 {
        match self.exc.kind {
            ExcKind::Multicycle(n) if !(hold && self.min_max == MinMax::All) => n as f64,
            ExcKind::Multicycle(_) => 0.0,
            ExcKind::FalsePath => 0.0,
        }
    }

    /// Does this exception cover the path from `from` to `to`? Never, when it is refused.
    pub fn covers(&self, from: &PathPoint, to: &PathPoint) -> bool {
        self.exc.refused.is_none() && side_matches(&self.from, from, true) && side_matches(&self.to, to, false)
    }
}

/// Does one side cover point `p`? `start`: the from side (an input port), else the to side.
fn side_matches(objs: &Option<Vec<Obj>>, p: &PathPoint, start: bool) -> bool {
    let Some(objs) = objs else { return true };
    objs.iter().any(|o| match o {
        Obj::Clock(c) => p.clock.is_some_and(|k| named(c, k)),
        Obj::Inst(n) => !p.name.is_empty() && named(n, p.name),
        Obj::Inputs => start && p.port,
        Obj::Outputs => !start && p.port,
        Obj::Registers => !p.port && !p.name.is_empty(),
    })
}

/// Every exception of `job`, read once against the design's clocks.
pub fn matchers<'a>(excs: &'a [Exception], clocks: &[&str]) -> Vec<Matcher<'a>> {
    excs.iter().map(|e| Matcher::new(e, clocks)).collect()
}

/// The exception the reference picks for the check (`hold`) on the path `from` -> `to`: the
/// highest priority, a smaller multiplier on a tie (`Sdc::exceptionTo`, `tighterThan`), else the
/// first in file order. `mcp_only`: multicycles only.
fn best<'m, 'a>(ms: &'m [Matcher<'a>], hold: bool, exactly: bool, mcp_only: bool, from: &PathPoint, to: &PathPoint) -> Option<&'m Matcher<'a>> {
    let mut hi: Option<&Matcher> = None;
    for m in ms.iter().filter(|m| m.matches_check(hold, exactly) && m.covers(from, to)) {
        if mcp_only && m.exc.kind == ExcKind::FalsePath {
            continue;
        }
        let tighter = |a: &Matcher, b: &Matcher| match (&a.exc.kind, &b.exc.kind) {
            (ExcKind::Multicycle(x), ExcKind::Multicycle(y)) => x < y,
            _ => false,
        };
        let take = match hi {
            None => true,
            Some(h) => m.priority(hold) > h.priority(hold) || (m.priority(hold) == h.priority(hold) && tighter(m, h)),
        };
        if take {
            hi = Some(m);
        }
    }
    hi
}

/// The exception the reference applies to the check (`hold`) on the path `from` -> `to`.
pub fn find<'a>(ms: &[Matcher<'a>], hold: bool, from: &PathPoint, to: &PathPoint) -> Option<&'a Exception> {
    best(ms, hold, false, false, from, to).map(|m| m.exc)
}

/// How far a check's capture edge moves, in ns, for the path `from` -> `to` (`pl` / `pc` the
/// launch and capture clock periods); `None` when a false path drops the check.
///
/// Setup (`PathEnd::checkSetupMcpAdjustment`): `(N - 1)` cycles of the end (or `-start`) clock.
/// Hold (`checkMcpAdjustment`, `findHoldMcps`): the setup multicycle moves the hold edge with it
/// and a hold multicycle moves it back — `(Nsetup - 1) * Tsetup - Nhold * Thold`, either term
/// absent when no such multicycle covers the path.
pub fn check_shift(ms: &[Matcher], hold: bool, from: &PathPoint, to: &PathPoint, pl: f64, pc: f64) -> Option<f64> {
    let Some(m) = best(ms, hold, false, false, from, to) else { return Some(0.0) };
    if m.exc.kind == ExcKind::FalsePath {
        return None;
    }
    let period = |m: &Matcher| if m.end_clk { pc } else { pl };
    if !hold {
        return Some((m.multiplier(false) - 1.0) * period(m));
    }
    let (setup, hold_mcp) = if m.min_max != MinMax::Max {
        (best(ms, false, true, true, from, to), Some(m))
    } else {
        (Some(m), best(ms, true, true, true, from, to))
    };
    Some(match (setup, hold_mcp) {
        (Some(s), Some(h)) => (s.multiplier(false) - 1.0) * period(s) - h.multiplier(true) * period(h),
        (None, Some(h)) => -h.multiplier(true) * period(h),
        (Some(s), None) => (s.multiplier(false) - 1.0) * period(s),
        (None, None) => 0.0,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(cmds: &str) -> Sdc {
        let base = "create_clock -name a -period 1 [get_ports ca]\ncreate_clock -name b -period 1 [get_ports cb]\n";
        let mut s = Sdc::parse(&format!("{base}{cmds}")).unwrap();
        refuse_unmatchable(&mut s);
        s
    }
    const CLOCKS: [&str; 2] = ["a", "b"];
    fn reg<'a>(name: &'a str, clock: &'a str) -> PathPoint<'a> {
        PathPoint { name, clock: Some(clock), port: false }
    }

    /// Reference rule (`ExceptionTo::matches`, `Sdc::exceptionFromStates`): a clock on the from
    /// side matches the launching clock, on the to side the capturing clock; a register of the
    /// SAME name does not stand in for it. Measured: `-from [get_clocks a] -to [get_clocks b]`
    /// cuts the a->b register path, setup and hold, and leaves b->a timed.
    #[test]
    fn a_clock_matches_the_launching_and_capturing_clock() {
        let s = parse("set_false_path -from [get_clocks a] -to [get_clocks b]\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        assert!(find(&ms, false, &reg("r1", "a"), &reg("r2", "b")).is_some());
        assert!(find(&ms, true, &reg("r1", "a"), &reg("r2", "b")).is_some());
        assert!(find(&ms, false, &reg("r2", "b"), &reg("r3", "a")).is_none());
        // a register named like the clock is not the clock
        assert!(find(&ms, false, &reg("a", "x"), &reg("b", "y")).is_none());
    }

    /// Reference rule (`get_object_args`): a bare name is looked up as a clock first, then as an
    /// instance, a port or a pin. Measured: `-from a -to b` cuts the a->b path as `get_clocks` does.
    #[test]
    fn a_bare_name_is_a_clock_when_one_has_that_name() {
        let s = parse("set_false_path -from a -to b\nset_false_path -from r9 -to b\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        assert!(std::ptr::eq(find(&ms, false, &reg("r1", "a"), &reg("r2", "b")).unwrap(), &s.exceptions[0]));
        assert!(std::ptr::eq(find(&ms, false, &reg("r9", "x"), &reg("r2", "b")).unwrap(), &s.exceptions[1]));
    }

    /// Reference rule (`ExceptionFromTo` holds pins, instances and clocks; any one matches): a
    /// side mixing a clock and a port covers both kinds of start. Measured: `-from [list
    /// [get_clocks b] [get_ports din]] -to [get_cells {r1 r3}]` cuts b->r3.
    #[test]
    fn a_mixed_side_matches_either_kind() {
        let s = parse("set_false_path -from [list [get_clocks b] [get_ports din]] -to [get_cells {r1 r3}]\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        assert!(find(&ms, false, &reg("r2", "b"), &reg("r3", "a")).is_some());
        let din = PathPoint { name: "din", clock: Some("a"), port: true };
        assert!(find(&ms, false, &din, &reg("r1", "a")).is_some());
        assert!(find(&ms, false, &reg("r1", "a"), &reg("r3", "a")).is_none());
        assert!(find(&ms, false, &reg("r2", "b"), &reg("r2", "b")).is_none());
    }

    /// Reference rule (error 100, unsupported object type Net): a net cannot be a -from / -to
    /// object. Refused, so the health line counts it, rather than read as an instance.
    #[test]
    fn an_object_kind_not_matched_is_refused() {
        let s = parse("set_false_path -from [get_nets q1]\nset_false_path -to [all_registers -clock a]\nset_false_path -from [get_clocks a]\n");
        let refused: Vec<bool> = s.exceptions.iter().map(|e| e.refused.is_some()).collect();
        assert_eq!(refused, [true, true, false]);
        assert!(s.health().unwrap().contains("2 timing exception(s) NOT applied"));
    }

    /// Reference rule (`findHoldMcps`, `MultiCyclePath::priority`): `-setup N` with `-hold N-1`
    /// leaves the hold check where it was; a `-setup` multicycle alone moves it N-1 cycles out; a
    /// plain one does both. Measured: `set_multicycle_path 2 -setup -to [get_clocks a]` plus
    /// `set_multicycle_path 1 -hold -to [get_clocks a]` keep hold at 0.3379, setup +1 period.
    #[test]
    fn a_hold_multicycle_moves_the_hold_edge_back() {
        let s = parse("set_multicycle_path 2 -setup -to [get_clocks a]\nset_multicycle_path 1 -hold -to [get_clocks a]\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        let (f, t) = (reg("r2", "b"), reg("r3", "a"));
        assert_eq!(check_shift(&ms, false, &f, &t, 1.0, 1.0), Some(1.0));
        assert_eq!(check_shift(&ms, true, &f, &t, 1.0, 1.0), Some(0.0));
        let one = parse("set_multicycle_path 3 -from [get_clocks b]\n");
        let ms = matchers(&one.exceptions, &CLOCKS);
        assert_eq!(check_shift(&ms, true, &f, &t, 1.0, 1.0), Some(2.0));
        assert_eq!(check_shift(&ms, false, &f, &t, 1.0, 1.0), Some(2.0));
    }

    /// Reference rule (`fromThruToPriority`, false path 4000 over multicycle 2000): an instance
    /// side outranks a clock side, and any false path outranks any multicycle — file order does
    /// not decide.
    #[test]
    fn the_more_specific_exception_wins() {
        let s = parse("set_multicycle_path 4 -from [get_clocks b]\nset_multicycle_path 2 -from [get_cells r2]\nset_false_path -setup -to [get_clocks a]\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        let (f, t) = (reg("r2", "b"), reg("r3", "a"));
        assert_eq!(check_shift(&ms, false, &f, &t, 1.0, 1.0), None);
        assert_eq!(check_shift(&ms, true, &f, &t, 1.0, 1.0), Some(1.0));
    }

    /// Reference rule (`all_inputs` / `all_outputs` / `all_registers` are port and instance
    /// collections): `-from [all_inputs]` covers paths from input ports only, not from registers.
    #[test]
    fn a_collection_covers_its_own_kind_only() {
        let s = parse("set_false_path -from [all_inputs]\nset_false_path -to [all_outputs]\n");
        let ms = matchers(&s.exceptions, &CLOCKS);
        let din = PathPoint { name: "din", clock: Some("a"), port: true };
        let dout = PathPoint { name: "dout", clock: Some("a"), port: true };
        assert!(std::ptr::eq(find(&ms, false, &din, &reg("r1", "a")).unwrap(), &s.exceptions[0]));
        assert!(std::ptr::eq(find(&ms, false, &reg("r3", "a"), &dout).unwrap(), &s.exceptions[1]));
        assert!(find(&ms, false, &reg("r1", "a"), &reg("r2", "b")).is_none());
    }
}
