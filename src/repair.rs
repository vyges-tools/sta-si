// SPDX-License-Identifier: Apache-2.0
//! Timing repair — deciding which fixes are worth making.
//!
//! This is the decision layer of the timing-driven ECO loop. Everything around it already
//! existed: the timer can apply a move and roll it back ([`Timer::checkpoint`]/`restore`),
//! violations resolve to addressable sites ([`Timer::violations`]), and [`judge`] says whether
//! a candidate earned its place. This module is the part that *chooses*.
//!
//! **It plans; it does not mutate a design database.** Speculation happens entirely in the
//! timer, where placement is not an input, so no legalization question arises while deciding.
//! The output is a [`Plan`] — an ordered list of moves with the numbers that justified each —
//! which an applier replays into the database in a single pass.
//!
//! Consequently the predicted slacks are **estimates**: they are computed against the
//! pre-repair parasitics, so an inserted cell's own wire load is not modelled. For hold repair
//! on short local nets the error is small and the *sign* of the effect is not in doubt, but a
//! plan is not sign-off — re-extract and re-time after legalization.
//!
//! Scope is deliberately narrow. v0 repairs **hold** by splicing a delay element in front of a
//! failing endpoint: local, and adding delay to a path that is too *fast* cannot break setup on
//! that same path. Setup repair wants cell resizing, which needs library equivalence classes
//! (same function, different drive) that the Liberty reader does not expose yet.
use crate::sta::{judge, Check, Move, PinSite, RevertReason, StaError, Timer, Verdict};

/// How to repair, and how hard to try.
#[derive(Debug, Clone)]
pub struct RepairOpts {
    /// Library cell to splice in. **Must be non-inverting** — the timer has no notion of logic
    /// function, so nothing here can check that for you.
    pub delay_cell: String,
    /// Stop after this many accepted fixes. `0` means no limit.
    pub max_fixes: usize,
    /// Improvement below this is treated as noise rather than progress (ns).
    pub eps: f64,
    /// Instance-name prefix for inserted cells.
    pub prefix: String,
    /// Try **swapping the endpoint's driver for a slower interchangeable cell** before
    /// inserting a delay cell. On by default: it adds no instance, so it costs no area and the
    /// cell keeps its own site instead of overlapping a neighbour.
    ///
    /// Deliberately "slower cell", not "smaller cell". Downsizing is the obvious move and often
    /// the wrong one: a weaker cell also presents **less input capacitance**, which speeds up
    /// the stage *before* it — sometimes by more than the weaker cell slows this one, making
    /// hold worse. The reliable version is a **Vt swap**: same size, same load, slower. Both are
    /// just cells in the same equivalence class, so both are tried and `judge` decides.
    ///
    /// Requires the library to carry `function` or `cell_footprint`; without them no cell can
    /// be shown interchangeable and this silently has no effect.
    pub prefer_swap: bool,
    /// How many interchangeable cells to try before falling back to insertion.
    pub max_candidates: usize,
}

impl Default for RepairOpts {
    fn default() -> Self {
        Self {
            delay_cell: String::new(),
            max_fixes: 0,
            eps: 1e-9,
            prefix: "vy_hold".into(),
            prefer_swap: true,
            max_candidates: 3,
        }
    }
}

/// One accepted fix, with the evidence for it.
#[derive(Debug, Clone)]
pub struct Fix {
    pub mv: Move,
    /// The failing endpoint (hold) or critical-path stage (setup) this was aimed at.
    pub target: PinSite,
    /// Which check the fix was aimed at — and therefore which metric `slack_*` reports.
    pub check: Check,
    /// The targeted metric before and after: WHS for a hold fix, WNS for a setup fix. A
    /// reviewer needs to see what each fix actually bought, not just the total.
    pub slack_before: f64,
    pub slack_after: f64,
}

/// A rejected candidate and why — kept because "the loop did nothing" is not a useful report.
#[derive(Debug, Clone)]
pub struct Rejection {
    pub target: PinSite,
    pub reason: RevertReason,
}

/// The result of planning: what to do, what it is predicted to achieve, and what was tried and
/// discarded.
#[derive(Debug, Clone)]
pub struct Plan {
    pub fixes: Vec<Fix>,
    pub rejected: Vec<Rejection>,
    pub whs_before: f64,
    pub whs_after: f64,
    pub ths_before: f64,
    pub ths_after: f64,
    /// Setup worst slack before/after — a hold repair spends setup margin, so this is how a
    /// reviewer sees the price.
    pub wns_before: f64,
    pub wns_after: f64,
}

impl Plan {
    /// A plan that changes nothing, carrying the timer's current metrics as the baseline.
    fn baseline(t: &Timer) -> Plan {
        Plan {
            fixes: Vec::new(),
            rejected: Vec::new(),
            whs_before: t.whs(),
            whs_after: t.whs(),
            ths_before: t.ths(),
            ths_after: t.ths(),
            wns_before: t.wns(),
            wns_after: t.wns(),
        }
    }

    /// Record an accepted fix and refresh the "after" metrics from the timer.
    fn keep(&mut self, t: &Timer, fix: Fix) {
        self.fixes.push(fix);
        self.whs_after = t.whs();
        self.ths_after = t.ths();
        self.wns_after = t.wns();
        // a site that now works should not still be listed as rejected
        if let Some(last) = self.fixes.last() {
            let label = last.target.label.clone();
            self.rejected.retain(|r| r.target.label != label);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.fixes.is_empty()
    }
    /// Hold slack gained, in ns.
    pub fn whs_gain(&self) -> f64 {
        self.whs_after - self.whs_before
    }
    /// Setup slack spent (positive means margin was given up).
    pub fn wns_cost(&self) -> f64 {
        self.wns_before - self.wns_after
    }
}

/// Plan a hold repair.
///
/// Walks failing hold endpoints worst-first, proposes a delay insertion at each, and keeps it
/// only if [`judge`] agrees. The timer is left holding the **repaired** netlist, so the caller
/// can inspect it or write it out; rejected candidates are rolled back as they are tried.
///
/// Termination is by construction: every round either accepts a fix (progress, bounded by
/// `max_fixes` and by slack becoming non-negative) or blacklists an endpoint (a finite set).
/// An endpoint that cannot be improved is never retried — the "never loop forever" rule.
/// The outcome of trying to fix one site.
enum Attempt {
    /// A fix was accepted.
    Kept(Fix),
    /// A candidate was tried and judged not worth keeping; the site is now blacklisted.
    Rejected(Rejection),
    /// Nothing left to try for this check.
    Exhausted,
}

/// Try to fix `target`'s hold violation by swapping its **driver** for a slower cell.
///
/// The endpoint of a hold violation is a sink pin, so the cell to slow down is whatever drives
/// its net — not the endpoint's own instance.
///
/// Every interchangeable cell is a candidate, cheapest (smallest) first, and `judge` decides.
/// That breadth matters: a plain downsize is the obvious move and often the wrong one, because
/// the weaker cell also presents less input capacitance and can speed the *previous* stage up
/// more than it slows this one. A same-size, same-load, higher-Vt cell has no such side effect.
///
/// Returns `None` when nothing is interchangeable — including any library with no `function`
/// or `cell_footprint` — or when no candidate earns its place, leaving insertion as the
/// fallback.
fn attempt_hold_swap(
    t: &mut Timer,
    opts: &RepairOpts,
    target: &PinSite,
) -> Result<Option<Fix>, StaError> {
    let Some(net) = target.net.clone() else {
        return Ok(None);
    };
    // the driver is the instance with an output pin on this net
    let driver = t.netlist().insts.iter().find_map(|i| {
        let drives = i.conns.iter().any(|(pin, n)| {
            *n == net
                && t.lib()
                    .cells
                    .get(&i.cell)
                    .and_then(|c| c.pins.get(pin))
                    .map(|p| p.direction == crate::liberty::Dir::Out)
                    .unwrap_or(false)
        });
        drives.then(|| (i.name.clone(), i.cell.clone()))
    });
    let Some((inst, master)) = driver else {
        return Ok(None);
    };

    let candidates: Vec<String> = t
        .lib()
        .equivalence_class(&master)
        .into_iter()
        .filter(|c| c.name != master) // the identity swap is not a fix
        .take(if opts.max_candidates == 0 { usize::MAX } else { opts.max_candidates })
        .map(|c| c.name.clone())
        .collect();

    for cell in candidates {
        let before = t.report().clone();
        let snapshot = t.checkpoint();
        let mv = Move::Resize { inst: inst.clone(), cell };
        if !t.stage(mv.clone()) {
            continue;
        }
        t.update()?;
        let after = t.report().clone();
        if let Verdict::Keep = judge(&before, &after, Check::Hold, opts.eps) {
            return Ok(Some(Fix {
                mv,
                target: target.clone(),
                check: Check::Hold,
                slack_before: before.whs,
                slack_after: after.whs,
            }));
        }
        t.restore(snapshot);
    }
    Ok(None)
}

/// Try to improve the worst remaining hold violation by one fix.
///
/// `tried` blacklists sites already given up on — the "never loop forever" rule. `seq` names
/// inserted cells and must be owned by the caller, so that a combined run does not restart the
/// numbering and collide with a cell it inserted earlier.
fn attempt_hold(
    t: &mut Timer,
    opts: &RepairOpts,
    tried: &mut Vec<String>,
    seq: &mut usize,
) -> Result<Attempt, StaError> {
    if opts.delay_cell.is_empty() {
        return Ok(Attempt::Exhausted);
    }
    let Some(v) = t
        .violations(Check::Hold, 0)
        .into_iter()
        .find(|v| v.site.is_instance_pin() && !tried.contains(&v.site.label))
    else {
        return Ok(Attempt::Exhausted);
    };

    let target = v.site.clone();

    // Prefer downsizing the driver of this endpoint: a weaker cell is slower, which is what
    // hold wants, and it adds no instance — no area, and the cell keeps its own site rather
    // than overlapping a neighbour. Falls through to insertion if nothing smaller works.
    if opts.prefer_swap {
        if let Some(fix) = attempt_hold_swap(t, opts, &target)? {
            return Ok(Attempt::Kept(fix));
        }
    }

    let mv = Move::InsertDelay {
        inst: target.inst.clone().unwrap(),
        pin: target.pin.clone().unwrap(),
        cell: opts.delay_cell.clone(),
        name: format!("{}{}", opts.prefix, seq),
    };

    let before = t.report().clone();
    let snapshot = t.checkpoint();
    if !t.stage(mv.clone()) {
        // did not apply at all (bad cell, name clash) — do not retry this site
        tried.push(target.label.clone());
        return Ok(Attempt::Rejected(Rejection { target, reason: RevertReason::NoImprovement }));
    }
    t.update()?;
    let after = t.report().clone();

    match judge(&before, &after, Check::Hold, opts.eps) {
        Verdict::Keep => {
            *seq += 1;
            Ok(Attempt::Kept(Fix {
                mv,
                target,
                check: Check::Hold,
                slack_before: before.whs,
                slack_after: after.whs,
            }))
        }
        Verdict::Revert(reason) => {
            t.restore(snapshot);
            tried.push(target.label.clone());
            Ok(Attempt::Rejected(Rejection { target, reason }))
        }
    }
}

/// Plan a hold repair.
///
/// Walks failing hold endpoints worst-first, proposes a delay insertion at each, and keeps it
/// only if [`judge`] agrees. The timer is left holding the **repaired** netlist; rejected
/// candidates are rolled back as they are tried.
///
/// Termination is by construction: every round either accepts a fix (bounded by `max_fixes` and
/// by slack reaching zero) or blacklists an endpoint, and the set of endpoints is finite.
pub fn plan_hold_repair(t: &mut Timer, opts: &RepairOpts) -> Result<Plan, StaError> {
    let mut plan = Plan::baseline(t);
    let (mut tried, mut seq) = (Vec::new(), 0usize);
    loop {
        if opts.max_fixes != 0 && plan.fixes.len() >= opts.max_fixes {
            break;
        }
        match attempt_hold(t, opts, &mut tried, &mut seq)? {
            Attempt::Kept(fix) => plan.keep(t, fix),
            Attempt::Rejected(r) => plan.rejected.push(r),
            Attempt::Exhausted => break,
        }
    }
    Ok(plan)
}

/// Schema tag written into every emitted plan. Bump it if the shape changes incompatibly —
/// an applier that does not recognise the tag must refuse rather than guess.
pub const PLAN_SCHEMA: &str = "vyges-eco-plan-v1";

impl Plan {
    /// Serialize as an **ECO plan** — the interchange between planning and applying.
    ///
    /// The two sides are deliberately joined by a *file*, not a library call: the planner lives
    /// in the timer and the applier lives in the database layer, and neither should have to link
    /// the other. It also makes the plan a reviewable artifact — someone can read what is about
    /// to happen to their design before it happens.
    ///
    /// `design` is carried so an applier can refuse a plan aimed at a different block.
    ///
    /// Each fix names the target as `"inst/pin"`, which is exactly the addressing the ODB
    /// applier already uses for buffer insertion.
    pub fn to_json(&self, design: &str) -> String {
        let num = |v: f64| {
            if v.is_finite() {
                format!("{v:.6}")
            } else {
                "null".to_string()
            }
        };
        let mut s = String::new();
        s.push('{');
        s.push_str(&format!("\"schema\":{PLAN_SCHEMA:?},"));
        s.push_str(&format!("\"design\":{design:?},"));
        s.push_str(&format!("\"fix_count\":{},", self.fixes.len()));
        s.push_str("\"metrics\":{");
        s.push_str(&format!("\"whs_before_ns\":{},", num(self.whs_before)));
        s.push_str(&format!("\"whs_after_ns\":{},", num(self.whs_after)));
        s.push_str(&format!("\"ths_before_ns\":{},", num(self.ths_before)));
        s.push_str(&format!("\"ths_after_ns\":{},", num(self.ths_after)));
        s.push_str(&format!("\"wns_before_ns\":{},", num(self.wns_before)));
        s.push_str(&format!("\"wns_after_ns\":{}", num(self.wns_after)));
        s.push_str("},");
        s.push_str("\"fixes\":[");
        for (i, f) in self.fixes.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            match &f.mv {
                Move::InsertDelay { inst, pin, cell, name } => {
                    s.push_str("{\"op\":\"insert_delay\",");
                    s.push_str(&format!("\"target\":{:?},", format!("{inst}/{pin}")));
                    s.push_str(&format!("\"inst\":{inst:?},"));
                    s.push_str(&format!("\"pin\":{pin:?},"));
                    s.push_str(&format!("\"cell\":{cell:?},"));
                    s.push_str(&format!("\"name\":{name:?},"));
                }
                Move::Resize { inst, cell } => {
                    s.push_str("{\"op\":\"resize\",");
                    s.push_str(&format!("\"target\":{inst:?},"));
                    s.push_str(&format!("\"inst\":{inst:?},"));
                    s.push_str(&format!("\"cell\":{cell:?},"));
                }
            }
            s.push_str(&format!(
                "\"check\":{:?},",
                match f.check {
                    Check::Setup => "setup",
                    Check::Hold => "hold",
                }
            ));
            s.push_str(&format!("\"slack_before_ns\":{},", num(f.slack_before)));
            s.push_str(&format!("\"slack_after_ns\":{}", num(f.slack_after)));
            s.push('}');
        }
        s.push_str("],");
        s.push_str("\"rejected\":[");
        for (i, r) in self.rejected.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!(
                "{{\"target\":{:?},\"reason\":{:?}}}",
                r.target.label,
                format!("{:?}", r.reason)
            ));
        }
        s.push_str("]}");
        s
    }
}

/// How to repair setup, and how hard to try.
#[derive(Debug, Clone)]
pub struct SetupRepairOpts {
    /// Stop after this many accepted fixes. `0` means no limit.
    pub max_fixes: usize,
    /// Improvement below this is treated as noise rather than progress (ns).
    pub eps: f64,
    /// How many larger cells to try at each site before giving up on it. Upsizing is not
    /// monotonic in slack — a bigger cell is faster but loads its own driver more — so the
    /// smallest upsize is not always the best one, and occasionally none of them help.
    pub max_candidates: usize,
}

impl Default for SetupRepairOpts {
    fn default() -> Self {
        Self { max_fixes: 0, eps: 1e-9, max_candidates: 3 }
    }
}

/// Plan a setup repair by **upsizing** cells on the critical path.
///
/// Where hold repair works from a list of failing endpoints, setup repair works from the
/// critical *path*: the fix that helps is a bigger drive on whichever **arc** is costing the
/// most, and a long path is rarely uniformly slow. So each round takes the worst path, picks
/// its most expensive stage, and tries progressively larger interchangeable cells for the
/// instance driving it.
///
/// Candidates come from [`Lib::upsize_candidates`](crate::liberty::Lib::upsize_candidates),
/// which will return nothing at all for a timing-only library — equivalence is not knowable
/// without `function` or `cell_footprint`, and guessing would swap in a cell that computes
/// something else. A repair that cannot prove a replacement is safe declines to make one.
///
/// Upsizing is **not** monotonic: a larger cell drives its own load faster but presents more
/// capacitance to the stage before it, so the smallest upsize is not reliably the best and
/// sometimes none help. Hence `max_candidates`, and hence every candidate being judged rather
/// than assumed.
///
/// Termination is by construction: each round either accepts a fix (bounded by `max_fixes` and
/// by WNS reaching zero) or blacklists a site, and the set of sites on a path is finite.
/// Try to improve setup by one fix: upsize the instance driving the most expensive arc on the
/// critical path.
fn attempt_setup(
    t: &mut Timer,
    opts: &SetupRepairOpts,
    tried: &mut Vec<String>,
) -> Result<Attempt, StaError> {
    if t.wns() >= 0.0 {
        return Ok(Attempt::Exhausted);
    }
    // `stage_delay` is the arrival delta into this pin — what that arc cost.
    let Some(stage) = t
        .worst_path_stages()
        .into_iter()
        .filter(|s| s.site.is_instance_pin() && !tried.contains(&s.site.label))
        .max_by(|a, b| {
            a.stage_delay.partial_cmp(&b.stage_delay).unwrap_or(std::cmp::Ordering::Equal)
        })
    else {
        return Ok(Attempt::Exhausted);
    };

    let target = stage.site.clone();
    let inst = target.inst.clone().unwrap();
    let Some(master) = target.master.clone() else {
        tried.push(target.label.clone());
        return Ok(Attempt::Rejected(Rejection { target, reason: RevertReason::NoImprovement }));
    };

    // Collect names up front: the library borrow must end before staging.
    let candidates: Vec<String> = t
        .lib()
        .upsize_candidates(&master)
        .into_iter()
        .take(if opts.max_candidates == 0 { usize::MAX } else { opts.max_candidates })
        .map(|c| c.name.clone())
        .collect();

    if candidates.is_empty() {
        // No interchangeable larger cell — usually a library with no `function` or
        // `cell_footprint`, where equivalence is unknowable and guessing would change what the
        // design computes.
        tried.push(target.label.clone());
        return Ok(Attempt::Rejected(Rejection { target, reason: RevertReason::NoImprovement }));
    }

    let mut last = RevertReason::NoImprovement;
    for cell in candidates {
        let before = t.report().clone();
        let snapshot = t.checkpoint();
        let mv = Move::Resize { inst: inst.clone(), cell };
        if !t.stage(mv.clone()) {
            continue;
        }
        t.update()?;
        let after = t.report().clone();
        match judge(&before, &after, Check::Setup, opts.eps) {
            Verdict::Keep => {
                return Ok(Attempt::Kept(Fix {
                    mv,
                    target,
                    check: Check::Setup,
                    slack_before: before.wns,
                    slack_after: after.wns,
                }))
            }
            Verdict::Revert(reason) => {
                t.restore(snapshot);
                last = reason;
            }
        }
    }
    tried.push(target.label.clone());
    Ok(Attempt::Rejected(Rejection { target, reason: last }))
}

/// Plan a setup repair by **upsizing** cells on the critical path.
///
/// Where hold repair works from a list of failing endpoints, setup repair works from the
/// critical *path*: the fix that helps is a bigger drive on whichever **arc** is costing the
/// most, and a long path is rarely uniformly slow. So each round takes the worst path, picks
/// its most expensive stage, and tries progressively larger interchangeable cells for the
/// instance driving it.
///
/// Candidates come from [`Lib::upsize_candidates`](crate::liberty::Lib::upsize_candidates),
/// which returns nothing for a library carrying no `function` or `cell_footprint` — equivalence
/// is not knowable there, and a repair that cannot prove a replacement is safe declines to make
/// one.
///
/// Upsizing is **not** monotonic: a larger cell drives its own load faster but presents more
/// capacitance to the stage before it, so the smallest upsize is not reliably best and
/// sometimes none help. Hence `max_candidates`, and hence judging every candidate.
pub fn plan_setup_repair(t: &mut Timer, opts: &SetupRepairOpts) -> Result<Plan, StaError> {
    let mut plan = Plan::baseline(t);
    let mut tried = Vec::new();
    loop {
        if opts.max_fixes != 0 && plan.fixes.len() >= opts.max_fixes {
            break;
        }
        match attempt_setup(t, opts, &mut tried)? {
            Attempt::Kept(fix) => plan.keep(t, fix),
            Attempt::Rejected(r) => plan.rejected.push(r),
            Attempt::Exhausted => break,
        }
    }
    Ok(plan)
}

/// How to repair both checks together.
#[derive(Debug, Clone, Default)]
pub struct CombinedOpts {
    pub hold: RepairOpts,
    pub setup: SetupRepairOpts,
    /// Total accepted fixes across both checks. `0` means no limit.
    pub max_fixes: usize,
}

/// Repair **setup and hold together**, worst-violation first.
///
/// A real block needs both, and they pull against each other: upsizing shortens paths and eats
/// hold margin, inserting delay lengthens them and eats setup margin. Running one rule to
/// completion and then the other lets the second undo the first's headroom.
///
/// So this attacks whichever check is *further* into violation each round. What stops the two
/// from fighting is not the ordering but [`judge`], which already refuses any fix that pushes
/// the other check into violation or deepens an existing one — so a repair that would start an
/// oscillation is rejected before it happens.
///
/// A check is set aside when it runs out of sites, and reconsidered as soon as the *other* one
/// lands a fix, since that changes the timing it gave up on. Termination: every round either
/// accepts a fix (bounded by `max_fixes`, and by both checks meeting) or sets a check aside,
/// and a check can only be revived by a fix.
pub fn plan_repair(t: &mut Timer, opts: &CombinedOpts) -> Result<Plan, StaError> {
    let mut plan = Plan::baseline(t);
    let (mut hold_tried, mut setup_tried) = (Vec::new(), Vec::new());
    let (mut hold_done, mut setup_done) = (false, false);
    let mut seq = 0usize;

    loop {
        if opts.max_fixes != 0 && plan.fixes.len() >= opts.max_fixes {
            break;
        }
        let (wns, whs) = (t.wns(), t.whs());
        // whichever is further into violation; a met check is never chosen
        let want_hold = whs < 0.0 && (wns >= 0.0 || whs <= wns);
        let want_setup = wns < 0.0 && !want_hold;

        let check = match (want_hold && !hold_done, want_setup && !setup_done) {
            (true, _) => Check::Hold,
            (_, true) => Check::Setup,
            // the preferred check is set aside — fall back to the other if it is still violating
            _ if whs < 0.0 && !hold_done => Check::Hold,
            _ if wns < 0.0 && !setup_done => Check::Setup,
            _ => break, // both met, or both set aside
        };

        let attempt = match check {
            Check::Hold => attempt_hold(t, &opts.hold, &mut hold_tried, &mut seq)?,
            Check::Setup => attempt_setup(t, &opts.setup, &mut setup_tried)?,
        };

        match attempt {
            Attempt::Kept(fix) => {
                plan.keep(t, fix);
                // the other check may now be fixable where it was not — give it another go
                match check {
                    Check::Hold => setup_done = false,
                    Check::Setup => hold_done = false,
                }
            }
            Attempt::Rejected(r) => plan.rejected.push(r),
            Attempt::Exhausted => match check {
                Check::Hold => hold_done = true,
                Check::Setup => setup_done = true,
            },
        }
    }
    Ok(plan)
}
