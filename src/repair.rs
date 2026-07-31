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
}

impl Default for RepairOpts {
    fn default() -> Self {
        Self {
            delay_cell: String::new(),
            max_fixes: 0,
            eps: 1e-9,
            prefix: "vy_hold".into(),
        }
    }
}

/// One accepted fix, with the evidence for it.
#[derive(Debug, Clone)]
pub struct Fix {
    pub mv: Move,
    /// The failing endpoint this was aimed at.
    pub target: PinSite,
    /// Worst hold slack before and after, so a reviewer can see what it bought.
    pub whs_before: f64,
    pub whs_after: f64,
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
pub fn plan_hold_repair(t: &mut Timer, opts: &RepairOpts) -> Result<Plan, StaError> {
    let mut plan = Plan {
        fixes: Vec::new(),
        rejected: Vec::new(),
        whs_before: t.whs(),
        whs_after: t.whs(),
        ths_before: t.ths(),
        ths_after: t.ths(),
        wns_before: t.wns(),
        wns_after: t.wns(),
    };
    if opts.delay_cell.is_empty() {
        return Ok(plan);
    }

    let mut tried: Vec<String> = Vec::new();
    loop {
        if opts.max_fixes != 0 && plan.fixes.len() >= opts.max_fixes {
            break;
        }
        // worst failing hold endpoint we have not already given up on
        let Some(v) = t
            .violations(Check::Hold, 0)
            .into_iter()
            .find(|v| v.site.is_instance_pin() && !tried.contains(&v.site.label))
        else {
            break;
        };

        let target = v.site.clone();
        let mv = Move::InsertDelay {
            inst: target.inst.clone().unwrap(),
            pin: target.pin.clone().unwrap(),
            cell: opts.delay_cell.clone(),
            name: format!("{}{}", opts.prefix, plan.fixes.len()),
        };

        let before = t.report().clone();
        let snapshot = t.checkpoint();
        if !t.stage(mv.clone()) {
            // the move did not apply at all (bad cell, name clash) — do not retry this endpoint
            tried.push(target.label.clone());
            continue;
        }
        t.update()?;
        let after = t.report().clone();

        match judge(&before, &after, Check::Hold, opts.eps) {
            Verdict::Keep => {
                plan.fixes.push(Fix {
                    mv,
                    target,
                    whs_before: before.whs,
                    whs_after: after.whs,
                });
                plan.whs_after = after.whs;
                plan.ths_after = after.ths;
                plan.wns_after = after.wns;
            }
            Verdict::Revert(reason) => {
                t.restore(snapshot);
                tried.push(target.label.clone());
                plan.rejected.push(Rejection { target, reason });
            }
        }
    }
    Ok(plan)
}
