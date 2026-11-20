// SPDX-License-Identifier: Apache-2.0
//! The engine's debug groups and stage lines, on the shared `vyges-events` standard.
//!
//! - `VYGES_DEBUG=sta-si:<group>=<level>` turns a group on; [`DEBUG_GROUPS`] lists them and
//!   `--describe` prints the table. A group mirrors the reference's debug group of the same name
//!   where one exists; our own carry a `vy_` prefix.
//! - `VYGES_STAGE_TIMES=1` prints one line per stage (wall, CPU, RSS, peak RSS). The stages are
//!   named after the reference's functions, so a stage table lines up with a reference profile.
//! - The old switches still work, as aliases with a one-time deprecation note:
//!   `VYGES_STA_DMP_TRACE` is `dmp_ceff`, `VYGES_STA_RC_TRACE` is `vy_rc_model`.
//!   `VYGES_STA_RC_NET=<net>` still picks the one net the model trace lists.
//!
//! ⛔ A group changes only what is printed (to stderr), never a result.

use vyges_events::DebugGroup;

/// The tool name in `VYGES_DEBUG` and on every stage line.
pub const TOOL: &str = "sta-si";

/// The DMP effective-capacitance trace.
pub const DMP_CEFF: &str = "dmp_ceff";
/// Which interconnect model each arc's delay came from.
pub const RC_MODEL: &str = "vy_rc_model";

/// Every debug group the engine offers.
pub const DEBUG_GROUPS: &[DebugGroup] = &[
    DebugGroup {
        group: DMP_CEFF,
        max_level: 1,
        what: "per driver edge, the effective-capacitance solve: its Pi model, algorithm, Ceff, delay and slew (`dmp|` lines); a port drive's (`drive|` lines)",
        reference: Some("sta dmp_ceff"),
    },
    DebugGroup {
        group: RC_MODEL,
        max_level: 1,
        what: "after the delay loop, which interconnect model each arc got (transient, tree Elmore, lumped, none), nets worst first; VYGES_STA_RC_NET=<net> lists one net",
        reference: None,
    },
];

/// The old switches and the group each now names.
const ALIASES: &[(&str, &str)] = &[("VYGES_STA_DMP_TRACE", DMP_CEFF), ("VYGES_STA_RC_TRACE", RC_MODEL)];

/// Map the old switches onto their groups, once per process. Called by [`on`], so a library
/// caller that never runs `main` keeps them too.
pub fn register_aliases() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        for (var, group) in ALIASES {
            vyges_events::debug::register_alias(var, TOOL, group, 1);
        }
    });
}

/// Whether `group` is on at level 1. It reads the environment each call: check once per run
/// and keep the answer, never in a hot loop.
pub fn on(group: &str) -> bool {
    register_aliases();
    vyges_events::debug::debug_enabled(TOOL, group, 1)
}

/// The stage line guard for `name` (a reference function name), reported when dropped.
/// When `VYGES_STAGE_TIMES` is off it reads no clock or counter.
pub fn stage(name: &str) -> vyges_events::stage::Stage {
    vyges_events::stage::stage_in(TOOL, name)
}

/// `--describe`'s debug-group table as JSON.
pub fn describe() -> String {
    vyges_events::debug::describe_debug_groups(TOOL, DEBUG_GROUPS).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each old switch names a group the table offers, so the alias turns something on.
    #[test]
    fn every_alias_names_a_listed_group() {
        for (var, group) in ALIASES {
            assert!(DEBUG_GROUPS.iter().any(|g| g.group == *group), "{var} -> {group} is not in DEBUG_GROUPS");
        }
    }

    /// The rule: a group mirroring the reference keeps the reference's name; our own are `vy_`.
    #[test]
    fn groups_are_the_references_or_prefixed() {
        for g in DEBUG_GROUPS {
            assert!(g.reference.is_some() != g.group.starts_with("vy_"), "{}: a reference group or vy_, not both or neither", g.group);
        }
    }

    #[test]
    fn describe_lists_every_group() {
        let d = describe();
        for g in DEBUG_GROUPS {
            assert!(d.contains(&format!("\"group\":\"{}\"", g.group)), "{d}");
        }
    }
}
