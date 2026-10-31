//! vyges-sta-si — sign-off static timing analysis with signal integrity.
//!
//! Closes the loop with the other engines: `vyges-char` emits the Liberty
//! (`.lib`) timing models and `vyges-extract` emits the SPEF parasitics; this
//! engine *consumes* them. Given a gate-level netlist + Liberty + a clock, it
//! builds a timing graph, propagates arrival/required times (NLDM delay from
//! slew × load, plus OCV derate), and reports slack — WNS/TNS and the worst
//! path.
//!
//! Boundaries (per the Vyges flow architecture): inputs and outputs are files
//! (Verilog netlist + Liberty [+ SPEF] in, a timing report out). The whole v0
//! is pure std and unit-tested offline — there is no subprocess. The external
//! tool (OpenSTA) is the *correlation baseline*, not a runtime dependency.
//!
//! v0 scope: combinational max-delay timing (primary input → primary output)
//! with NLDM cell delays and late OCV derate. Crosstalk delta-delay (the SI
//! layer), SPEF-driven net delay, and sequential (register) timing build on the
//! same graph; the engine reserves the SI hook (`StaError::SiNotModeled`).

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_sta_si::…` paths resolve here.
extern crate self as vyges_sta_si;

pub mod job;
// Parsers + CCS data model now come from the shared vyges-loom foundation
// (sta-si was loom's seed — these originated here). Re-exported under the crate
// root so `crate::liberty` / `crate::sdc` / … keep resolving across the engine.
pub use vyges_loom::{ccs, dmp, liberty, names, netlist, sdc, spef, yosys_json};

/// Timing repair — deciding which fixes are worth making (the ECO planner).
pub mod repair;
pub mod engine;
mod inc;
// SDC constraint linter + its metadata reader: in loom since 2026-10-30, shared with `vyges sdc`.
pub use vyges_loom::{ipmeta, sdc_lint as sdclint};
pub mod sdf; // SDF (Standard Delay Format) back-annotation writer
pub mod si;
pub mod sta;
pub mod tcl; // experimental: OpenSTA-TCL-subset adapter (Layer 1)

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const COPYRIGHT: &str = "© 2026 Vyges. All Rights Reserved.  https://vyges.com";

/// The `vyges-sta-si` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
