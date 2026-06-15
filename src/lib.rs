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

pub mod job;
// Parsers + CCS data model now come from the shared vyges-loom foundation
// (sta-si was loom's seed — these originated here). Re-exported under the crate
// root so `crate::liberty` / `crate::sdc` / … keep resolving across the engine.
pub use vyges_loom::{ccs, liberty, netlist, sdc, spef};
pub mod sta;
pub mod si;
pub mod engine;
pub mod tcl; // experimental: OpenSTA-TCL-subset adapter (Layer 1)

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const COPYRIGHT: &str = "© 2026 Vyges. All Rights Reserved.  https://vyges.com";
