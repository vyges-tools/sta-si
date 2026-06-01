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
pub mod liberty;
pub mod sdc;
pub mod netlist;
pub mod spef;
pub mod sta;
pub mod si;
pub mod ccs;
pub mod engine;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
