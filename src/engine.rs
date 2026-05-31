//! STA engine wiring: job → netlist + merged Liberty → analyze → report text.
//!
//! Files in / report out; no subprocess (v0 is fully self-contained). OpenSTA
//! is the correlation baseline this engine is checked against, not a runtime
//! dependency — so `analyze_job` runs anywhere.

use crate::job::StaJob;
use crate::liberty::Lib;
use crate::netlist;
use crate::spef::Spef;
use crate::sta::{self, StaError, TimingReport};

const DEMO_LIB: &str = r#"
library (demo) {
  delay_model : table_lookup;
  cell (INV) {
    pin (A) { direction : input; capacitance : 0.0015; }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        timing_sense : negative_unate;
        cell_rise (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)       { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t) { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
      }
    }
  }
}
"#;

const DEMO_NETLIST: &str = r#"
module top ( a, y );
  input a;
  output y;
  wire n1;
  INV u1 ( .A(a),  .Y(n1) );
  INV u2 ( .A(n1), .Y(y)  );
endmodule
"#;

/// A built-in 2-inverter design analyzed offline (for `demo`).
pub fn demo() -> (StaJob, TimingReport) {
    let job = StaJob {
        design: "demo".into(),
        netlist: "(builtin)".into(),
        libs: vec!["(builtin)".into()],
        spef: None,
        clock_port: "clk".into(),
        period_ns: 1.0,
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        early_derate: 1.0,
        pocv_sigma: 0.0,
        pocv_n: 3.0,
        aocv_late: vec![],
        aocv_early: vec![],
        miller: 2.0,
        xtalk_window: 0.0,
        base_dir: String::new(),
    };
    let rep = analyze_inputs(DEMO_NETLIST, DEMO_LIB, &job).unwrap_or(TimingReport {
        wns: f64::INFINITY,
        tns: 0.0,
        endpoints: 0,
        worst_endpoint: String::new(),
        worst_path: Vec::new(),
        whs: f64::INFINITY,
        ths: 0.0,
        hold_endpoints: 0,
        worst_hold_endpoint: String::new(),
        worst_hold_path: Vec::new(),
    });
    (job, rep)
}

/// Load the job's netlist + (merged) Liberty and run STA.
pub fn analyze_job(job: &StaJob) -> Result<TimingReport, StaError> {
    let nl = netlist::load(&job.resolve(&job.netlist)).map_err(|e| StaError::Parse(e.to_string()))?;
    let mut lib = Lib::default();
    for l in &job.libs {
        let one = Lib::load(&job.resolve(l)).map_err(|e| StaError::Parse(e.to_string()))?;
        lib.cells.extend(one.cells); // later libs override earlier on name clash
    }
    if lib.cells.is_empty() {
        return Err(StaError::Parse("no cells in any .lib".into()));
    }
    let spef = match &job.spef {
        Some(p) => Some(Spef::load(&job.resolve(p)).map_err(|e| StaError::Parse(e.to_string()))?),
        None => None,
    };
    sta::analyze(&nl, &lib, job, spef.as_ref())
}

/// Render a human-readable timing report.
pub fn render_report(job: &StaJob, rep: &TimingReport) -> String {
    let mut s = String::new();
    s.push_str(&format!("STA report — design {}\n", job.design));
    s.push_str(&format!(
        "  clock {}  period {:.3} ns   xtalk_miller {:.2}\n",
        job.clock_port, job.period_ns, job.miller
    ));
    let ocv = if job.pocv_sigma > 0.0 {
        format!("POCV — per-stage sigma {:.3}, {:.1}-sigma band", job.pocv_sigma, job.pocv_n)
    } else if !job.aocv_late.is_empty() || !job.aocv_early.is_empty() {
        "AOCV — depth-dependent derate table".to_string()
    } else {
        format!("flat derate — late {:.3} / early {:.3}", job.late_derate, job.early_derate)
    };
    s.push_str(&format!("  OCV: {ocv}\n"));
    s.push_str(&format!("  endpoints: {}\n", rep.endpoints));
    if rep.endpoints == 0 {
        s.push_str("  (no timing endpoints — no primary outputs reached)\n");
        return s;
    }
    let verdict = if rep.wns >= 0.0 { "MET" } else { "VIOLATED" };
    s.push_str(&format!(
        "  WNS: {:.4} ns    TNS: {:.4} ns    [{}]\n\n",
        rep.wns, rep.tns, verdict
    ));
    s.push_str(&format!(
        "  worst path to {}  (slack {:.4} ns):\n",
        rep.worst_endpoint, rep.wns
    ));
    s.push_str(&format!("    {:>9}  {:>7}   node\n", "arrival", "slew"));
    for p in &rep.worst_path {
        s.push_str(&format!("    {:9.4}  {:7.4}   {}\n", p.arrival, p.slew, p.label));
    }
    if rep.hold_endpoints > 0 {
        let hverdict = if rep.whs >= 0.0 { "MET" } else { "VIOLATED" };
        s.push_str(&format!(
            "\n  hold endpoints: {}    WHS: {:.4} ns    THS: {:.4} ns    [{}]\n",
            rep.hold_endpoints, rep.whs, rep.ths, hverdict
        ));
        s.push_str(&format!(
            "  worst hold path to {}  (slack {:.4} ns):\n",
            rep.worst_hold_endpoint, rep.whs
        ));
        s.push_str(&format!("    {:>9}  {:>7}   node\n", "arrival", "slew"));
        for p in &rep.worst_hold_path {
            s.push_str(&format!("    {:9.4}  {:7.4}   {}\n", p.arrival, p.slew, p.label));
        }
    }
    s
}

/// Render the report as machine-readable JSON (std-only, no deps).
pub fn report_json(job: &StaJob, rep: &TimingReport) -> String {
    let num = |v: f64| if v.is_finite() { format!("{v:.6}") } else { "null".to_string() };
    let mut s = String::new();
    s.push('{');
    s.push_str(&format!("\"design\":{:?},", job.design));
    s.push_str(&format!("\"clock\":{:?},", job.clock_port));
    s.push_str(&format!("\"period_ns\":{:.6},", job.period_ns));
    s.push_str(&format!("\"xtalk_miller\":{:.2},", job.miller));
    s.push_str(&format!("\"endpoints\":{},", rep.endpoints));
    s.push_str(&format!("\"wns_ns\":{},", num(rep.wns)));
    s.push_str(&format!("\"tns_ns\":{},", num(rep.tns)));
    s.push_str(&format!("\"met\":{},", rep.endpoints > 0 && rep.wns >= 0.0));
    s.push_str(&format!("\"hold_endpoints\":{},", rep.hold_endpoints));
    s.push_str(&format!("\"whs_ns\":{},", num(rep.whs)));
    s.push_str(&format!("\"ths_ns\":{},", num(rep.ths)));
    s.push_str(&format!("\"hold_met\":{},", rep.hold_endpoints > 0 && rep.whs >= 0.0));
    s.push_str(&format!("\"worst_hold_endpoint\":{:?},", rep.worst_hold_endpoint));
    s.push_str(&format!("\"worst_endpoint\":{:?},", rep.worst_endpoint));
    s.push_str("\"worst_path\":[");
    for (i, p) in rep.worst_path.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!(
            "{{\"label\":{:?},\"arrival_ns\":{:.6},\"slew_ns\":{:.6}}}",
            p.label, p.arrival, p.slew
        ));
    }
    s.push_str("]}\n");
    s
}

/// Convenience for callers that already hold parsed inputs (used by `demo`).
pub fn analyze_inputs(
    nl_text: &str,
    lib_text: &str,
    job: &StaJob,
) -> Result<TimingReport, StaError> {
    let nl = netlist::parse(nl_text).map_err(|e| StaError::Parse(e.to_string()))?;
    let lib = Lib::parse(lib_text).map_err(|e| StaError::Parse(e.to_string()))?;
    sta::analyze(&nl, &lib, job, None)
}
