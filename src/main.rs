//! vyges-sta-si CLI.
//!
//!   vyges-sta-si run   JOB [-o OUT.rpt]   analyze -> timing report
//!   vyges-sta-si check JOB                parse + validate the job, print summary
//!   vyges-sta-si demo  [-o OUT.rpt]       analyze a built-in 2-gate design

use std::process::exit;

use vyges_sta_si::engine;
use vyges_sta_si::job::StaJob;

fn arg_after(args: &[String], flag: &str) -> Option<String> {
    args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1)).cloned()
}

fn write_out(text: &str, out: Option<String>) {
    match out {
        Some(path) => match std::fs::write(&path, text) {
            Ok(_) => println!("wrote {path}"),
            Err(e) => {
                eprintln!("error: {path}: {e}");
                exit(1);
            }
        },
        None => print!("{text}"),
    }
}

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
        cell_rise (t)        { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.08, 0.20", "0.12, 0.28" ); }
        cell_fall (t)        { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.07, 0.18", "0.11, 0.26" ); }
        rise_transition (t)  { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.09", "0.04, 0.11" ); }
        fall_transition (t)  { index_1 ("0.01, 0.08"); index_2 ("0.001, 0.01"); values ( "0.03, 0.08", "0.04, 0.10" ); }
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

fn demo_report() -> String {
    let job = StaJob {
        design: "demo".into(),
        netlist: "(builtin)".into(),
        libs: vec!["(builtin)".into()],
        clock_port: "clk".into(),
        period_ns: 1.0,
        input_slew: 0.02,
        output_load: 0.005,
        late_derate: 1.0,
        base_dir: String::new(),
    };
    match engine::analyze_inputs(DEMO_NETLIST, DEMO_LIB, &job) {
        Ok(rep) => engine::render_report(&job, &rep),
        Err(e) => format!("demo error: {e}\n"),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cmd = args.first().map(String::as_str).unwrap_or("");
    match cmd {
        "--version" | "-V" => println!("vyges-sta-si {}", vyges_sta_si::VERSION),
        "demo" => write_out(&demo_report(), arg_after(&args, "-o")),
        "check" => {
            let Some(path) = args.get(1) else {
                eprintln!("usage: vyges-sta-si check JOB");
                exit(2);
            };
            match StaJob::load(path) {
                Ok(j) => println!(
                    "OK  design={} netlist={} libs={} clock={}@{}ns",
                    j.design, j.netlist, j.libs.len(), j.clock_port, j.period_ns
                ),
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            }
        }
        "run" => {
            let Some(path) = args.get(1) else {
                eprintln!("usage: vyges-sta-si run JOB [-o OUT.rpt]");
                exit(2);
            };
            let job = match StaJob::load(path) {
                Ok(j) => j,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            };
            match engine::analyze_job(&job) {
                Ok(rep) => write_out(&engine::render_report(&job, &rep), arg_after(&args, "-o")),
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(1);
                }
            }
        }
        _ => {
            eprintln!(
                "vyges-sta-si {}\nusage: vyges-sta-si <run|check|demo|--version>",
                vyges_sta_si::VERSION
            );
            exit(2);
        }
    }
}
