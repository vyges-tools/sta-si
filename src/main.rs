//! vyges-sta-si CLI.
//!
//!   vyges-sta-si run   JOB [-o OUT] [--json] [--fail-on-violation]   analyze -> report
//!   vyges-sta-si check JOB                                          validate the job
//!   vyges-sta-si demo  [-o OUT] [--json]                            built-in design
//!
//! Common flags: -h/--help, -V/--version, -q/--quiet, -v/--verbose.
//! Exit codes: 0 ok · 1 runtime/analysis error · 2 usage/validation · 3 timing
//! violation (only with --fail-on-violation).

use std::process::exit;

use vyges_sta_si::engine;
use vyges_sta_si::job::StaJob;
use vyges_sta_si::sta::TimingReport;

const USAGE: &str = "\
vyges-sta-si — sign-off static timing analysis with signal integrity

usage:
  vyges-sta-si run   JOB [-o OUT] [--json] [--fail-on-violation]
  vyges-sta-si check JOB
  vyges-sta-si demo  [-o OUT] [--json]

flags:
  -o FILE               write output to FILE (default: stdout)
  --json                machine-readable JSON instead of the text report
  --fail-on-violation   exit 3 if WNS < 0 (CI timing gate)
  -q, --quiet           suppress non-essential output
  -v, --verbose         extra detail on stderr
  -h, --help            show this help
  -V, --version         show version
";

#[derive(Default)]
struct Cli {
    positionals: Vec<String>,
    out: Option<String>,
    json: bool,
    quiet: bool,
    verbose: bool,
    fail_on_violation: bool,
    help: bool,
    version: bool,
}

fn parse_cli(args: &[String]) -> Cli {
    let mut c = Cli::default();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-o" => {
                c.out = args.get(i + 1).cloned();
                i += 1;
            }
            "--json" => c.json = true,
            "--fail-on-violation" => c.fail_on_violation = true,
            "-q" | "--quiet" => c.quiet = true,
            "-v" | "--verbose" => c.verbose = true,
            "-h" | "--help" => c.help = true,
            "-V" | "--version" => c.version = true,
            other => c.positionals.push(other.to_string()),
        }
        i += 1;
    }
    c
}

fn write_out(text: &str, cli: &Cli) {
    match &cli.out {
        Some(path) => match std::fs::write(path, text) {
            Ok(_) => {
                if !cli.quiet {
                    println!("wrote {path}");
                }
            }
            Err(e) => {
                eprintln!("error: {path}: {e}");
                exit(1);
            }
        },
        None => print!("{text}"),
    }
}

fn emit(job: &StaJob, rep: &TimingReport, cli: &Cli) -> ! {
    let text = if cli.json {
        engine::report_json(job, rep)
    } else {
        engine::render_report(job, rep)
    };
    write_out(&text, cli);
    if cli.fail_on_violation && rep.endpoints > 0 && rep.wns < 0.0 {
        if !cli.quiet {
            eprintln!("timing VIOLATED: WNS {:.4} ns", rep.wns);
        }
        exit(3);
    }
    exit(0);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cli = parse_cli(&args);

    if cli.version {
        println!("vyges-sta-si {}", vyges_sta_si::VERSION);
        return;
    }
    let cmd = cli.positionals.first().cloned().unwrap_or_default();
    if cli.help || cmd.is_empty() {
        print!("{USAGE}");
        exit(if cmd.is_empty() && !cli.help { 2 } else { 0 });
    }

    match cmd.as_str() {
        "demo" => {
            let (job, rep) = engine::demo();
            emit(&job, &rep, &cli);
        }
        "check" => {
            let Some(path) = cli.positionals.get(1) else {
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
            let Some(path) = cli.positionals.get(1) else {
                eprintln!("usage: vyges-sta-si run JOB [-o OUT]");
                exit(2);
            };
            let job = match StaJob::load(path) {
                Ok(j) => j,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            };
            if cli.verbose {
                eprintln!("loaded {} ({} lib(s))", job.netlist, job.libs.len());
            }
            match engine::analyze_job(&job) {
                Ok(rep) => emit(&job, &rep, &cli),
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(1);
                }
            }
        }
        other => {
            eprintln!("vyges-sta-si: unknown command {other:?}\n");
            print!("{USAGE}");
            exit(2);
        }
    }
}
