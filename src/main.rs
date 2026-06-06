//! vyges-sta-si CLI.
//!
//!   vyges-sta-si run   JOB [-o OUT] [--json] [--fail-on-violation]   analyze -> report
//!   vyges-sta-si check JOB                                          validate the job
//!   vyges-sta-si demo  [-o OUT] [--json]                            built-in design
//!
//! `run` auto-detects an MCMM job (one with a `scenarios:` list) and reports the
//! worst setup/hold across the per-corner scenarios.
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
  --bug-report     file a bug (central: vyges/community)
  --feature-request request a feature (central)
  --sponsor        sponsor Vyges (github.com/sponsors/vyges-ip)
  --star           star this tool on GitHub ⭐
";

const BUG_URL: &str =
    "https://github.com/vyges/community/issues/new?template=bug_report_template.yaml";
const FEATURE_URL: &str = "https://github.com/vyges/community/issues/new?labels=enhancement";
const SPONSOR_URL: &str = "https://github.com/sponsors/vyges-ip";
const STAR_URL: &str = "https://github.com/vyges-tools/sta-si";

/// Print a labelled URL; if stdout is a terminal, also try to open it in a browser.
/// In headless / agent contexts (not a TTY) it just prints the URL.
fn link(label: &str, url: &str) {
    use std::io::IsTerminal;
    println!("{label}:\n  {url}");
    if std::io::stdout().is_terminal() {
        let opener = if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
        let _ = std::process::Command::new(opener).arg(url).status();
    }
}

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
    bug_report: bool,
    feature_request: bool,
    sponsor: bool,
    star: bool,
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
            "--bug-report" => c.bug_report = true,
            "--feature-request" => c.feature_request = true,
            "--sponsor" => c.sponsor = true,
            "--star" => c.star = true,
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
    if cli.fail_on_violation {
        let setup_bad = rep.endpoints > 0 && rep.wns < 0.0;
        let hold_bad = rep.hold_endpoints > 0 && rep.whs < 0.0;
        if setup_bad || hold_bad {
            if !cli.quiet {
                if setup_bad {
                    eprintln!("setup VIOLATED: WNS {:.4} ns", rep.wns);
                }
                if hold_bad {
                    eprintln!("hold VIOLATED: WHS {:.4} ns", rep.whs);
                }
            }
            exit(3);
        }
    }
    exit(0);
}

fn emit_mcmm(job: &StaJob, rep: &engine::McmmReport, cli: &Cli) -> ! {
    let text = if cli.json { engine::mcmm_json(job, rep) } else { engine::render_mcmm(job, rep) };
    write_out(&text, cli);
    if cli.fail_on_violation {
        let setup_bad = rep.worst_setup().map(|x| x.1 < 0.0).unwrap_or(false);
        let hold_bad = rep.worst_hold().map(|x| x.1 < 0.0).unwrap_or(false);
        if setup_bad || hold_bad {
            if !cli.quiet {
                eprintln!("MCMM timing VIOLATED");
            }
            exit(3);
        }
    }
    exit(0);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cli = parse_cli(&args);

    if cli.bug_report {
        return link("Report a bug (central — vyges/community)", BUG_URL);
    }
    if cli.feature_request {
        return link("Request a feature (central — vyges/community)", FEATURE_URL);
    }
    if cli.sponsor {
        return link("Sponsor Vyges", SPONSOR_URL);
    }
    if cli.star {
        return link("Star vyges-sta-si on GitHub ⭐", STAR_URL);
    }
    if cli.version {
        println!("vyges-sta-si {} ({})", vyges_sta_si::VERSION, env!("VYGES_GIT_SHA"));
        println!("{}", vyges_sta_si::COPYRIGHT);
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
            if job.is_mcmm() {
                if cli.verbose {
                    eprintln!("MCMM: {} scenario(s)", job.scenarios.len());
                }
                match engine::analyze_mcmm(&job) {
                    Ok(rep) => emit_mcmm(&job, &rep, &cli),
                    Err(e) => {
                        eprintln!("error: {e}");
                        exit(1);
                    }
                }
            }
            if cli.verbose {
                eprintln!("loaded {} ({} lib(s))", job.netlist, job.libs.len());
                if let Some(sdc_path) = &job.sdc {
                    eprintln!(
                        "  sdc: {} -> {} clock(s), {} exception(s)",
                        sdc_path,
                        job.clocks.len(),
                        job.exceptions.len()
                    );
                    // surface SDC commands we recognised but do not model — never
                    // silently drop constraints (re-parse; cheap, verbose-only).
                    if let Ok(sdc) = vyges_sta_si::sdc::Sdc::load(&job.resolve(sdc_path)) {
                        if !sdc.ignored.is_empty() {
                            let mut u = sdc.ignored.clone();
                            u.sort();
                            u.dedup();
                            eprintln!("  sdc: unsupported (ignored): {}", u.join(", "));
                        }
                    }
                }
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
