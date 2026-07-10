//! vyges-sta-si CLI.
//!
//!   vyges-sta-si run   JOB [-o OUT] [--json] [--fail-on-violation] [--sdf FILE]  analyze -> report
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
  vyges-sta-si run      JOB    [-o OUT] [--json] [--fail-on-violation] [--sdf FILE]
  vyges-sta-si sdc-lint JOB    [-o OUT] [--json] [--fail-on-violation]
  vyges-sta-si check    JOB
  vyges-sta-si demo            [-o OUT] [--json]
  vyges-sta-si tcl      SCRIPT [-o OUT] [--json] [--fail-on-violation]   (experimental)

`sdc-lint` checks the SDC for completeness/consistency (unconstrained I/O, a clock with
no period, duplicate clocks, a clock on a port the design lacks) — independent of timing.

`tcl` runs an OpenSTA-style TCL *subset* (read_liberty/verilog/spef/sdc + inline SDC +
report_checks/report_wns/report_tns) through the Vyges engine — EXPERIMENTAL; not a TCL
interpreter and not a drop-in for LibreLane's corner.tcl. See docs/opensta-integration.md.

flags:
  -o FILE               write output to FILE (default: stdout)
  --json                machine-readable JSON instead of the text report
  --fail-on-violation   exit 3 if WNS < 0 (CI timing gate)
  --pdk NAME           resolve liberty from pdk-store (lib) when the job has none
  --corner C           PDK corner for --pdk (default: the PDK's default corner)
  --sdf FILE            also write an SDF back-annotation file (IOPATH + setup/hold,
                        + INTERCONNECT from SPEF) — feeds gate-level / back-annotated sim
  -q, --quiet           suppress non-essential output
  -v, --verbose         extra detail on stderr
  --describe            print a machine-readable JSON description of the command
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
    sdf: Option<String>,
    help: bool,
    version: bool,
    bug_report: bool,
    feature_request: bool,
    sponsor: bool,
    star: bool,
    pdk: Option<String>,
    corner: Option<String>,
}

/// Resolve a PDK collateral key (e.g. `lib`) to a concrete path via the installed
/// `vyges-pdk-store` resolver — the PDK adapter. Prefers the sibling binary next
/// to this one, else falls back to PATH. On failure returns the resolver's own
/// message (e.g. `"foo": not a known PDK — run list…`) so the caller can surface it.
fn pdk_resolve(pdk: &str, key: &str, corner: Option<&str>) -> Result<String, String> {
    let sibling = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("vyges-pdk-store")))
        .filter(|p| p.exists())
        .map(|p| p.to_string_lossy().into_owned());
    let prog = sibling.unwrap_or_else(|| "vyges-pdk-store".into());
    let mut cmd = std::process::Command::new(prog);
    cmd.args(["resolve", pdk, key]);
    if let Some(c) = corner {
        cmd.args(["--corner", c]);
    }
    let out = cmd.output().map_err(|e| format!("vyges-pdk-store not runnable: {e}"))?;
    if !out.status.success() {
        let err = String::from_utf8_lossy(&out.stderr).trim().trim_start_matches("error:").trim().to_string();
        return Err(if err.is_empty() { format!("could not resolve {key} for PDK {pdk:?}") } else { err });
    }
    let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
    if s.is_empty() {
        Err(format!("pdk-store returned no path for {key} of PDK {pdk:?}"))
    } else {
        Ok(s)
    }
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
            "--sdf" => {
                c.sdf = args.get(i + 1).cloned();
                i += 1;
            }
            "--pdk" => {
                c.pdk = args.get(i + 1).cloned();
                i += 1;
            }
            "--corner" => {
                c.corner = args.get(i + 1).cloned();
                i += 1;
            }
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

fn render_lint(r: &vyges_sta_si::sdclint::LintReport) -> String {
    let mut s = String::new();
    if r.findings.is_empty() {
        s.push_str("vyges-sta-si sdc-lint — CLEAN ✓  (no constraint issues)\n");
        return s;
    }
    s.push_str(&format!(
        "vyges-sta-si sdc-lint — {} error(s), {} warning(s)\n",
        r.errors(),
        r.warnings()
    ));
    for f in &r.findings {
        s.push_str(&format!("  {:7} [{}] {}\n", f.severity.tag(), f.code, f.message));
    }
    s
}

fn render_lint_json(r: &vyges_sta_si::sdclint::LintReport) -> String {
    let mut s = String::from("{\n");
    s.push_str(&format!("  \"errors\": {},\n", r.errors()));
    s.push_str(&format!("  \"warnings\": {},\n", r.warnings()));
    s.push_str("  \"findings\": [\n");
    for (i, f) in r.findings.iter().enumerate() {
        let comma = if i + 1 < r.findings.len() { "," } else { "" };
        s.push_str(&format!(
            "    {{\"severity\": \"{}\", \"code\": \"{}\", \"message\": {:?}}}{}\n",
            f.severity.tag(),
            f.code,
            f.message,
            comma
        ));
    }
    s.push_str("  ]\n}\n");
    s
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

/// Emit for the experimental `tcl` adapter: OpenSTA-flavoured text (or our JSON),
/// honouring the script's `report_*` requests. Same fail-on-violation gate as `emit`.
fn emit_tcl(
    job: &StaJob,
    rep: &TimingReport,
    reports: &vyges_sta_si::tcl::Reports,
    cli: &Cli,
) -> ! {
    let text = if cli.json {
        engine::report_json(job, rep)
    } else {
        vyges_sta_si::tcl::render(job, rep, reports)
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

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--describe") {
        // Machine-readable description of `run` for tooling that drives it.
        const DESCRIBE: &str = r#"{
  "name": "sta-si",
  "summary": "static timing analysis with signal integrity (job → report)",
  "invocation": {
    "args_template": ["run", "{job}"],
    "optional": [ { "arg": "sdf", "flag": "--sdf" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["job"],
    "properties": {
      "job": { "type": "string", "description": "the timing job file" },
      "sdf": { "type": "string", "description": "optional SDF delays file" }
    }
  },
  "artifacts": [ { "role": "timing_report" }, { "role": "sdf", "from_arg": "sdf" } ],
  "consumes": ["netlist", "liberty", "spef"]
}
"#;
        print!("{DESCRIBE}");
        return;
    }
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
            let mut job = match StaJob::load(path) {
                Ok(j) => j,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            };
            // no liberty in the job? resolve it from --pdk via pdk-store (`lib`,
            // for --corner or the PDK default corner).
            if job.libs.is_empty() {
                if let Some(p) = &cli.pdk {
                    match pdk_resolve(p, "lib", cli.corner.as_deref()) {
                        Ok(path) => job.libs.push(path),
                        Err(e) => {
                            eprintln!("error: {e}");
                            exit(2);
                        }
                    }
                }
            }
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
            if let Some(sdf_out) = &cli.sdf {
                match engine::sdf_for_job(&job) {
                    Ok(text) => {
                        if let Err(e) = std::fs::write(sdf_out, &text) {
                            eprintln!("error: {sdf_out}: {e}");
                            exit(1);
                        }
                        if !cli.quiet {
                            eprintln!("wrote SDF: {sdf_out}");
                        }
                    }
                    Err(e) => {
                        eprintln!("error: {e}");
                        exit(1);
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
        "sdc-lint" => {
            let Some(path) = cli.positionals.get(1) else {
                eprintln!("usage: vyges-sta-si sdc-lint JOB [-o OUT]");
                exit(2);
            };
            let job = match StaJob::load(path) {
                Ok(j) => j,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            };
            let report = match engine::lint_job(&job) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(1);
                }
            };
            let text = if cli.json { render_lint_json(&report) } else { render_lint(&report) };
            write_out(&text, &cli);
            // gate: errors always fail; --fail-on-violation also fails on warnings.
            let fail = report.errors() > 0 || (cli.fail_on_violation && report.warnings() > 0);
            if fail {
                exit(3);
            }
        }
        "tcl" => {
            // EXPERIMENTAL: OpenSTA-TCL-subset adapter (Layer 1).
            let Some(path) = cli.positionals.get(1) else {
                eprintln!("usage: vyges-sta-si tcl SCRIPT [-o OUT]");
                exit(2);
            };
            let adapted = match vyges_sta_si::tcl::adapt(path) {
                Ok(a) => a,
                Err(e) => {
                    eprintln!("error: {e}");
                    exit(2);
                }
            };
            // Never silently drop: surface OpenSTA commands outside the subset.
            if !adapted.ignored.is_empty() && !cli.quiet {
                eprintln!(
                    "note: {} OpenSTA command(s) outside the supported subset, ignored: {}\n      (experimental adapter — see docs/opensta-integration.md)",
                    adapted.ignored.len(),
                    adapted.ignored.join(", ")
                );
            }
            match engine::analyze_job(&adapted.job) {
                Ok(rep) => emit_tcl(&adapted.job, &rep, &adapted.reports, &cli),
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
