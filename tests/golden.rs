// SPDX-License-Identifier: Apache-2.0
//! Golden-log harness — every example job is a regression test of the whole binary.
//!
//! The suite this file joins compares the engine against *itself*: fixtures assert numbers the
//! engine produced, so a behaviour we never implemented has no test to fail, and a behaviour we
//! silently change has only the tests that happen to name it. Every real defect found on real
//! designs so far — the SDF writer's loads, the slew thresholds, recovery/removal checks,
//! constant propagation, clock-source→output-port paths, a SPEF that parsed to nothing — was
//! found by comparing against an artifact from *outside*, never by this suite.
//!
//! A golden log is the cheap half of that discipline turned inward: it does not know what the
//! right answer is, but it knows when the answer *changed*. Nothing here needs to be true — it
//! needs to be **stable**, and to make any drift visible in the diff rather than in a number
//! nobody re-read.
//!
//! **A case is data, not code.** Every `examples/**/*.sta` and `examples/**/*.tcl` is discovered
//! automatically and run through the real binary; the expected output lives beside it under
//! `tests/golden/`. Adding coverage means adding an example and a golden — no Rust. This is the
//! same shape OpenROAD's and OpenSTA's own suites use (`.tcl` + `.ok`, diffed after
//! canonicalization), so situations from those suites port here as data.
//!
//! ```text
//! cargo test --test golden                  # check
//! UPDATE_GOLDEN=1 cargo test --test golden  # re-bless after an intended change
//! ```
//!
//! **Re-blessing is the dangerous operation.** `UPDATE_GOLDEN=1` will happily record a
//! regression as the new truth. Read the diff first; the harness prints it precisely so that
//! reading it is easy.
use std::path::{Path, PathBuf};

const BIN: &str = env!("CARGO_BIN_EXE_vyges-sta-si");

/// Everything in the output that is true of *this run* rather than of the engine.
///
/// Without this the goldens are unstable on the second run and the harness is worse than
/// useless — it trains you to re-bless without reading. Each rule below exists because the
/// value it erases was observed to change between two identical runs.
fn canonicalize(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for line in s.lines() {
        let mut line = line.to_string();

        // vyges-events carries a wall-clock stamp on every record
        if let Some(i) = line.find("\"ts_ms\":") {
            let start = i + "\"ts_ms\":".len();
            let end = start
                + line[start..]
                    .find(|c: char| !c.is_ascii_digit())
                    .unwrap_or(0);
            line.replace_range(start..end, "<TS>");
        }

        // the crate version is printed by the tcl adapter banner; a release bump is not a
        // behaviour change and must not churn every golden
        if let Some(i) = line.find("vyges-sta-si ") {
            let start = i + "vyges-sta-si ".len();
            let end = start
                + line[start..]
                    .find(|c: char| !(c.is_ascii_digit() || c == '.'))
                    .unwrap_or(line.len() - start);
            if end > start && line[start..end].contains('.') {
                line.replace_range(start..end, "<VERSION>");
            }
        }

        // absolute paths differ per checkout and per machine
        line = line.replace(env!("CARGO_MANIFEST_DIR"), "<REPO>");

        out.push_str(line.trim_end());
        out.push('\n');
    }
    out
}

/// The recorded form of one run: exit status and both streams, so the golden covers the whole
/// contract a caller sees — a report that silently moved to stderr, or a zero exit on a failing
/// run, is exactly the kind of change that otherwise slips through.
fn run_case(case: &Path) -> String {
    let sub = match case.extension().and_then(|e| e.to_str()) {
        Some("tcl") => "tcl",
        _ => "run",
    };
    let out = std::process::Command::new(BIN)
        .arg(sub)
        .arg(case)
        .output()
        .unwrap_or_else(|e| panic!("failed to run {BIN} {sub} {}: {e}", case.display()));
    format!(
        "$ vyges-sta-si {sub} {}\nexit: {}\n--- stdout ---\n{}--- stderr ---\n{}",
        case.display(),
        out.status
            .code()
            .map(|c| c.to_string())
            .unwrap_or_else(|| "signal".into()),
        canonicalize(&String::from_utf8_lossy(&out.stdout)),
        canonicalize(&String::from_utf8_lossy(&out.stderr)),
    )
}

fn find_cases(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            find_cases(&p, out);
        } else if matches!(
            p.extension().and_then(|x| x.to_str()),
            Some("sta") | Some("tcl")
        ) {
            out.push(p);
        }
    }
}

/// `examples/seq/seq.sta` -> `tests/golden/seq__seq.sta.ok`
fn golden_path(case: &Path) -> PathBuf {
    let rel = case.strip_prefix("examples").unwrap_or(case);
    let flat = rel
        .to_string_lossy()
        .replace(std::path::MAIN_SEPARATOR, "__");
    PathBuf::from("tests/golden").join(format!("{flat}.ok"))
}

fn first_difference(want: &str, got: &str) -> String {
    let (w, g): (Vec<_>, Vec<_>) = (want.lines().collect(), got.lines().collect());
    for i in 0..w.len().max(g.len()) {
        let (a, b) = (
            w.get(i).copied().unwrap_or("<missing>"),
            g.get(i).copied().unwrap_or("<missing>"),
        );
        if a != b {
            let from = i.saturating_sub(3);
            let mut ctx = String::new();
            for (j, l) in w.iter().enumerate().take(i).skip(from) {
                ctx.push_str(&format!("  {:>4} | {l}\n", j + 1));
            }
            ctx.push_str(&format!("- {:>4} | {a}\n+ {:>4} | {b}\n", i + 1, i + 1));
            return format!("first difference at line {}:\n{ctx}", i + 1);
        }
    }
    "outputs differ only in trailing content".to_string()
}

#[test]
fn every_example_matches_its_golden_log() {
    let update = std::env::var_os("UPDATE_GOLDEN").is_some();
    let mut cases = Vec::new();
    find_cases(Path::new("examples"), &mut cases);
    cases.sort();
    assert!(
        !cases.is_empty(),
        "no example jobs found — is the working directory the crate root?"
    );

    std::fs::create_dir_all("tests/golden").expect("create tests/golden");
    let mut failures: Vec<String> = Vec::new();
    let mut blessed = 0usize;

    for case in &cases {
        let got = run_case(case);
        let gp = golden_path(case);
        match std::fs::read_to_string(&gp) {
            Ok(want) if want == got => {}
            Ok(want) => {
                if update {
                    std::fs::write(&gp, &got).expect("write golden");
                    blessed += 1;
                } else {
                    failures.push(format!(
                        "\n=== {} drifted from {} ===\n{}",
                        case.display(),
                        gp.display(),
                        first_difference(&want, &got)
                    ));
                }
            }
            Err(_) if update => {
                std::fs::write(&gp, &got).expect("write golden");
                blessed += 1;
            }
            Err(_) => failures.push(format!(
                "\n=== {} has no golden ===\nexpected {}\nrun `UPDATE_GOLDEN=1 cargo test --test golden` to record it",
                case.display(),
                gp.display()
            )),
        }
    }

    if blessed > 0 {
        eprintln!("UPDATE_GOLDEN: wrote {blessed} golden log(s) — read the diff before committing");
    }
    assert!(
        failures.is_empty(),
        "{} of {} golden log(s) drifted:{}\n\nIf the change is intended, re-bless with \
         `UPDATE_GOLDEN=1 cargo test --test golden` — after reading the diff above.",
        failures.len(),
        cases.len(),
        failures.join("")
    );
}

#[test]
fn no_golden_is_left_behind_by_a_deleted_case() {
    // A golden whose case is gone is worse than no golden: it looks like coverage, is never
    // executed, and quietly rots. Cheap to catch, so catch it.
    let mut cases = Vec::new();
    find_cases(Path::new("examples"), &mut cases);
    let expected: std::collections::HashSet<PathBuf> =
        cases.iter().map(|c| golden_path(c)).collect();

    let Ok(rd) = std::fs::read_dir("tests/golden") else {
        return;
    };
    let orphans: Vec<String> = rd
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|x| x.to_str()) == Some("ok"))
        .filter(|p| !expected.contains(p))
        .map(|p| p.display().to_string())
        .collect();
    assert!(
        orphans.is_empty(),
        "golden logs with no matching example: {orphans:?}"
    );
}

#[test]
fn the_canonicalizer_erases_exactly_what_moves_between_runs() {
    // The harness is only as good as this function: under-canonicalize and every run is a false
    // failure, over-canonicalize and real drift hides inside a placeholder. Both directions are
    // asserted here rather than assumed.
    let a = r#"{"ts_ms":1785531518402,"code":"STA-DONE"}"#;
    let b = r#"{"ts_ms":1785999999999,"code":"STA-DONE"}"#;
    assert_eq!(
        canonicalize(a),
        canonicalize(b),
        "a wall-clock stamp must not fail a run"
    );

    assert_eq!(
        canonicalize("# vyges-sta-si 0.1.18 — adapter").trim_end(),
        "# vyges-sta-si <VERSION> — adapter",
        "a release bump must not churn every golden"
    );

    // ...and the numbers the goldens exist to protect must survive untouched
    let real = "  WNS: 5.7390 ns    TNS: 0.0000 ns    [MET]";
    assert_eq!(
        canonicalize(real).trim_end(),
        real,
        "slack values must never be canonicalized away"
    );
    assert_ne!(
        canonicalize("  WNS: 5.7390 ns"),
        canonicalize("  WNS: 5.7391 ns"),
        "a one-digit slack change must still fail the diff"
    );
}
