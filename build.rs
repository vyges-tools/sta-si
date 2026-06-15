//! Bakes the build's git commit into the binary as `VYGES_GIT_SHA`, so
//! `--version` reports exactly which commit a user is running — essential for
//! tracing bug reports back to a build.
//!
//! Resolution order:
//!   1. `VYGES_GIT_SHA` env at build time — wins (CI / release / tarball builds).
//!   2. `git rev-parse --short HEAD` from the checkout, with a `-dirty` suffix
//!      when the working tree has uncommitted changes.
//!   3. `unknown` — no override and no git.

use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/logs/HEAD");
    println!("cargo:rerun-if-env-changed=VYGES_GIT_SHA");
    println!("cargo:rustc-env=VYGES_GIT_SHA={}", build_sha());
}

fn build_sha() -> String {
    if let Ok(s) = std::env::var("VYGES_GIT_SHA") {
        let s = s.trim();
        if !s.is_empty() {
            return s.to_string();
        }
    }
    match git(&["rev-parse", "--short", "HEAD"]) {
        Some(sha) => {
            let dirty = git(&["status", "--porcelain"]).is_some_and(|s| !s.trim().is_empty());
            if dirty { format!("{sha}-dirty") } else { sha }
        }
        None => "unknown".to_string(),
    }
}

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
    if s.is_empty() { None } else { Some(s) }
}
