//! Links `linkall.x` (esp-hal) and records the git commit for `ESP_INFO`.

use std::fs;
use std::path::Path;
use std::process::Command;

fn main() {
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");

    let commit = git(&["rev-parse", "--short=8", "HEAD"]).unwrap_or_default();
    println!("cargo:rustc-env=HOVERBOARD_BRIDGE_COMMIT={commit}");
    watch_head();
}

/// `git` run in the crate's directory; its trimmed output, or `None` if it failed.
fn git(args: &[&str]) -> Option<String> {
    Command::new("git")
        .args(args)
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
}

/// Rerun when `HEAD` or the branch it names moves, in a plain checkout and in a worktree
/// (`specs/hoverboard-bridge.md`, "Installing and updating the bridge", "The image's commit
/// stamp"). A worktree's `.git` is a file, so the git directory comes from `git rev-parse
/// --git-dir`; its branch refs live in the common directory shared with the main checkout.
fn watch_head() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let Some(git_dir) = git(&["rev-parse", "--git-dir"]).map(|d| dir.join(d)) else {
        return;
    };
    let common = git(&["rev-parse", "--git-common-dir"])
        .map(|d| dir.join(d))
        .unwrap_or_else(|| git_dir.clone());
    let head = git_dir.join("HEAD");
    println!("cargo:rerun-if-changed={}", head.display());
    let Some(target) = fs::read_to_string(&head)
        .ok()
        .and_then(|h| h.strip_prefix("ref: ").map(|r| r.trim().to_string()))
    else {
        return;
    };
    // A loose ref is rewritten by a commit; a packed one changes `packed-refs`. A ref that is
    // only packed has no loose file, and watching a missing file reruns every build, which is
    // also correct.
    println!("cargo:rerun-if-changed={}", common.join(&target).display());
    let packed = common.join("packed-refs");
    if packed.exists() {
        println!("cargo:rerun-if-changed={}", packed.display());
    }
}

// The undefined-symbol hints esp-hal's templates print (from ~/dev/hoverboard-esp/build.rs).
fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                "_stack_start" => {
                    eprintln!();
                    eprintln!("Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "`esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "free"
                | "malloc"
                | "calloc"
                | "get_free_internal_heap_size"
                | "malloc_internal"
                | "realloc_internal"
                | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
