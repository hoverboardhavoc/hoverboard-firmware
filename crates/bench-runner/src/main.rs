//! Tier-3 silicon driver for the test harness: the same `harness-smoke` image, on real GD32
//! silicon, reaching its verdict the same RAM-channel way the emulator does, but over SWD.
//!
//! This is the MANUAL, bench-locked tier. It is NOT run in CI (no hardware there) and is not part
//! of any automated gate. It exists so the emulator's verdict can be tied back to silicon: the
//! emulator is best-effort, the bench is the authoritative oracle.
//!
//! What it does, per the harness spec's bench-execution contract:
//!   1. Holds `tools/bench-lock.sh` before any hardware op (a distinct `BENCH_OWNER` per agent) and
//!      releases it on every exit path (a Drop guard, so a panic still releases).
//!   2. Attaches to the named probe by serial (the bench has near-duplicate ST-Link clones; they
//!      are disambiguated by full serial, not USB port) and selects the GD32 part. The GD32 needs
//!      `CPUTAPID = 0`, which probe-rs applies via the chip's target description; here the chip name
//!      is passed on the command line (e.g. `STM32F103C8` for the F103 master, `STM32F130x` family
//!      maps the F130 slave) so probe-rs loads the right description.
//!   3. Flashes the subject ELF over SWD.
//!   4. For each command word: reset the core, write the command word to `CMD_ADDR`, run, poll
//!      `RESULT_ADDR` magic (a non-halting memory read) until `SMOKE_MAGIC` or a timeout, read
//!      `SmokeObs`, and assert `verdict == VERDICT_PASS`. A reset between commands is the cold
//!      re-entry, mirroring the emulator's SP/PC reset between commands.
//!
//! The ST-Link clones do NOT drive NRST, so a subject that faults / `wfi`-parks before writing
//! magic can lock the F130's SWD; the subject busy-spins (never `wfi`) and the poll uses a timeout
//! to catch a hung subject rather than waiting forever. See `~/notes/bench-overview.md`.
//!
//! NOTE: this has NOT been run on silicon (no hardware in this environment, and the ST-Link clones
//! are mid-firmware-upgrade so probe-rs may not drive the F130 clone yet). It is written and
//! host-compile-checked only.

use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use harness_abi::{smoke, CMD_ADDR, RESULT_ADDR, SMOKE_MAGIC, VERDICT_PASS};
use probe_rs::flashing::{download_file, Format};
use probe_rs::probe::list::Lister;
use probe_rs::{MemoryInterface, Permissions};

/// The two command words the smoke test pins, identical to the emulator's. A correct echo for each
/// proves the silicon actually read the delivered word (the XOR would not match a constant).
const COMMANDS: [u32; 2] = [0xDEAD_0000, 0x0000_BEEF];

/// How long to poll `RESULT_ADDR` magic before declaring the subject hung. The smoke subject reaches
/// its magic write in microseconds; this is a generous backstop, since the clones cannot drive NRST
/// to recover a wedged part, so waiting forever is the failure mode to avoid.
const POLL_TIMEOUT: Duration = Duration::from_secs(5);
const POLL_INTERVAL: Duration = Duration::from_millis(10);

fn main() -> Result<()> {
    let args = Args::parse()?;

    // Acquire the bench lock around ALL hardware ops; the guard releases on every exit path
    // (including a panic via Drop). No bench op runs without it.
    let _lock = BenchLock::acquire(&args.owner, &format!("bench-runner: {}", args.chip))?;

    run(&args)
}

/// Run the smoke sequence on silicon. Held inside the bench lock by `main`.
fn run(args: &Args) -> Result<()> {
    // Pick the probe by full serial (the bench clones share near-duplicate mangled serials and the
    // USB port drifts, so the serial is the only stable identifier).
    let lister = Lister::new();
    let probes = lister.list_all();
    let info = probes
        .iter()
        .find(|p| p.serial_number.as_deref() == Some(args.probe_serial.as_str()))
        .ok_or_else(|| {
            anyhow!(
                "no probe with serial {:?}; found: {:?}",
                args.probe_serial,
                probes
                    .iter()
                    .map(|p| p.serial_number.clone())
                    .collect::<Vec<_>>()
            )
        })?;

    let probe = lister
        .open(info)
        .with_context(|| format!("open probe {}", args.probe_serial))?;

    // Attach + select the part. probe-rs loads the chip's target description (which carries the
    // GD32 CPUTAPID = 0 and flash map) from the chip name. Permissions::default() is read/program,
    // no full-chip erase needed for this subject.
    let mut session = probe
        .attach(args.chip.as_str(), Permissions::default())
        .with_context(|| format!("attach to {}", args.chip))?;

    // Flash the subject ELF over SWD.
    download_file(&mut session, &args.elf, Format::Elf(Default::default()))
        .with_context(|| format!("flash {}", args.elf.display()))?;

    let mut failures = 0u32;
    for &cmd in &COMMANDS {
        let obs = run_phase(&mut session, cmd)?;
        let expect_echo = smoke(cmd);
        let pass = obs.magic == SMOKE_MAGIC && obs.verdict == VERDICT_PASS;
        println!(
            "cmd {:#010x}: magic={:#010x} verdict={} echo={:#010x} (expect echo {:#010x}) -> {}",
            cmd,
            obs.magic,
            obs.verdict,
            obs.echo,
            expect_echo,
            if pass { "PASS" } else { "FAIL" }
        );
        if !pass {
            failures += 1;
        }
    }

    if failures != 0 {
        bail!(
            "{failures} of {} smoke command(s) failed on silicon",
            COMMANDS.len()
        );
    }
    println!("all {} smoke commands passed on silicon", COMMANDS.len());
    Ok(())
}

/// A decoded `SmokeObs` read back over SWD.
struct Obs {
    magic: u32,
    verdict: u32,
    echo: u32,
}

/// One command phase: reset (cold re-entry), deliver the command word, run, poll magic until the
/// sentinel or the timeout, then read the struct back.
fn run_phase(session: &mut probe_rs::Session, cmd: u32) -> Result<Obs> {
    let mut core = session.core(0).context("select core 0")?;

    // Reset and halt so we can prime RAM before the subject runs. A halted reset means the command
    // word and the cleared magic land before the first instruction executes.
    core.reset_and_halt(Duration::from_secs(1))
        .context("reset_and_halt")?;

    // Clear the magic word so a stale value from a previous run cannot read as completion, then
    // deliver the command word to its fixed tail address.
    core.write_word_32(RESULT_ADDR as u64, 0)
        .context("clear magic")?;
    core.write_word_32(CMD_ADDR as u64, cmd)
        .context("write command word")?;

    // Resume; the subject reads the command, computes + self-checks, writes the result with magic
    // last, then busy-spins.
    core.run().context("resume core")?;

    // Poll magic non-halting until the sentinel or the timeout. The subject keeps spinning, so the
    // poll is a plain memory read with the core running.
    let start = Instant::now();
    loop {
        let magic = core
            .read_word_32(RESULT_ADDR as u64)
            .context("poll magic")?;
        if magic == SMOKE_MAGIC {
            break;
        }
        if start.elapsed() > POLL_TIMEOUT {
            bail!(
                "timeout waiting for magic (cmd {cmd:#010x}); last magic {magic:#010x} (subject hung or faulted before completing)"
            );
        }
        std::thread::sleep(POLL_INTERVAL);
    }

    // Read the whole struct: magic (already the sentinel), verdict, echo, by fixed offset.
    let magic = core
        .read_word_32(RESULT_ADDR as u64)
        .context("read magic")?;
    let verdict = core
        .read_word_32(RESULT_ADDR as u64 + 4)
        .context("read verdict")?;
    let echo = core
        .read_word_32(RESULT_ADDR as u64 + 8)
        .context("read echo")?;
    Ok(Obs {
        magic,
        verdict,
        echo,
    })
}

// --- bench lock guard --------------------------------------------------------------------------

/// RAII guard around `tools/bench-lock.sh`: acquires on construction, releases on Drop (so a panic
/// or an early `?` return still releases). Only releases a lock this process acquired.
struct BenchLock {
    script: PathBuf,
    owner: String,
}

impl BenchLock {
    fn script_path() -> PathBuf {
        // crates/bench-runner/src -> repo root (../../.. from the manifest dir) /tools/bench-lock.sh
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|root| root.join("tools/bench-lock.sh"))
            .expect("locate tools/bench-lock.sh")
    }

    fn acquire(owner: &str, note: &str) -> Result<Self> {
        let script = Self::script_path();
        let out = Command::new(&script)
            .args(["acquire", owner, note])
            .output()
            .with_context(|| format!("run {}", script.display()))?;
        let stdout = String::from_utf8_lossy(&out.stdout);
        if !out.status.success() {
            bail!(
                "bench is busy, not running: {}",
                stdout.trim().lines().next().unwrap_or("HELD")
            );
        }
        Ok(BenchLock {
            script,
            owner: owner.to_string(),
        })
    }
}

impl Drop for BenchLock {
    fn drop(&mut self) {
        // Best-effort release; the script only releases a lock this owner holds.
        let _ = Command::new(&self.script)
            .args(["release", &self.owner])
            .status();
    }
}

// --- arg parsing -------------------------------------------------------------------------------

struct Args {
    elf: PathBuf,
    chip: String,
    probe_serial: String,
    owner: String,
}

impl Args {
    fn parse() -> Result<Self> {
        // Minimal positional + env parsing, no clap dependency (this is a small bench tool).
        //   bench-runner <elf> <chip> <probe-serial>
        //   BENCH_OWNER  (env, default bench-runner@host) attributes the lock to this agent.
        let mut a = std::env::args().skip(1);
        let elf = a
            .next()
            .ok_or_else(|| anyhow!("usage: bench-runner <elf> <chip> <probe-serial>"))?;
        let chip = a
            .next()
            .ok_or_else(|| anyhow!("usage: bench-runner <elf> <chip> <probe-serial>"))?;
        let probe_serial = a
            .next()
            .ok_or_else(|| anyhow!("usage: bench-runner <elf> <chip> <probe-serial>"))?;
        let owner = std::env::var("BENCH_OWNER").unwrap_or_else(|_| {
            format!(
                "bench-runner@{}",
                hostname().unwrap_or_else(|| "host".to_string())
            )
        });
        Ok(Args {
            elf: PathBuf::from(elf),
            chip,
            probe_serial,
            owner,
        })
    }
}

fn hostname() -> Option<String> {
    Command::new("hostname")
        .arg("-s")
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
}
