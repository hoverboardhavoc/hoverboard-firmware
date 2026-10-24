#!/usr/bin/env python3
"""Watch the rider phone for the thing that ends an arm level, read-only.

The arm level the boards hold comes from the app's 20 Hz stream, and the app drops it the
moment its Activity gets ON_STOP (MainActivity.DisarmWhenBackgrounded -> forceDisarm). The
display timing out IS an ON_STOP, so a long armed run on an untouched phone ends at the
screen timeout with no link event and nothing logged. See specs/ble-reliability.md.

Two modes, both read-only on the phone (nothing here writes a setting or taps anything):

  check [seconds]   preconditions for an armed run of that length: the screen timeout, the
                    stay-awake setting, the app process, the GATT connection age. Exit 1 if
                    the run cannot stay armed to the end.
  watch [all]       stream lifecycle and screen events with HOST-clock timestamps, so a
                    capture taken on this machine can be aligned to them afterwards. New
                    events only; `watch all` replays the event buffer first.
"""
import re
import subprocess
import sys
import time

SERIAL = "192.168.0.201:5555"
PKG = "com.hoverboard.remote"
ADB = ["adb", "-s", SERIAL]

# ON_STOP is the event that disarms. The screen ones are here because the screen timing out
# is what produces it when nobody has touched the phone.
INTERESTING = re.compile(
    r"wm_on_(stop|paused|resume|start|restart)_called|screen_toggled|am_kill|am_proc_(start|died)"
)


def sh(*a):
    return subprocess.run(ADB + list(a), capture_output=True, text=True).stdout.strip()


def skew():
    """Phone clock minus host clock, in seconds. The event log is stamped on the phone."""
    h0 = time.time()
    p = float(sh("shell", "date +%s.%N") or 0)
    h1 = time.time()
    return p - (h0 + h1) / 2


def check(secs):
    bad = []
    timeout_ms = sh("shell", "settings get system screen_off_timeout")
    stayon = sh("shell", "settings get global stay_on_while_plugged_in")
    pid = sh("shell", "pidof " + PKG)
    gatt = sh("shell", "dumpsys bluetooth_manager")

    try:
        timeout_s = int(timeout_ms) / 1000.0
    except ValueError:
        timeout_s = 0.0
    held = stayon not in ("", "0", "null")
    print("screen_off_timeout      %s ms (%.0f s)" % (timeout_ms, timeout_s))
    print("stay_on_while_plugged_in %s%s" % (stayon, "" if held else "   (screen WILL time out)"))
    if not held and timeout_s < secs:
        bad.append(
            "the screen times out %.0f s into a %.0f s run, which disarms the boards"
            % (timeout_s, secs)
        )
    print("app pid                 %s" % (pid or "NOT RUNNING"))
    if not pid:
        bad.append("the app is not running")

    # One line per live GATT connection: "<date> - <age>ms : <addr> (<app id>)".
    conns = [l.strip() for l in gatt.splitlines() if re.search(r"- \d+ms : ", l)]
    print("gatt connections        %s" % (" | ".join(conns) or "none"))
    if not conns:
        bad.append("no GATT connection is up")

    for b in bad:
        print("FAIL: " + b)
    if not bad:
        print("PASS: nothing here will end the arm level inside %.0f s" % secs)
    print(
        "\nNote: `svc power stayon true` is SIGKILLed on this phone (exit 137) and "
        "`settings put global` is refused (no WRITE_SECURE_SETTINGS), so the stay-awake "
        "setting can only be turned on by hand, in Developer options."
    )
    return 0 if not bad else 1


def watch(replay):
    off = skew()
    print("# phone clock is %+.3f s against this host; times below are HOST clock" % off)
    cmd = ADB + ["logcat", "-b", "events", "-v", "epoch"]
    if not replay:
        cmd += ["-T", "1"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    for line in p.stdout:
        if not INTERESTING.search(line):
            continue
        if "hoverboard" not in line and "screen_toggled" not in line:
            continue
        f = line.split(None, 4)
        if len(f) < 5:
            continue
        try:
            host = float(f[0]) - off
        except ValueError:
            continue
        print(
            "%s  %s" % (time.strftime("%H:%M:%S", time.localtime(host)), f[4].strip()),
            flush=True,
        )


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "check"
    if mode == "check":
        sys.exit(check(float(sys.argv[2]) if len(sys.argv) > 2 else 600.0))
    elif mode == "watch":
        watch(replay=len(sys.argv) > 2 and sys.argv[2] == "all")
    else:
        sys.exit(__doc__)
