#!/usr/bin/env python3
"""Tap and read the rider app by node text, for screens target/bench/app.py has no command for.

Same discipline as app.py: the view hierarchy is the source of truth, a control is found by its
own label, never by pixel coordinates.
"""
import os
import re
import subprocess
import sys
import time

SERIAL = "192.168.0.201:5555"
ADB = ["adb", "-s", SERIAL]
NODE = re.compile(r"<node[^>]*>")
ATTR = re.compile(r'(\w[\w-]*)="([^"]*)"')
BOUNDS = re.compile(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]")
DUMP = "/sdcard/ui-%d.xml" % os.getpid()


def sh(*a, binary=False):
    p = subprocess.run(ADB + list(a), capture_output=True)
    return p.stdout if binary else p.stdout.decode(errors="replace")


def nodes():
    sh("shell", "uiautomator", "dump", DUMP)
    xml = sh("shell", "cat", DUMP)
    out = []
    for m in NODE.finditer(xml):
        a = dict(ATTR.findall(m.group(0)))
        b = BOUNDS.search(a.get("bounds", ""))
        if b:
            x1, y1, x2, y2 = (int(v) for v in b.groups())
            a["_c"] = ((x1 + x2) // 2, (y1 + y2) // 2)
            a["_h"] = y2 - y1
        out.append(a)
    return out


def find(ns, needle):
    needle = needle.lower()
    hits = [n for n in ns
            if needle in (n.get("text", "") + " " + n.get("content-desc", "")).lower()]
    return hits


cmd = sys.argv[1]
if cmd == "text":
    seen = []
    for n in nodes():
        for k in ("text", "content-desc"):
            t = n.get(k, "").strip()
            if t and t not in seen:
                seen.append(t)
    print("\n".join(seen))
elif cmd == "tap":
    needle = sys.argv[2]
    hits = find(nodes(), needle)
    if not hits:
        sys.exit("ui: nothing matching %r on screen" % needle)
    n = hits[0] if len(sys.argv) < 4 else hits[int(sys.argv[3])]
    x, y = n["_c"]
    sh("shell", "input", "tap", str(x), str(y))
    print("tapped %r at %d,%d (%d match(es))" % (needle, x, y, len(hits)))
    time.sleep(1.2)
elif cmd == "scroll":
    sh("shell", "input", "swipe", "540", "1800", "540", "700", "350")
    time.sleep(1.0)
    print("scrolled")
