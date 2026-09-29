#!/usr/bin/env python3
# ===========================================================================
#  µnleashed camsat: the camera satellite's firmware
# ===========================================================================
#
# File:         tools/release.py
# Module:       Tools / release build
#
# Purpose:      Build a camsat release from a clean tree: the camsat env,
#               checked, into release/<version>/assets/ as the directory's
#               fetcher reads it (unleashed_site deploy/fetch_release.py,
#               PRODUCTS' camera sat row): bootloader.bin, partitions.bin and
#               firmware.bin (one factory app partition, so three parts, not
#               the board's five), version.txt, THIRD_PARTY_NOTICES.md and
#               SHA256SUMS.
#
#               python3 tools/release.py --tag v1.1.0
#
#               Refuses: a tag that is not v<CAMSAT_VERSION>; uncommitted
#               changes; a core.lock that is a local path or not a full
#               commit; a notice line naming Anthropic or Claude, or a
#               GPL-2.0 SPDX line, in any tracked file; an image that does not
#               carry its version, or that mentions either name.
#
# Copyright 2026 - Robert Mech
# License:      GNU General Public License v3 or later
# SPDX-License-Identifier: GPL-3.0-or-later
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation; either version 3 of the License, or (at your
# option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.
# ===========================================================================
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "firmware"
ENV = "camsat"
PARTS = ("bootloader.bin", "partitions.bin", "firmware.bin")
NOTICES = "THIRD_PARTY_NOTICES.md"

# The same two rules as the core's release.py: the copyright is Robert
# Mech's alone, and the project is GPL-3.0-or-later.
NOTICE_LINE = re.compile(
    r"^\W*(copyright|\(c\)|©|spdx-filecopyrighttext|spdx-license-identifier|"
    r"licen[cs]ed?\s+(to|by)|authors?\s*:|maintainers?\s*:|co-authored-by\s*:|"
    r"generated\s+(with|by))[^\n]*\b(anthropic|claude)\b",
    re.I | re.M)
OLD_SPDX = re.compile(r"^\W*SPDX-License-Identifier:[^\n]*\bGPL-2\.0", re.M)


def die(msg):
    """Stop the release with a reason."""
    print("release: " + msg, file=sys.stderr)
    sys.exit(1)


def git(*args, cwd=ROOT):
    """A git command's output, or the release stops."""
    r = subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True)
    if r.returncode:
        die("git %s: %s" % (" ".join(args), r.stderr.strip()))
    return r.stdout.strip()


def version():
    """CAMSAT_VERSION from firmware/src/board.h."""
    m = re.search(r'#define\s+CAMSAT_VERSION\s+"([^"]+)"',
                  (FW / "src" / "board.h").read_text(encoding="utf-8"))
    if not m:
        die("CAMSAT_VERSION not found in firmware/src/board.h")
    return m.group(1)


def core_lock():
    """core.lock's source and commit; a release is built from a URL at a full commit."""
    lock = {}
    for ln in (ROOT / "core.lock").read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if ln and not ln.startswith(";") and "=" in ln:
            k, v = ln.split("=", 1)
            lock[k.strip()] = v.strip()
    src, commit = lock.get("source", ""), lock.get("commit", "")
    if not re.match(r"^[a-z]+://|^git@", src):
        die("core.lock's source is a local path; a release builds from the core's URL")
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        die("core.lock's commit must be a full 40-character commit")
    return src, commit


def check_notices():
    """No tracked file names Anthropic or Claude in a notice line, or says GPL-2.0."""
    bad, old = [], []
    for rel in git("ls-files").splitlines():
        try:
            text = (ROOT / rel).read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue                          # binary
        for m in OLD_SPDX.finditer(text):
            old.append("%s:%d" % (rel, text.count("\n", 0, m.start()) + 1))
        for m in NOTICE_LINE.finditer(text):
            bad.append("%s:%d" % (rel, text.count("\n", 0, m.start()) + 1))
    if old:
        die("GPL-2.0 SPDX lines; the project is GPL-3.0-or-later: " + ", ".join(old[:10]))
    if bad:
        die("copyright or licence lines name Anthropic or Claude: " + ", ".join(bad[:10]))


def framework_dir():
    """The ESP-IDF package the build used, found by its version (5.3.1)."""
    base = Path.home() / ".platformio" / "packages"
    for d in sorted(base.glob("framework-espidf*")):
        pj = d / "package.json"
        if pj.exists() and json.loads(pj.read_text()).get("version") == "3.50301.0":
            return d
    die("framework-espidf 3.50301.0 (ESP-IDF 5.3.1) not found under ~/.platformio/packages")


def notices(fw, commit):
    """THIRD_PARTY_NOTICES.md from the licence files of what the image links."""
    mc = FW / "managed_components"
    # The Spleen font's licence lives in the core, beside the font the
    # watermark draws with; read it from the core cache at the locked commit.
    spleen = subprocess.run(["git", "show", commit + ":tools/fonts/SPLEEN-LICENSE"],
                            cwd=FW / ".core-cache", capture_output=True, text=True)
    if spleen.returncode:
        die("could not read the Spleen licence from the core cache: " + spleen.stderr.strip())
    items = [
        ("ESP-IDF 5.3.1 (Espressif)", "Apache-2.0", fw / "LICENSE"),
        ("FreeRTOS kernel", "MIT", fw / "components/freertos/FreeRTOS-Kernel/LICENSE.md"),
        ("lwIP TCP/IP stack", "BSD-3-Clause", fw / "components/lwip/lwip/COPYING"),
        ("Mbed TLS", "Apache-2.0", fw / "components/mbedtls/mbedtls/LICENSE"),
        ("Espressif Wi-Fi libraries", "see text", fw / "components/esp_wifi/lib/LICENSE"),
        ("Espressif PHY libraries", "see text", fw / "components/esp_phy/lib/LICENSE"),
        ("newlib C library", "see text", fw / "components/newlib/COPYING.NEWLIB"),
        ("esp32-camera 2.1.7 (Espressif)", "Apache-2.0", mc / "espressif__esp32-camera/LICENSE"),
        ("TJpgDec (ChaN), in the chip's ROM", "TJpgDec licence (BSD-style)",
         mc / "espressif__esp_jpeg/tjpgd/tjpgd.c"),
        ("Spleen bitmap font 2.2.0 (Frederic Cambus)", "BSD-2-Clause", None),
    ]
    out = ["# Third-party notices",
           "",
           "The µnleashed camsat firmware is free software under the GNU General",
           "Public License, version 3 or later. The release image also contains the",
           "following components, each under its own licence, reproduced below from",
           "the exact packages this release was built with.",
           ""]
    for name, lic, _ in items:
        out.append("- %s: %s" % (name, lic))
    for name, lic, path in items:
        if path is None:
            text = spleen.stdout
        else:
            if not path.exists():
                die("licence file missing: %s" % path)
            text = path.read_text(encoding="utf-8", errors="replace")
            if path.name == "tjpgd.c":          # the notice is the file header
                text = text.split("*/", 1)[0] if "*/" in text else "\n".join(text.splitlines()[:25])
        out += ["", "---", "", "## " + name, "", "```", text.rstrip(), "```"]
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description="Build a camsat release.")
    ap.add_argument("--tag", help="the git tag being released: v<CAMSAT_VERSION>")
    ap.add_argument("--allow-dirty", action="store_true",
                    help="build from a working tree with uncommitted changes")
    a = ap.parse_args()

    ver = version()
    if a.tag is not None and a.tag != "v" + ver:
        die("tag %s does not match CAMSAT_VERSION %s (want v%s)" % (a.tag, ver, ver))
    dirty = git("status", "--porcelain", "--untracked-files=no")
    if dirty and not a.allow_dirty:
        die("the working tree has uncommitted changes; commit, or --allow-dirty to test")
    _, commit = core_lock()
    check_notices()

    # A fresh build: fetch_core.py (pre-script) copies the core at the locked
    # commit into firmware/core/ before anything compiles.
    r = subprocess.run(["pio", "run", "-d", str(FW), "-e", ENV])
    if r.returncode:
        die("pio run -e %s failed" % ENV)
    build = FW / ".pio" / "build" / ENV
    blobs = {}
    for name in PARTS:
        p = build / name
        if not p.exists():
            die("%s did not produce %s" % (ENV, p))
        blobs[name] = p.read_bytes()
    if ver.encode("ascii") not in blobs["firmware.bin"]:
        die("firmware.bin does not carry the version %r" % ver)
    for name, data in blobs.items():
        if re.search(rb"(?i)anthropic|claude", data):
            die("%s mentions Anthropic or Claude; the images carry no such credit" % name)

    out = ROOT / "release" / ver
    if out.exists():
        shutil.rmtree(out)
    assets = out / "assets"
    assets.mkdir(parents=True)
    sums = []
    for name, data in blobs.items():
        (assets / name).write_bytes(data)
        sums.append("%s  %s" % (hashlib.sha256(data).hexdigest(), name))
    vtxt = (ver + "\n").encode("ascii")
    (assets / "version.txt").write_bytes(vtxt)
    sums.append("%s  version.txt" % hashlib.sha256(vtxt).hexdigest())
    note = notices(framework_dir(), commit).encode("utf-8")
    (assets / NOTICES).write_bytes(note)
    sums.append("%s  %s" % (hashlib.sha256(note).hexdigest(), NOTICES))
    (assets / "SHA256SUMS").write_text("\n".join(sums) + "\n", encoding="ascii")

    head = git("rev-parse", "--short", "HEAD") + ("-dirty" if dirty else "")
    print("release camsat %s (%s, core %s)" % (ver, head, commit[:7]))
    for line in sums:
        print("  " + line)
    print("assets: %s" % assets)


if __name__ == "__main__":
    main()
