# ===========================================================================
#  µnleashed camsat
# ===========================================================================
#
# File:         firmware/tools/fetch_core.py
# Module:       Build / the core files the satellite shares with the board
#
# Purpose:      Copies the link engine, its crypto and the picture headers
#               out of the µnleashed core at the commit core.lock pins, into
#               firmware/core/ (ignored by git). The satellite then speaks
#               exactly the link the board speaks, with no second copy of it
#               kept here to drift.
#
#               A PlatformIO pre-script (platformio.ini extra_scripts), and
#               runnable alone:  python firmware/tools/fetch_core.py
#
#               core.lock's source is a git URL or a local path; its commit is
#               a full 40-character commit, or "-" for a local path's working
#               tree as it stands (development only: the build says so).
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
import io
import os
import re
import shutil
import subprocess
import sys
import tarfile

# The files, as paths in the core. They land under firmware/core/ keeping
# their folder (core/ or plugins/), so their own includes still resolve.
FILES = [
    "src/core/link.h",
    "src/core/link.cpp",
    "src/core/linkcrypto.h",
    "src/core/linkcrypto.cpp",
    "src/core/linkfam.h",
    "src/core/crc32.h",
    "src/plugins/camera_pic.h",
    "src/plugins/camera_mark.h",
    "src/plugins/camera_brand.h",
    "src/plugins/camera_rules.h",
    "src/plugins/panel_font.h",
]


def here():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:                      # run by PlatformIO's SCons
        return os.path.join(os.getcwd(), "tools")


def read_lock(root):
    lock = {}
    with open(os.path.join(root, "core.lock"), encoding="utf-8") as f:
        for line in f:
            line = line.split(";", 1)[0].strip()
            if "=" in line:
                k, v = line.split("=", 1)
                lock[k.strip()] = v.strip()
    if "source" not in lock or "commit" not in lock:
        sys.exit("fetch_core: core.lock needs source and commit")
    return lock


def git(*args, cwd=None):
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True).stdout


def fetch(root, fw):
    lock = read_lock(root)
    src, commit = lock["source"], lock["commit"]
    out = os.path.join(fw, "core")
    stamp = os.path.join(out, ".commit")
    want = src + " " + commit
    if commit != "-" and os.path.exists(stamp) and open(stamp).read().strip() == want:
        return                                           # already there
    if commit != "-" and not re.fullmatch(r"[0-9a-f]{40}", commit):
        sys.exit("fetch_core: core.lock commit must be 40 hex characters (or - for a working tree)")
    local = not re.match(r"^[a-z]+://|^git@", src)
    if local:
        repo = os.path.normpath(os.path.join(root, src))
    else:
        repo = os.path.join(fw, ".core-cache")
        if not os.path.isdir(os.path.join(repo, ".git")):
            git("clone", "--no-checkout", src, repo)
        else:
            git("fetch", "origin", cwd=repo)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    if commit == "-":
        if not local:
            sys.exit("fetch_core: a working tree (-) needs a local source")
        print("fetch_core: DEVELOPMENT build from the working tree of %s" % repo)
        for p in FILES:
            dst = os.path.join(out, p[len("src/"):])
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(os.path.join(repo, p), dst)
    else:
        tar = git("archive", "--format=tar", commit, *FILES, cwd=repo)
        with tarfile.open(fileobj=io.BytesIO(tar)) as t:
            for m in t.getmembers():
                if not m.isfile():
                    continue
                dst = os.path.join(out, m.name[len("src/"):])
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                with open(dst, "wb") as f:
                    f.write(t.extractfile(m).read())
        with open(stamp, "w") as f:
            f.write(want + "\n")
        print("fetch_core: core files at %s" % commit[:12])


FW = os.path.dirname(here())
fetch(os.path.dirname(FW), FW)
