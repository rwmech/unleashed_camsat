# µnleashed camsat: the satellite's firmware.
# Copyright 2026 - Robert Mech. SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.16.0)
# The version in the image, set here rather than from git describe (which
# fails on a repository with no commits and says nothing useful in a clone).
set(PROJECT_VER "1.0.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(camsat)
