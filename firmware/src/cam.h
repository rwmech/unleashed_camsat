// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/cam.h
// Module:       Taking the picture
//
// Purpose:      One picture, start to finish, on the camera task: bring the
//               sensor up cold, wait for its exposure and white balance to
//               settle (the core's campic::Settle and Steady), light the
//               flash if asked, take the frame, put the sensor down, then do
//               the pixel work the board's built-in camera does: Auto
//               levels and gamma (campic), the watermark (cammark) and the
//               JPEG comment (camrules::ComSink), re-encoded once. The
//               result is a whole JPEG in PSRAM, behind 16 bytes left free
//               for the link's picture header, ready to go as one bulk
//               message.
//
//               The board chooses the words (board, when, who, comment);
//               the satellite only draws them.
//
// Copyright 2026 - Robert Mech
// License:      GNU General Public License v3 or later
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.
// ===========================================================================
#pragma once
#include <cstddef>
#include <cstdint>

// The picture settings the board sends (SETTINGS), as the satellite keeps them.
struct PicSettings {
    uint8_t size     = 5;        // linkfam CS_* (5: XGA)
    uint8_t quality  = 10;       // the sensor's JPEG quality, 4-40, lower is better
    uint8_t flash    = 0;        // linkfam CF_*
    bool    flip     = false;
    bool    mirror   = false;
    int8_t  bright   = 0;        // -2..2
    int8_t  contrast = 0;
    int8_t  sat      = 0;
    int8_t  exposure = 0;
    uint8_t wb       = 0;        // 0 auto, 1 sunny, 2 cloudy, 3 office, 4 home
    uint8_t effect   = 0;
    bool    levels   = true;     // Auto levels
    uint8_t gamma    = 4;        // campic::kGammas index, 4 is 1.0
};

// One request, as a SNAP carries it.
struct SnapReq {
    uint8_t size = 0;            // 0: the settings' size
    uint8_t quality = 0;         // 0: the settings' quality
    uint8_t flash = 0xFF;        // 0xFF: the settings' flash
    bool    mark = false;
    char    board[21] = {};
    char    when[17] = {};
    char    who[25] = {};
    char    comment[151] = {};
};

struct Pic {
    uint8_t* buf = nullptr;      // kHead bytes free, then the JPEG
    size_t   len = 0;            // the JPEG's length
    size_t   cap = 0;
    uint16_t w = 0, h = 0;
    char     err[64] = {};
    uint8_t  failCode = 0;       // linkfam CE_* when it failed
    // what it took, for the console
    uint32_t msUp = 0, msShot = 0, msWork = 0;
    uint32_t srcBytes = 0;
    uint8_t  settleFrames = 0;
    bool     marked = false, fixed = false, flashed = false;
    bool     stuck = false;       // a sensor that answered this boot and now does not
};

namespace cam {

constexpr size_t kHead = 16;                 // linkfam::kPictureHeader

// begin: the output buffer (PSRAM) and the flash pin held low. Once.
bool begin();
// snap: one picture into pic (pic.buf is the buffer begin made). False with
// pic.err and pic.failCode set.
bool snap(const SnapReq& r, const PicSettings& s, Pic& pic);
// sensor: the model the last bring-up found, "" before one.
const char* sensor();
// maxSize: the largest CS_* the sensor gives, 0 before a bring-up.
uint8_t maxSize();

}  // namespace cam
