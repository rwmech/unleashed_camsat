// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/store.h
// Module:       What the satellite keeps across a power cut
//
// Purpose:      In NVS: the pairings, up to five boards (1.2.0: each
//               board's MAC, its link key, never printed, its place in the
//               order, its name and what it wants delivered), the settings
//               the owner last sent, and the satellite's own name. In RTC memory, across deep sleep only:
//               the channel the board was last found on, so a satellite
//               that wakes to take a picture does not scan for it.
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

#include "cam.h"

// The board's SETTINGS as the satellite runs them (linkfam, 24 bytes on the
// wire; this is the same thing unpacked).
struct SatSettings {
    uint16_t tlMin     = 0;      // timelapse, minutes and seconds; both 0 off
    uint8_t  tlSec     = 0;
    bool     motion    = false;
    uint16_t holdoffS  = 60;
    bool     sleep     = false;  // deep sleep between pictures
    uint8_t  motionPin = 13;
    PicSettings pic;
};

namespace store {

// One board this satellite is paired with (1.2.0).
constexpr uint8_t kBoards = 5;
struct BoardRec {
    uint8_t mac[6];
    uint8_t key[16];
    uint8_t ord;               // the lowest is the owner
    uint8_t recv;              // RECV_* (linkfam), what it wants delivered
    char    name[17];
};

bool begin();
// loadBoards: the boards, their count. A satellite paired before 1.2.0 has
// one pairing under the old keys: it is read as the owner, wanting every
// picture, and written in the new form at the next save.
uint8_t loadBoards(BoardRec out[kBoards]);
bool saveBoards(const BoardRec* b, uint8_t n);
void forget();
// The settings the board last sent (defaults when it never has).
void loadSettings(SatSettings& s);
void saveSettings(const SatSettings& s);
// name: "camsat-" and the last two bytes of the MAC, or what NVS holds.
void name(char* out, size_t n);
// The channel the board was last heard on, kept across deep sleep. 0 unknown.
uint8_t lastChannel();
void setLastChannel(uint8_t ch);

}  // namespace store
