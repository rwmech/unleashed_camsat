// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/store.h
// Module:       What the satellite keeps across a power cut
//
// Purpose:      In NVS: the pairing (the board's MAC and the link key,
//               never printed), the settings the board last sent, and the
//               satellite's own name. In RTC memory, across deep sleep only:
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

bool begin();
// The pairing. load: false when there is none.
bool loadPairing(uint8_t mac[6], uint8_t key[16]);
bool savePairing(const uint8_t mac[6], const uint8_t key[16]);
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
