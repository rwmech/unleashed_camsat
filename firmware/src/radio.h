// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/radio.h
// Module:       The satellite's radio: ESP-NOW as the link engine's Io
//
// Purpose:      The link engine (core/link.h, fetched from the core) talks
//               to the world through an Io. This one is ESP-NOW on a station
//               that never joins an access point, so it may move channel to
//               follow its board.
//
//               Received frames go into a ring from the Wi-Fi task's receive
//               callback, one copy and nothing else; the engine takes them
//               from the satellite's own loop. One frame at a time goes to
//               the MAC: the engine asks idle() before the next.
//
//               The rate, as the bench measured it (release-prep, 2026-09-26):
//               11g 24 Mbps to the board, six to ten times 1 Mbps and less
//               air for the board's own callers; 1 Mbps after three MAC
//               failures in a row, and 24 Mbps again after 30 s clean.
//               Broadcasts (discovery, pairing) stay at 1 Mbps, the range.
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

#include "link.h"

class SatRadio : public ulink::Io {
public:
    // start: Wi-Fi as a bare station on channel ch, ESP-NOW, the ring.
    bool start(uint8_t ch);

    bool send(const ulink::Mac* to, const uint8_t* f, size_t n) override;
    bool idle() override;
    size_t recv(uint8_t* out, size_t cap, ulink::Mac& from, int8_t& rssi) override;
    uint32_t millis() override;
    uint32_t micros() override;
    void random(uint8_t* out, size_t n) override;
    uint8_t channel() override;
    void setChannel(uint8_t ch) override;
    bool addPeer(const ulink::Mac& m) override;
    void delPeer(const ulink::Mac& m) override;
    uint32_t heapFree() override;
    uint8_t macFailStreak() override;

    // For the console and STATUS.
    uint32_t ringDrops() const;
    uint32_t sendFails() const;
    bool     slowRate() const;
    void     mac(uint8_t out[6]) const;
};

// rngMbed: esp_fill_random as mbedTLS's RNG, for pairing's key pair.
int rngMbed(void* ctx, unsigned char* out, size_t n);
