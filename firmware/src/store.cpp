// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/store.cpp
// Module:       What the satellite keeps across a power cut
//
// Purpose:      See store.h.
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
#include "store.h"

#include <cstdio>
#include <cstring>

#include "esp_attr.h"
#include "esp_mac.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {

const char* kNs = "camsat";
nvs_handle_t g_nvs = 0;

// Across deep sleep, not a power cut.
RTC_DATA_ATTR uint8_t g_rtcChan = 0;
RTC_DATA_ATTR uint32_t g_rtcMagic = 0;
constexpr uint32_t kMagic = 0xCA45A7u;

// The settings in NVS, versioned so a later layout never reads an old one.
struct Saved {
    uint8_t     ver = 1;
    SatSettings s;
};

}  // namespace

namespace store {

bool begin() {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e != ESP_OK) return false;
    return nvs_open(kNs, NVS_READWRITE, &g_nvs) == ESP_OK;
}

// The boards in NVS, versioned like the settings.
struct Boards {
    uint8_t  ver = 2;
    uint8_t  n = 0;
    store::BoardRec b[store::kBoards] = {};
};

uint8_t loadBoards(BoardRec out[kBoards]) {
    if (!g_nvs) return 0;
    Boards v;
    size_t n = sizeof(v);
    if (nvs_get_blob(g_nvs, "boards", &v, &n) == ESP_OK && n == sizeof(v) && v.ver == 2 && v.n <= kBoards) {
        memcpy(out, v.b, sizeof(v.b));
        const uint8_t k = v.n;
        memset(&v, 0, sizeof(v));
        return k;
    }
    // Before 1.2.0: one board, under "host" and "key".
    BoardRec& r = out[0];
    memset(&r, 0, sizeof(r));
    n = 6;
    if (nvs_get_blob(g_nvs, "host", r.mac, &n) != ESP_OK || n != 6) return 0;
    n = 16;
    if (nvs_get_blob(g_nvs, "key", r.key, &n) != ESP_OK || n != 16) { memset(&r, 0, sizeof(r)); return 0; }
    r.ord = 0;
    r.recv = 3;                                        // RECV_ALL: it had every picture
    return 1;
}

bool saveBoards(const BoardRec* b, uint8_t n) {
    if (!g_nvs || n > kBoards) return false;
    Boards v;
    v.n = n;
    memcpy(v.b, b, sizeof(BoardRec) * n);
    const bool ok = nvs_set_blob(g_nvs, "boards", &v, sizeof(v)) == ESP_OK && nvs_commit(g_nvs) == ESP_OK;
    memset(&v, 0, sizeof(v));
    // The one-board keys go once the new form is written, and only then: a
    // full partition must not lose the pairing (code review).
    if (!ok) return false;
    nvs_erase_key(g_nvs, "host");
    nvs_erase_key(g_nvs, "key");
    return nvs_commit(g_nvs) == ESP_OK;
}

void forget() {
    if (!g_nvs) return;
    nvs_erase_key(g_nvs, "boards");
    nvs_erase_key(g_nvs, "host");
    nvs_erase_key(g_nvs, "key");
    nvs_erase_key(g_nvs, "set");
    nvs_commit(g_nvs);
    g_rtcMagic = 0;
}

void loadSettings(SatSettings& s) {
    s = SatSettings();
    Saved v;
    size_t n = sizeof(v);
    if (g_nvs && nvs_get_blob(g_nvs, "set", &v, &n) == ESP_OK && n == sizeof(v) && v.ver == 1) s = v.s;
}

void saveSettings(const SatSettings& s) {
    if (!g_nvs) return;
    Saved v;
    v.s = s;
    nvs_set_blob(g_nvs, "set", &v, sizeof(v));
    nvs_commit(g_nvs);
}

void name(char* out, size_t n) {
    size_t len = n;
    if (g_nvs && nvs_get_str(g_nvs, "name", out, &len) == ESP_OK && out[0]) return;
    uint8_t m[6];
    esp_read_mac(m, ESP_MAC_WIFI_STA);
    snprintf(out, n, "camsat-%02x%02x", m[4], m[5]);
}

uint8_t lastChannel() { return g_rtcMagic == kMagic ? g_rtcChan : 0; }

void setLastChannel(uint8_t ch) {
    g_rtcChan = ch;
    g_rtcMagic = kMagic;
}

}  // namespace store
