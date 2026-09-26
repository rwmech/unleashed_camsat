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

bool loadPairing(uint8_t mac[6], uint8_t key[16]) {
    size_t n = 6;
    if (!g_nvs || nvs_get_blob(g_nvs, "host", mac, &n) != ESP_OK || n != 6) return false;
    n = 16;
    return nvs_get_blob(g_nvs, "key", key, &n) == ESP_OK && n == 16;
}

bool savePairing(const uint8_t mac[6], const uint8_t key[16]) {
    if (!g_nvs) return false;
    return nvs_set_blob(g_nvs, "host", mac, 6) == ESP_OK && nvs_set_blob(g_nvs, "key", key, 16) == ESP_OK &&
           nvs_commit(g_nvs) == ESP_OK;
}

void forget() {
    if (!g_nvs) return;
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
