// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/radio.cpp
// Module:       The satellite's radio: ESP-NOW as the link engine's Io
//
// Purpose:      See radio.h.
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
#include "radio.h"

#include <atomic>
#include <cstring>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"

namespace {

const char* TAG = "radio";

// One received frame. 16 slots: a satellite hears acknowledgements and the
// odd request, never a bulk message, so this is ample.
struct Slot {
    uint8_t mac[6];
    int8_t  rssi;
    uint8_t len;
    uint8_t data[ESP_NOW_MAX_DATA_LEN];
};
constexpr uint8_t kSlots = 16;
Slot g_ring[kSlots];
std::atomic<uint16_t> g_head{ 0 };           // the Wi-Fi task's
std::atomic<uint16_t> g_tail{ 0 };           // the loop's
std::atomic<bool>     g_busy{ false };
std::atomic<uint32_t> g_drops{ 0 };
std::atomic<uint32_t> g_fails{ 0 };
std::atomic<uint8_t>  g_streak{ 0 };         // MAC failures in a row
uint32_t              g_sentAt = 0;
uint8_t               g_chan = 1;

// The rate to the board.
constexpr uint8_t  kSlowAfter  = 3;          // failures in a row
constexpr uint32_t kFastAgain  = 30000;      // ms clean at 1 Mbps
bool     g_slow = false;
uint32_t g_slowSince = 0;
uint8_t  g_rateMac[6] = {};
bool     g_rateSet = false;

const uint8_t kBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!info || !data || len <= 0 || len > ESP_NOW_MAX_DATA_LEN) return;
    const uint16_t h = g_head.load(std::memory_order_relaxed);
    const uint16_t t = g_tail.load(std::memory_order_acquire);
    if (static_cast<uint16_t>(h - t) >= kSlots) { g_drops.fetch_add(1); return; }
    Slot& s = g_ring[h % kSlots];
    memcpy(s.mac, info->src_addr, 6);
    s.rssi = info->rx_ctrl ? static_cast<int8_t>(info->rx_ctrl->rssi) : 0;
    s.len = static_cast<uint8_t>(len);
    memcpy(s.data, data, static_cast<size_t>(len));
    g_head.store(static_cast<uint16_t>(h + 1), std::memory_order_release);
}

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    const bool unicast = mac && memcmp(mac, kBroadcast, 6) != 0;
    if (status != ESP_NOW_SEND_SUCCESS) {
        g_fails.fetch_add(1);
        if (unicast && g_streak.load() < 255) g_streak.fetch_add(1);
    } else if (unicast) {
        g_streak.store(0);
    }
    g_busy.store(false);
}

void applyRate(const uint8_t* mac, bool slow) {
    esp_now_rate_config_t rc = {};
    rc.phymode = slow ? WIFI_PHY_MODE_11B : WIFI_PHY_MODE_11G;
    rc.rate    = slow ? WIFI_PHY_RATE_1M_L : WIFI_PHY_RATE_24M;
    if (esp_now_set_peer_rate_config(mac, &rc) != ESP_OK) ESP_LOGW(TAG, "rate not set");
}

// rateCheck: before each unicast, fall back or come back as the streak says.
void rateCheck(const uint8_t* mac) {
    const uint32_t now = nowMs();
    if (!g_rateSet || memcmp(g_rateMac, mac, 6)) {
        memcpy(g_rateMac, mac, 6);
        g_rateSet = true;
        g_slow = false;
        g_streak.store(0);
        applyRate(mac, false);
        return;
    }
    if (!g_slow && g_streak.load() >= kSlowAfter) {
        g_slow = true;
        g_slowSince = now;
        applyRate(mac, true);
        ESP_LOGI(TAG, "%u failures in a row: 1 Mbps", static_cast<unsigned>(g_streak.load()));
    } else if (g_slow && g_streak.load() == 0 && now - g_slowSince >= kFastAgain) {
        g_slow = false;
        applyRate(mac, false);
        ESP_LOGI(TAG, "clean for 30 s: 24 Mbps again");
    } else if (g_slow && g_streak.load()) {
        g_slowSince = now;                       // not clean yet
    }
}

bool ensurePeer(const uint8_t* mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 0;                               // whatever channel the radio is on
    p.ifidx = WIFI_IF_STA;
    p.encrypt = false;                           // the link seals for itself
    const esp_err_t e = esp_now_add_peer(&p);
    if (e == ESP_OK && memcmp(mac, kBroadcast, 6)) g_rateSet = false;   // a new entry has the default rate
    return e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST;
}

}  // namespace

bool SatRadio::start(uint8_t ch) {
    if (ch < 1 || ch > 13) ch = 1;
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) return false;
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    if (esp_wifi_start() != ESP_OK) return false;
    esp_wifi_set_ps(WIFI_PS_NONE);
    setChannel(ch);
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onRecv);
    esp_now_register_send_cb(onSent);
    ensurePeer(kBroadcast);
    return true;
}

bool SatRadio::send(const ulink::Mac* to, const uint8_t* f, size_t n) {
    if (n > ESP_NOW_MAX_DATA_LEN) return false;
    const uint8_t* dst = to ? to->b : kBroadcast;
    if (!ensurePeer(dst)) return false;
    if (to) rateCheck(dst);
    g_busy.store(true);
    g_sentAt = nowMs();
    if (esp_now_send(dst, f, n) != ESP_OK) {
        g_busy.store(false);
        return false;
    }
    return true;
}

bool SatRadio::idle() {
    if (!g_busy.load()) return true;
    // A callback that never came must not stop the link for ever.
    if (nowMs() - g_sentAt > 100) {
        g_busy.store(false);
        g_fails.fetch_add(1);
        return true;
    }
    return false;
}

size_t SatRadio::recv(uint8_t* out, size_t cap, ulink::Mac& from, int8_t& rssi) {
    const uint16_t t = g_tail.load(std::memory_order_relaxed);
    const uint16_t h = g_head.load(std::memory_order_acquire);
    if (t == h) return 0;
    const Slot& s = g_ring[t % kSlots];
    const size_t n = s.len < cap ? s.len : cap;
    memcpy(out, s.data, n);
    memcpy(from.b, s.mac, 6);
    rssi = s.rssi;
    g_tail.store(static_cast<uint16_t>(t + 1), std::memory_order_release);
    return n;
}

uint32_t SatRadio::millis() { return nowMs(); }
uint32_t SatRadio::micros() { return static_cast<uint32_t>(esp_timer_get_time()); }

void SatRadio::random(uint8_t* out, size_t n) { esp_fill_random(out, n); }

uint8_t SatRadio::channel() { return g_chan; }

void SatRadio::setChannel(uint8_t ch) {
    if (ch < 1 || ch > 13) return;
    if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK) g_chan = ch;
}

bool SatRadio::addPeer(const ulink::Mac& m) { return ensurePeer(m.b); }

void SatRadio::delPeer(const ulink::Mac& m) {
    if (esp_now_is_peer_exist(m.b)) esp_now_del_peer(m.b);
}

uint32_t SatRadio::heapFree() { return static_cast<uint32_t>(esp_get_free_heap_size()); }

uint32_t SatRadio::ringDrops() const { return g_drops.load(); }
uint32_t SatRadio::sendFails() const { return g_fails.load(); }
bool     SatRadio::slowRate() const { return g_slow; }
void     SatRadio::mac(uint8_t out[6]) const { esp_read_mac(out, ESP_MAC_WIFI_STA); }

int rngMbed(void*, unsigned char* out, size_t n) {
    esp_fill_random(out, n);
    return 0;
}
