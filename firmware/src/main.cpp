// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/main.cpp
// Module:       The satellite
//
// Purpose:      Pairs with a µnleashed board, keeps the link up, and takes a
//               picture whenever the board sends SNAP: for a caller, for the
//               board's timelapse, or because this satellite's motion sensor
//               or its own timer asked (EVENT; the board answers with SNAP,
//               so the board names, dates and limits every picture).
//
//               Two tasks. The link task (core 1) owns the engine: every
//               engine call is its. The camera task (core 0, below the
//               radio) takes and works the picture, which is seconds of CPU,
//               and hands back a finished JPEG the link task then sends.
//
//               The lights: the red LED on the back says what the link is
//               doing (a double blink every second: pairing; a slow blink:
//               looking for the board; a short blip every 3 s: up; lit:
//               taking a picture; fast: forgetting). The white flash LED is
//               held low from the first instruction and lit only for a
//               picture that asks for it.
//
//               Forgetting the pairing: hold IO0 (the MB board's button) for
//               5 s any time after boot. Held at power-up it is the ROM's
//               download mode instead, and this firmware never runs.
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
#include <atomic>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board.h"
#include "cam.h"
#include "link.h"
#include "linkfam.h"
#include "radio.h"
#include "store.h"

using namespace ulink;
using namespace linkfam;

namespace {

const char* TAG = "camsat";

// ---------------------------------------------------------------------------
// The flash LED, held low before anything else runs: its transistor lights
// on a floating pin (the core's ESPCAM profile, BBS_PINS_HOLD_LOW).
// ---------------------------------------------------------------------------
__attribute__((constructor)) void holdFlashLow() {
    const gpio_num_t g = static_cast<gpio_num_t>(PIN_FLASH);
    gpio_set_pull_mode(g, GPIO_FLOATING);
    gpio_set_level(g, 0);
    gpio_set_direction(g, GPIO_MODE_OUTPUT);
}

uint32_t ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// ---------------------------------------------------------------------------
// The red LED
// ---------------------------------------------------------------------------
enum class Led : uint8_t { Off, Pairing, Search, Up, Busy, Forget };
Led g_led = Led::Off;

void ledSet(bool on) { gpio_set_level(static_cast<gpio_num_t>(PIN_LED), on ? 0 : 1); }

void ledTick(uint32_t now) {
    bool on = false;
    switch (g_led) {
        case Led::Off:     on = false; break;
        case Led::Pairing: { uint32_t t = now % 1000; on = t < 80 || (t >= 200 && t < 280); } break;
        case Led::Search:  on = (now % 1000) < 500; break;
        case Led::Up:      on = (now % 3000) < 40; break;
        case Led::Busy:    on = true; break;
        case Led::Forget:  on = (now % 100) < 50; break;
    }
    ledSet(on);
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
SatRadio    g_radio;
Engine*     g_eng = nullptr;
SatSettings g_set;
char        g_name[17] = "";
bool        g_paired = false;
uint32_t    g_pairUntil = 0;              // pairing mode ends (5 minutes)
uint32_t    g_unixAt = 0, g_unixMs = 0;   // the board's clock, as PONG last said
uint8_t     g_lastErr = 0;
uint32_t    g_statusAt = 0;
uint32_t    g_bootMs = 0;

// The one picture in hand. The link task owns every field but the phase,
// which the camera task moves from Taking to Ready or Failed.
enum : uint8_t { P_IDLE, P_TAKING, P_READY, P_FAILED, P_SENDING };
struct Job {
    std::atomic<uint8_t> ph{ P_IDLE };
    uint16_t sess = 0;
    uint16_t req = 0;
    uint8_t  reason = 0;
    bool     ours = false;                // a session this satellite opened (EVENT)
    SnapReq  r;
    Pic      pic;
};
Job g_job;
TaskHandle_t g_camTask = nullptr;

// Waking and sleeping
uint8_t  g_wakeKind = 0;                  // CEV_* the wake is for, 0 none
bool     g_wakeSent = false;
uint32_t g_quietSince = 0;                // nothing to do since, for sleep
uint32_t g_motionAt = 0;                  // the last motion EVENT
bool     g_motionWas = false;
uint32_t g_forgetFrom = 0;

uint32_t unixNow() {
    return g_unixAt ? g_unixAt + (ms() - g_unixMs) / 1000 : 0;
}

void put16(uint8_t* p, uint16_t v) { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
void put32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i)); }
uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }

// text: a NUL-padded field of the wire into a C string.
void text(char* out, size_t cap, const uint8_t* p, size_t n) {
    size_t k = 0;
    while (k < n && k + 1 < cap && p[k]) { out[k] = static_cast<char>(p[k]); ++k; }
    out[k] = '\0';
}

// ---------------------------------------------------------------------------
// The camera task
// ---------------------------------------------------------------------------
void camTask(void*) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (g_job.ph.load() != P_TAKING) continue;
        const bool ok = cam::snap(g_job.r, g_set.pic, g_job.pic);
        // The camera had GPIO 0 as its clock: the button's again.
        gpio_reset_pin(static_cast<gpio_num_t>(PIN_FORGET));
        gpio_set_direction(static_cast<gpio_num_t>(PIN_FORGET), GPIO_MODE_INPUT);
        gpio_set_pull_mode(static_cast<gpio_num_t>(PIN_FORGET), GPIO_PULLUP_ONLY);
        const Pic& p = g_job.pic;
        if (ok)
            ESP_LOGI(TAG, "picture %ux%u, %u bytes (sensor %u), up %u ms, shot %u ms, work %u ms, "
                          "%u settle frames%s%s%s",
                     p.w, p.h, static_cast<unsigned>(p.len), static_cast<unsigned>(p.srcBytes),
                     static_cast<unsigned>(p.msUp), static_cast<unsigned>(p.msShot),
                     static_cast<unsigned>(p.msWork), p.settleFrames, p.marked ? ", marked" : "",
                     p.fixed ? ", corrected" : "", p.flashed ? ", flash" : "");
        else
            ESP_LOGW(TAG, "no picture: %s", p.err);
        g_job.ph.store(ok ? P_READY : P_FAILED);
    }
}

// ---------------------------------------------------------------------------
// Messages to the board
// ---------------------------------------------------------------------------
void sendFail(uint16_t sess, uint16_t req, uint8_t code) {
    uint8_t b[3];
    put16(b, req);
    b[2] = code;
    g_lastErr = code;
    g_eng->send(0, sess, FAM_CAMERA, CAM_SNAP_FAIL, b, sizeof(b));
}

void sendStatus() {
    const uint16_t sess = g_eng->openSession(0, FAM_CAMERA);
    if (!sess) return;
    uint8_t b[28] = {};
    b[0] = cam::sensor()[0] ? 1 : 0;
    b[1] = cam::maxSize();
    put32(b + 2, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    put32(b + 6, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    put32(b + 10, ms() / 1000);
    b[14] = g_lastErr;
    snprintf(reinterpret_cast<char*>(b + 16), 12, "%s", cam::sensor());
    g_eng->send(0, sess, FAM_CAMERA, CAM_STATUS, b, sizeof(b));
    g_eng->closeAfter(0, sess);
}

void sendSettingsOk(uint16_t sess) {
    uint8_t b[24] = {};
    const SatSettings& s = g_set;
    put16(b, s.tlMin);
    b[2] = s.tlSec;
    b[3] = s.motion;
    put16(b + 4, s.holdoffS);
    b[6] = s.pic.size;
    b[7] = s.pic.quality;
    b[8] = s.pic.flash;
    b[9] = s.sleep;
    b[10] = s.pic.flip;
    b[11] = s.pic.mirror;
    b[12] = static_cast<uint8_t>(s.pic.bright);
    b[13] = static_cast<uint8_t>(s.pic.contrast);
    b[14] = static_cast<uint8_t>(s.pic.sat);
    b[15] = static_cast<uint8_t>(s.pic.exposure);
    b[16] = s.pic.wb;
    b[17] = s.pic.effect;
    b[18] = s.pic.levels;
    b[19] = s.pic.gamma;
    b[20] = s.motion ? s.motionPin : 0xFF;
    g_eng->send(0, sess, FAM_CAMERA, CAM_SETTINGS_OK, b, sizeof(b));
}

// event: ask the board for a picture (motion or this satellite's timer).
bool sendEvent(uint8_t kind) {
    if (g_job.ph.load() != P_IDLE) return false;
    const uint16_t sess = g_eng->openSession(0, FAM_CAMERA);
    if (!sess) return false;
    uint8_t b[5];
    b[0] = kind;
    put32(b + 1, unixNow());
    if (g_eng->send(0, sess, FAM_CAMERA, CAM_EVENT, b, sizeof(b)) != 1) {
        g_eng->closeSession(0, sess);
        return false;
    }
    ESP_LOGI(TAG, "asked the board for a %s picture", kind == CEV_MOTION ? "motion" : "timelapse");
    return true;
}

// ---------------------------------------------------------------------------
// Messages from the board
// ---------------------------------------------------------------------------
void clampSettings(SatSettings& s) {
    if (s.pic.size < CS_QQVGA || s.pic.size > CS_UXGA) s.pic.size = CS_XGA;
    if (s.pic.quality < 4 || s.pic.quality > 40) s.pic.quality = 10;
    if (s.pic.flash > CF_AUTO) s.pic.flash = CF_OFF;
    if (s.pic.gamma > 8) s.pic.gamma = 4;
    if (s.pic.wb > 4) s.pic.wb = 0;
    if (s.pic.effect > 6) s.pic.effect = 0;
    auto c2 = [](int8_t& v) { if (v < -2) v = -2; if (v > 2) v = 2; };
    c2(s.pic.bright); c2(s.pic.contrast); c2(s.pic.sat); c2(s.pic.exposure);
    if (s.tlSec > 59) s.tlSec = 59;
    if (!motionPinOk(s.motionPin)) { s.motionPin = PIN_MOTION_DEF; }
}

void motionPinSetup() {
    if (!g_set.motion) return;
    const gpio_num_t g = static_cast<gpio_num_t>(g_set.motionPin);
    gpio_reset_pin(g);
    gpio_set_direction(g, GPIO_MODE_INPUT);
    gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);
}

void onSettings(uint16_t sess, const uint8_t* p, size_t n) {
    if (n < 21) return;
    SatSettings s = g_set;
    s.tlMin = get16(p);
    s.tlSec = p[2];
    s.motion = p[3] != 0;
    s.holdoffS = get16(p + 4);
    s.pic.size = p[6];
    s.pic.quality = p[7];
    s.pic.flash = p[8];
    s.sleep = p[9] != 0;
    s.pic.flip = p[10] != 0;
    s.pic.mirror = p[11] != 0;
    s.pic.bright = static_cast<int8_t>(p[12]);
    s.pic.contrast = static_cast<int8_t>(p[13]);
    s.pic.sat = static_cast<int8_t>(p[14]);
    s.pic.exposure = static_cast<int8_t>(p[15]);
    s.pic.wb = p[16];
    s.pic.effect = p[17];
    s.pic.levels = p[18] != 0;
    s.pic.gamma = p[19];
    if (p[20] != 0xFF) s.motionPin = p[20];
    clampSettings(s);
    const bool changed = memcmp(&s, &g_set, sizeof(s)) != 0;
    g_set = s;
    if (changed) {
        store::saveSettings(g_set);
        motionPinSetup();
        ESP_LOGI(TAG, "settings: %s q%u flash %u, timelapse %u:%02u, motion %s (pin %u, %u s), %s",
                 g_set.pic.size == CS_UXGA ? "uxga" : g_set.pic.size == CS_SXGA ? "sxga" : g_set.pic.size == CS_XGA ? "xga"
                 : g_set.pic.size == CS_SVGA ? "svga" : g_set.pic.size == CS_VGA ? "vga" : "small",
                 g_set.pic.quality, g_set.pic.flash, g_set.tlMin, g_set.tlSec, g_set.motion ? "on" : "off",
                 g_set.motionPin, g_set.holdoffS, g_set.sleep ? "deep sleep between pictures" : "awake");
    }
    sendSettingsOk(sess);
}

void onSnap(uint16_t sess, const uint8_t* p, size_t n) {
    if (n < 6) return;
    const uint16_t req = get16(p);
    if (g_job.ph.load() != P_IDLE) { sendFail(sess, req, CE_BUSY); return; }
    Job& j = g_job;
    j.r = SnapReq();
    j.r.size = p[2];
    j.r.quality = p[3];
    j.r.flash = p[4];
    j.reason = p[5];
    if (n >= 67) {
        j.r.mark = p[6] != 0;
        text(j.r.board, sizeof(j.r.board), p + 7, 20);
        text(j.r.when, sizeof(j.r.when), p + 27, 16);
        text(j.r.who, sizeof(j.r.who), p + 43, 24);
        text(j.r.comment, sizeof(j.r.comment), p + 67, n - 67);
    }
    j.sess = sess;
    j.req = req;
    j.ours = sess & 0x8000;
    j.ph.store(P_TAKING);
    g_led = Led::Busy;
    xTaskNotifyGive(g_camTask);
    ESP_LOGI(TAG, "SNAP %u for %s", static_cast<unsigned>(req), j.r.who[0] ? j.r.who : "the board");
}

bool evMessage(void*, uint8_t, uint16_t sess, uint8_t family, uint8_t type, const uint8_t* p, size_t n) {
    if (family != FAM_CAMERA) return true;
    if (type == CAM_SNAP) onSnap(sess, p, n);
    else if (type == CAM_SETTINGS) onSettings(sess, p, n);
    g_quietSince = ms();
    return true;
}

void jobDone() {
    if (g_job.ours) g_eng->closeAfter(0, g_job.sess);
    g_job.ph.store(P_IDLE);
    g_led = g_eng->hostUp() ? Led::Up : Led::Search;
    g_quietSince = ms();
}

uint32_t g_sendAt = 0;

void evBulkSent(void*, uint8_t, uint16_t sess, uint8_t, bool ok) {
    if (g_job.ph.load() != P_SENDING || sess != g_job.sess) return;
    const uint32_t took = ms() - g_sendAt;
    const PeerStats& st = g_eng->peerStats(0);
    ESP_LOGI(TAG, "picture %u %s: %u bytes in %u ms (%u KB/s); link tx %u, retries %u, drops %u; "
                  "radio fails %u%s",
             static_cast<unsigned>(g_job.req), ok ? "taken by the board" : "not delivered",
             static_cast<unsigned>(g_job.pic.len), static_cast<unsigned>(took),
             static_cast<unsigned>(took ? g_job.pic.len / took : 0), static_cast<unsigned>(st.tx),
             static_cast<unsigned>(st.retries), static_cast<unsigned>(st.drops),
             static_cast<unsigned>(g_radio.sendFails()), g_radio.slowRate() ? ", at 1 Mbps" : "");
    if (!ok) g_lastErr = CE_CAPTURE;
    jobDone();
}

void evReset(void*, uint8_t, uint16_t sess, uint8_t, uint8_t reason) {
    if (sess != g_job.sess) return;
    const uint8_t ph = g_job.ph.load();
    if (ph == P_SENDING || ph == P_READY || ph == P_FAILED) {
        ESP_LOGW(TAG, "the board ended the picture's session (%u)", reason);
        g_job.ph.store(P_IDLE);
        g_led = g_eng->hostUp() ? Led::Up : Led::Search;
    }
}

void evPeerState(void*, uint8_t, bool up) {
    ESP_LOGI(TAG, "link %s", up ? "up" : "down");
    if (g_job.ph.load() == P_IDLE) g_led = up ? Led::Up : Led::Search;
    if (up) {
        store::setLastChannel(g_radio.channel());
        g_statusAt = 0;                          // STATUS now
        g_wakeSent = false;
    }
}

void evPaired(void*, uint8_t, const PairInfo& who) {
    store::savePairing(who.mac.b, who.key);
    g_paired = true;
    g_pairUntil = 0;
    ESP_LOGI(TAG, "paired with %02x:%02x:%02x:%02x:%02x:%02x, code %04u", who.mac.b[0], who.mac.b[1],
             who.mac.b[2], who.mac.b[3], who.mac.b[4], who.mac.b[5], static_cast<unsigned>(who.code));
    g_led = Led::Search;
}

void evChannel(void*, uint8_t ch) {
    store::setLastChannel(ch);
    ESP_LOGI(TAG, "on channel %u", ch);
}

void evClock(void*, uint32_t unix) {
    g_unixAt = unix;
    g_unixMs = ms();
}

// ---------------------------------------------------------------------------
// The loop's other duties
// ---------------------------------------------------------------------------
// sendPicture: a finished picture goes as one bulk message: the header, then
// the JPEG, from the buffer the camera wrote.
void sendPicture() {
    Job& j = g_job;
    uint8_t* h = j.pic.buf;
    memset(h, 0, cam::kHead);
    put16(h, j.req);
    h[2] = j.reason;
    put16(h + 4, j.pic.w);
    put16(h + 6, j.pic.h);
    put32(h + 8, unixNow());
    const int r = g_eng->sendBulk(0, j.sess, FAM_CAMERA, CAM_PICTURE, h,
                                  static_cast<uint32_t>(cam::kHead + j.pic.len));
    if (r == 1) { j.ph.store(P_SENDING); g_sendAt = ms(); return; }
    if (r < 0) {
        ESP_LOGW(TAG, "the picture could not be queued");
        sendFail(j.sess, j.req, CE_NOMEM);
        jobDone();
    }
    // 0: not now, again next pass
}

void forgetWatch(uint32_t now) {
    if (g_job.ph.load() == P_TAKING) { g_forgetFrom = 0; return; }   // GPIO 0 is the camera's clock
    const bool down = gpio_get_level(static_cast<gpio_num_t>(PIN_FORGET)) == 0;
    if (!down) {
        if (g_forgetFrom && g_led == Led::Forget) g_led = Led::Search;
        g_forgetFrom = 0;
        return;
    }
    if (!g_forgetFrom) g_forgetFrom = now ? now : 1;
    if (now - g_forgetFrom >= 1000) g_led = Led::Forget;
    if (now - g_forgetFrom >= 5000) {
        ESP_LOGW(TAG, "IO0 held 5 s: forgetting the pairing and starting again");
        store::forget();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
}

void motionWatch(uint32_t now) {
    if (!g_set.motion || !g_eng->hostUp()) return;
    const bool high = gpio_get_level(static_cast<gpio_num_t>(g_set.motionPin)) != 0;
    const bool rise = high && !g_motionWas;
    g_motionWas = high;
    if (!rise) return;
    if (g_motionAt && now - g_motionAt < static_cast<uint32_t>(g_set.holdoffS) * 1000u) return;
    if (sendEvent(CEV_MOTION)) g_motionAt = now;
}

// sleepWatch: with deep sleep on, sleep once nothing is going on: a picture
// sent, or a wake the board never answered. Awake for the first minute
// after a power-up, so the board can send settings and a sysop can pair.
void sleepWatch(uint32_t now) {
    if (!g_set.sleep || !g_paired || g_job.ph.load() != P_IDLE) return;
    const uint32_t tl = static_cast<uint32_t>(g_set.tlMin) * 60u + g_set.tlSec;
    const bool motion = g_set.motion && motionPinOk(g_set.motionPin);
    if (!tl && !motion) return;                          // nothing would wake it
    const bool cold = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED;
    if (cold && now - g_bootMs < 60000) return;
    const uint32_t quietFor = g_wakeKind && !g_wakeSent ? 10000u : 2000u;
    if (now - g_quietSince < quietFor) return;
    if (g_wakeKind && !g_wakeSent && now - g_bootMs < 15000) return;   // still finding the board
    ESP_LOGI(TAG, "sleeping%s%s", tl ? ", timer" : "", motion ? ", motion" : "");
    if (tl) esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(tl) * 1000000ull);
    if (motion) esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(g_set.motionPin), 1);
    ledSet(false);
    esp_deep_sleep_start();
}

void linkTask(void*) {
    for (;;) {
        const uint32_t now = ms();
        for (int i = 0; i < 4; ++i) g_eng->poll();
        if (g_eng->pairComputeWanted()) g_eng->pairCompute();
        const uint8_t ph = g_job.ph.load();
#ifdef CAMSAT_SELFTEST
        // The bench's picture, taken with no board: said and dropped.
        if (!g_job.sess && (ph == P_READY || ph == P_FAILED)) {
            ESP_LOGI(TAG, "selftest: %s, %u bytes", ph == P_READY ? "picture" : g_job.pic.err,
                     static_cast<unsigned>(g_job.pic.len));
            if (ph == P_READY) {
                // The first bytes, to see the comment went in behind SOI.
                const uint8_t* b = g_job.pic.buf + cam::kHead;
                ESP_LOGI(TAG, "selftest: %02x %02x %02x %02x len %u: %.60s", b[0], b[1], b[2], b[3],
                         static_cast<unsigned>(b[4] << 8 | b[5]), b + 6);
            }
            g_job.ph.store(P_IDLE);
            g_led = Led::Pairing;
        }
        else
#endif
        if (ph == P_READY) sendPicture();
        else if (ph == P_FAILED) { sendFail(g_job.sess, g_job.req, g_job.pic.failCode); jobDone(); }
        if (g_eng->hostUp()) {
            if (!g_statusAt || now - g_statusAt >= 60000) { g_statusAt = now; sendStatus(); }
            if (g_wakeKind && !g_wakeSent) g_wakeSent = sendEvent(g_wakeKind);
        }
        if (g_pairUntil && now > g_pairUntil) {
            g_eng->stopPairing();
            g_pairUntil = 0;
            g_led = Led::Off;
            ESP_LOGW(TAG, "no board answered in 5 minutes; reset the satellite to pair again");
        }
        forgetWatch(now);
        motionWatch(now);
        sleepWatch(now);
        ledTick(now);
        vTaskDelay(1);
    }
}

}  // namespace

extern "C" void app_main(void) {
    g_bootMs = ms();
    gpio_reset_pin(static_cast<gpio_num_t>(PIN_LED));
    gpio_set_direction(static_cast<gpio_num_t>(PIN_LED), GPIO_MODE_OUTPUT);
    ledSet(false);
    gpio_reset_pin(static_cast<gpio_num_t>(PIN_FORGET));
    gpio_set_direction(static_cast<gpio_num_t>(PIN_FORGET), GPIO_MODE_INPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(PIN_FORGET), GPIO_PULLUP_ONLY);

    store::begin();
    store::loadSettings(g_set);
    clampSettings(g_set);
    store::name(g_name, sizeof(g_name));
    motionPinSetup();
    if (!cam::begin()) ESP_LOGE(TAG, "no PSRAM for the picture buffer");

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: g_wakeKind = CEV_TIMELAPSE; break;
        case ESP_SLEEP_WAKEUP_EXT0:  g_wakeKind = CEV_MOTION; g_motionWas = true; break;
        default: break;
    }

    const uint8_t ch = store::lastChannel();
    if (!g_radio.start(ch ? ch : 1)) {
        ESP_LOGE(TAG, "the radio would not start");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    uint8_t mac[6];
    g_radio.mac(mac);
    ESP_LOGI(TAG, "µnleashed camsat %s, %s, %02x:%02x:%02x:%02x:%02x:%02x", CAMSAT_VERSION, g_name, mac[0],
             mac[1], mac[2], mac[3], mac[4], mac[5]);

    Events ev;
    ev.message = evMessage;
    ev.bulkSent = evBulkSent;
    ev.reset = evReset;
    ev.peerState = evPeerState;
    ev.paired = evPaired;
    ev.channel = evChannel;
    ev.clock = evClock;
    static uint8_t win[4 * kPayloadMax];            // a satellite receives no bulk
    g_eng = new Engine(Role::Peer, g_radio, ev, 4, win);
    if (!g_eng->ok()) {
        ESP_LOGE(TAG, "the link engine could not start");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    g_eng->setIdentity(KIND_CAMSAT, CAMSAT_VERSION, 1u << FAM_CAMERA);

    uint8_t key[16];
    Mac host;
    if (store::loadPairing(host.b, key)) {
        g_eng->addPeer(host, key, KIND_UNKNOWN);
        g_paired = true;
        g_led = Led::Search;
        ESP_LOGI(TAG, "paired with %02x:%02x:%02x:%02x:%02x:%02x; looking for it%s", host.b[0], host.b[1],
                 host.b[2], host.b[3], host.b[4], host.b[5], ch ? " where it was" : "");
    } else {
        g_eng->startPairing(KIND_CAMSAT, g_name, CAMSAT_VERSION);
        g_pairUntil = ms() + 5u * 60u * 1000u;
        g_led = Led::Pairing;
        ESP_LOGI(TAG, "not paired: pairing for 5 minutes. On the board: LINK PAIR");
    }
    memset(key, 0, sizeof(key));
    g_quietSince = ms();

    xTaskCreatePinnedToCore(camTask, "camera", 8192, nullptr, 2, &g_camTask, 0);
    xTaskCreatePinnedToCore(linkTask, "link", 8192, nullptr, 5, nullptr, 1);
#ifdef CAMSAT_SELFTEST
    // One picture at boot with made-up words, on no session: the camera, the
    // correction and the watermark without a board.
    vTaskDelay(pdMS_TO_TICKS(1500));
    g_job.r = SnapReq();
    g_job.r.mark = true;
    snprintf(g_job.r.board, sizeof(g_job.r.board), "\xC2\xB5nleashed Bench");
    snprintf(g_job.r.when, sizeof(g_job.r.when), "2026-09-26 12:00");
    snprintf(g_job.r.who, sizeof(g_job.r.who), "selftest");
    snprintf(g_job.r.comment, sizeof(g_job.r.comment), "camsat selftest");
    g_job.sess = 0;
    g_job.ph.store(P_TAKING);
    g_led = Led::Busy;
    xTaskNotifyGive(g_camTask);
#endif
}
