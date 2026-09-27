// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/main.cpp
// Module:       The satellite
//
// Purpose:      Pairs with up to five µnleashed boards (1.2.0), keeps the
//               link up, and takes a picture whenever a board sends SNAP:
//               for a caller, or because this satellite's motion sensor or
//               its timer asked (EVENT; each board that wants that kind
//               answers with SNAP, so the board names, dates and limits
//               every picture). A caller's SNAP waits in a queue while the
//               camera works (satsched.h: two a board, eight in all, in turn);
//               an EVENT's picture is taken once and sent to every board
//               that answered. The first board paired owns the satellite:
//               its camera settings are the ones used.
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
#include "driver/rtc_io.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp32/rtc.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board.h"
#include "cam.h"
#include "link.h"
#include "linkfam.h"
#include "radio.h"
#include "satsched.h"
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

// What each board wants delivered (RECV_*), by the engine's slot for it,
// with RECV_SAID once the board has said: a board from before 1.2.0 never
// does, keeps its own timelapse clock, and so is not asked by this one's
// (code review: it took every timelapse twice).
uint8_t     g_recv[Engine::kHosts];
uint32_t    g_shareUntil = 0;             // an owner's share window, for the LED

// The one picture in hand, and the boards it goes to: one for a caller's
// SNAP, every board that answered for an EVENT's. The link task owns every
// field but the phase, which the camera task moves from Taking to Ready or
// Failed. Sending goes board by board (cur); each is a transfer of its own,
// sealed with that board's key.
enum : uint8_t { P_IDLE, P_TAKING, P_READY, P_FAILED, P_SENDING };
struct Job {
    std::atomic<uint8_t> ph{ P_IDLE };
    sched::Req to[sched::kBoards];
    uint8_t  n = 0;
    uint8_t  cur = 0;
    uint16_t sess = 0;                    // the board being sent to now: to[cur]'s
    bool     group = false;               // an EVENT's picture (g_group), not a queued SNAP's
    SnapReq  r;
    Pic      pic;
};
Job g_job;
TaskHandle_t g_camTask = nullptr;
sched::Queue g_q;                         // callers' SNAPs waiting for the camera
sched::Group g_group;                     // the EVENT being answered

// Waking and sleeping
uint8_t  g_wakeKind = 0;                  // CEV_* the wake is for, 0 none
bool     g_wakeSent = false;
uint32_t g_firstUpAt = 0;                 // the first board up this wake
uint32_t g_quietSince = 0;                // nothing to do since, for sleep
uint32_t g_motionAt = 0;                  // the last motion EVENT
bool     g_motionWas = false;
uint32_t g_forgetFrom = 0;
uint32_t g_tlNextMs = 0;                  // awake: the next timelapse EVENT (1.2.0)
// An EVENT the camera was too busy to send: tried again as soon as it is
// free, for up to 10 s, so a visitor during another board's picture is not
// lost (code review).
uint8_t  g_pendKind = 0;
uint32_t g_pendAt = 0;

// Across deep sleep, on the RTC clock (esp_timer starts again at each wake,
// the RTC clock keeps counting through sleep): the last motion EVENT, so the
// hold-off holds across sleeps and one visitor is not a burst, and when the
// next timelapse picture is due, so a wake to re-arm the sensor is not taken
// for one.
RTC_DATA_ATTR uint64_t r_motionUs = 0;
RTC_DATA_ATTR bool     r_motionSet = false;
RTC_DATA_ATTR uint64_t r_tlDueUs = 0;

uint64_t rtcUs() { return esp_rtc_get_time_us(); }

// holdLeftUs: how much of the motion hold-off is still to run, 0 when none.
uint64_t holdLeftUs() {
    if (!r_motionSet) return 0;
    const uint64_t hold = static_cast<uint64_t>(g_set.holdoffS) * 1000000ull, now = rtcUs();
    return now < r_motionUs + hold ? r_motionUs + hold - now : 0;
}

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
// saveBoards: every board this satellite is paired with, as the engine holds
// them, to NVS (a pairing, an unpairing, a board's wants changing).
void saveBoards() {
    store::BoardRec b[store::kBoards] = {};
    uint8_t n = 0;
    for (uint8_t i = 0; i < Engine::kHosts && n < store::kBoards; ++i) {
        if (!g_eng->peerUsed(i)) continue;
        store::BoardRec& r = b[n++];
        memcpy(r.mac, g_eng->peerMac(i).b, 6);
        memcpy(r.key, g_eng->peerKey(i), 16);
        r.ord = g_eng->peerOrd(i);
        r.recv = g_recv[i];
        snprintf(r.name, sizeof(r.name), "%s", g_eng->peerName(i));
    }
    store::saveBoards(b, n);
    memset(b, 0, sizeof(b));
}

// sendFail: SNAP_FAIL to one board. place, for CE_BUSY, is the requests
// ahead (sched::kFull when the queue is full); 0 leaves the byte off.
void sendFail(uint8_t board, uint16_t sess, uint16_t req, uint8_t code, uint8_t place = 0) {
    uint8_t b[4];
    put16(b, req);
    b[2] = code;
    b[3] = place;
    if (code != CE_BUSY) g_lastErr = code;
    g_eng->send(board, sess, FAM_CAMERA, CAM_SNAP_FAIL, b, code == CE_BUSY ? 4 : 3);
}

// sendStatus: STATUS to every board that is up, each on a session of its own.
void sendStatus() {
    uint8_t b[28] = {};
    b[0] = cam::sensor()[0] ? 1 : 0;
    b[1] = cam::maxSize();
    put32(b + 2, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    put32(b + 6, static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    put32(b + 10, ms() / 1000);
    b[14] = g_lastErr;
    snprintf(reinterpret_cast<char*>(b + 16), 12, "%s", cam::sensor());
    for (uint8_t i = 0; i < Engine::kHosts; ++i) {
        if (!g_eng->hostUp(i)) continue;
        const uint16_t sess = g_eng->openSession(i, FAM_CAMERA);
        if (!sess) continue;
        g_eng->send(i, sess, FAM_CAMERA, CAM_STATUS, b, sizeof(b));
        g_eng->closeAfter(i, sess);
    }
}

// sendSettingsOk: the settings as this satellite runs them (the owner's), to
// one board: byte 21 what that board wants, byte 22 whether it is the owner,
// and that this satellite drives its own timelapse by EVENT.
void sendSettingsOk(uint8_t board, uint16_t sess) {
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
    b[21] = static_cast<uint8_t>((g_recv[board] & RECV_ALL) | RECV_SAID);
    b[22] = static_cast<uint8_t>((static_cast<int>(board) == g_eng->ownerIndex() ? SO_OWNER : 0) | SO_EVENTS);
    g_eng->send(board, sess, FAM_CAMERA, CAM_SETTINGS_OK, b, sizeof(b));
}

// sendEvent: ask for a picture (motion, or the timelapse), to every board
// that is up and wants that kind. Each that answers with a SNAP gets the
// same picture (loop: g_group). True when it went, or when no board wants
// one (nothing to wait for). saidOnly: only boards from 1.2.0 on (the awake
// timelapse clock; an older board runs its own).
bool sendEvent(uint8_t kind, bool saidOnly = false) {
    if (g_job.ph.load() != P_IDLE || g_group.active()) return false;
    const uint32_t now = ms();
    g_group.begin(kind, now);
    uint8_t b[5];
    b[0] = kind;
    put32(b + 1, unixNow());
    uint8_t asked = 0;
    for (uint8_t i = 0; i < Engine::kHosts; ++i) {
        if (!g_eng->hostUp(i) || !sched::wants(g_recv[i], kind)) continue;
        if (saidOnly && !(g_recv[i] & RECV_SAID)) continue;
        const uint16_t sess = g_eng->openSession(i, FAM_CAMERA);
        if (!sess) continue;
        if (g_eng->send(i, sess, FAM_CAMERA, CAM_EVENT, b, sizeof(b)) != 1) { g_eng->closeSession(i, sess); continue; }
        g_group.asked(i, sess);
        ++asked;
    }
    if (!asked) {
        g_group.end();
        ESP_LOGI(TAG, "no board wants a %s picture", kind == CEV_MOTION ? "motion" : "timelapse");
        return true;
    }
    ESP_LOGI(TAG, "asked %u board%s for a %s picture", asked, asked == 1 ? "" : "s",
             kind == CEV_MOTION ? "motion" : "timelapse");
    g_quietSince = now;
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
    if (rtc_gpio_is_valid_gpio(g)) rtc_gpio_deinit(g);       // back from deep sleep's RTC mux
    gpio_reset_pin(g);
    gpio_set_direction(g, GPIO_MODE_INPUT);
    gpio_set_pull_mode(g, GPIO_PULLDOWN_ONLY);
}

// onSettings: SETTINGS from a board. What it wants delivered (byte 21) is
// every board's own; the camera settings (bytes 0 to 20) are taken only from
// the owner, so two boards cannot fight over one camera. Every board is
// answered with the settings the satellite runs.
void onSettings(uint8_t board, uint16_t sess, const uint8_t* p, size_t n) {
    if (n < 21 || board >= Engine::kHosts) return;
    if (n >= 22 && (p[21] & RECV_SAID)) {
        const uint8_t want = static_cast<uint8_t>((p[21] & RECV_ALL) | RECV_SAID);
        if (want != g_recv[board]) {
            g_recv[board] = want;
            saveBoards();
            ESP_LOGI(TAG, "\"%s\" wants %s", g_eng->peerName(board),
                     (want & RECV_ALL) == RECV_ALL ? "timelapse and motion"
                     : (want & RECV_ALL) == RECV_TIMELAPSE ? "timelapse only"
                     : (want & RECV_ALL) == RECV_MOTION ? "motion only" : "no timed pictures");
        }
    }
    if (static_cast<int>(board) == g_eng->ownerIndex()) {
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
        const bool tlChanged = s.tlMin != g_set.tlMin || s.tlSec != g_set.tlSec || s.sleep != g_set.sleep;
        g_set = s;
        if (tlChanged) g_tlNextMs = 0;                 // the awake timer starts again from now
        if (changed) {
            store::saveSettings(g_set);
            motionPinSetup();
            ESP_LOGI(TAG, "settings: %s q%u flash %u, timelapse %u:%02u, motion %s (pin %u, %u s), %s",
                     g_set.pic.size == CS_UXGA ? "uxga" : g_set.pic.size == CS_SXGA ? "sxga" : g_set.pic.size == CS_XGA ? "xga"
                     : g_set.pic.size == CS_SVGA ? "svga" : g_set.pic.size == CS_VGA ? "vga" : "small",
                     g_set.pic.quality, g_set.pic.flash, g_set.tlMin, g_set.tlSec, g_set.motion ? "on" : "off",
                     g_set.motionPin, g_set.holdoffS, g_set.sleep ? "deep sleep between pictures" : "awake");
        }
    }
    sendSettingsOk(board, sess);
    g_eng->closeAfter(board, sess);
}

// onSnap: a board asks for a picture. The answer to this satellite's EVENT
// joins the group (one picture for every board that answers); anything else
// waits its turn in the queue, or is told how many are ahead.
void onSnap(uint8_t board, uint16_t sess, const uint8_t* p, size_t n) {
    if (n < 6 || board >= Engine::kHosts) return;
    if (g_group.answer(board, sess, p, n, ms())) return;
    uint8_t place = 0;
    if (!g_q.add(board, sess, p, n, place)) {
        sendFail(board, sess, get16(p), CE_BUSY, place);
        g_eng->closeAfter(board, sess);
        ESP_LOGI(TAG, "SNAP from \"%s\" refused: %s", g_eng->peerName(board),
                 place == sched::kFull ? "the queue is full" : "that board has two waiting");
    }
}

bool evMessage(void*, uint8_t board, uint16_t sess, uint8_t family, uint8_t type, const uint8_t* p, size_t n) {
    if (family != FAM_CAMERA) return true;
    if (type == CAM_SNAP) onSnap(board, sess, p, n);
    else if (type == CAM_SETTINGS) onSettings(board, sess, p, n);
    g_quietSince = ms();
    return true;
}

// parseSnap: a SNAP's bytes into what the camera is told. The size,
// quality and flash are the owner's to choose: another board's SNAP takes
// the settings (the owner's) rather than lighting the owner's flash
// (code review).
void parseSnap(const sched::Req& q, SnapReq& r) {
    r = SnapReq();
    const uint8_t* p = q.p;
    const bool owner = static_cast<int>(q.board) == g_eng->ownerIndex();
    r.size = owner ? p[2] : 0;
    r.quality = owner ? p[3] : 0;
    r.flash = owner ? p[4] : 0xFF;
    if (q.n >= 67) {
        r.mark = p[6] != 0;
        text(r.board, sizeof(r.board), p + 7, 20);
        text(r.when, sizeof(r.when), p + 27, 16);
        text(r.who, sizeof(r.who), p + 43, 24);
        text(r.comment, sizeof(r.comment), p + 67, q.n - 67);
    }
}

// startJob: take one picture for the boards in to[0..n): the first one's
// words go on it (for an EVENT's, the owner's when it answered).
void startJob(const sched::Req* to, uint8_t n, bool group) {
    Job& j = g_job;
    j.group = group;
    for (uint8_t i = 0; i < n; ++i) j.to[i] = to[i];
    j.n = n;
    j.cur = 0;
    j.sess = to[0].sess;
    parseSnap(to[0], j.r);
    j.ph.store(P_TAKING);
    g_led = Led::Busy;
    xTaskNotifyGive(g_camTask);
    ESP_LOGI(TAG, "SNAP %u for %s, %u board%s", static_cast<unsigned>(to[0].req),
             j.r.who[0] ? j.r.who : "the board", n, n == 1 ? "" : "s");
}

// jobDone: every board's session is let go here (the engine keeps one the
// far end has finished with for two minutes, and its table holds 16: the
// bench, 2026-09-26), and the next request can go.
void jobDone() {
    Job& j = g_job;
    for (uint8_t i = 0; i < j.n; ++i)
        if (j.to[i].sess) g_eng->closeAfter(j.to[i].board, j.to[i].sess);
    j.n = 0;
    j.cur = 0;
    j.sess = 0;
    if (j.group) g_group.end();                 // its EVENT is answered
    j.group = false;
    j.ph.store(P_IDLE);
    g_led = g_eng->hostUp() ? Led::Up : Led::Search;
    g_quietSince = ms();
}

// nextBoard: the picture has gone to to[cur] (or could not): on to the next
// board still there, or done.
void nextBoard() {
    Job& j = g_job;
    while (++j.cur < j.n && !j.to[j.cur].sess) {}
    if (j.cur >= j.n) { jobDone(); return; }
    j.sess = j.to[j.cur].sess;
    j.ph.store(P_READY);
}

uint32_t g_sendAt = 0;

void evBulkSent(void*, uint8_t board, uint16_t sess, uint8_t, bool ok) {
    Job& j = g_job;
    if (j.ph.load() != P_SENDING || j.cur >= j.n || sess != j.sess || board != j.to[j.cur].board) return;
    const uint32_t took = ms() - g_sendAt;
    const PeerStats& st = g_eng->peerStats(board);
    ESP_LOGI(TAG, "picture %u to \"%s\" %s: %u bytes in %u ms (%u KB/s); link tx %u, retries %u, drops %u; "
                  "radio fails %u%s",
             static_cast<unsigned>(j.to[j.cur].req), g_eng->peerName(board), ok ? "taken" : "not delivered",
             static_cast<unsigned>(j.pic.len), static_cast<unsigned>(took),
             static_cast<unsigned>(took ? j.pic.len / took : 0), static_cast<unsigned>(st.tx),
             static_cast<unsigned>(st.retries), static_cast<unsigned>(st.drops),
             static_cast<unsigned>(g_radio.sendFails()), g_radio.slowRate(g_eng->peerMac(board)) ? ", at 1 Mbps" : "");
    if (!ok) g_lastErr = CE_CAPTURE;
    g_eng->closeAfter(board, sess);
    j.to[j.cur].sess = 0;
    nextBoard();
}

void evReset(void*, uint8_t board, uint16_t sess, uint8_t, uint8_t reason) {
    // A board that gave up on a queued SNAP: not taken for nobody.
    if (g_q.dropSess(board, sess)) ESP_LOGI(TAG, "\"%s\" gave up a queued SNAP", g_eng->peerName(board));
    Job& j = g_job;
    for (uint8_t i = 0; i < j.n; ++i) {
        if (j.to[i].board != board || j.to[i].sess != sess) continue;
        ESP_LOGW(TAG, "\"%s\" ended the picture's session (%u)", g_eng->peerName(board), reason);
        j.to[i].sess = 0;
        const uint8_t ph = j.ph.load();
        if (i == j.cur && (ph == P_SENDING || ph == P_READY)) nextBoard();
    }
}

void evPeerState(void*, uint8_t board, bool up) {
    ESP_LOGI(TAG, "link to \"%s\" %s", g_eng->peerName(board), up ? "up" : "down");
    if (g_job.ph.load() == P_IDLE) g_led = g_eng->hostUp() ? Led::Up : Led::Search;
    if (up) {
        g_radio.fastAgain(g_eng->peerMac(board));
        store::setLastChannel(g_radio.channel());
        g_statusAt = 0;                          // STATUS now
        if (!g_firstUpAt) g_firstUpAt = ms();
        return;
    }
    // Gone: its waiting SNAPs go, an EVENT does not wait for it, and the
    // picture being sent skips it (the one on the air now ends by itself).
    g_q.drop(board, [board](uint16_t s) { g_eng->closeSession(board, s); });
    g_group.forget(board);
    Job& j = g_job;
    for (uint8_t i = 0; i < j.n; ++i)
        if (j.to[i].board == board && (i != j.cur || j.ph.load() != P_SENDING)) j.to[i].sess = 0;
}

void evPaired(void*, uint8_t board, const PairInfo& who) {
    if (board < Engine::kHosts) g_recv[board] = RECV_ALL;
    saveBoards();
    const bool first = !g_paired;
    g_paired = true;
    g_pairUntil = 0;
    g_shareUntil = 0;
    ESP_LOGI(TAG, "paired with \"%s\" %02x:%02x:%02x:%02x:%02x:%02x, code %04u%s", who.name, who.mac.b[0],
             who.mac.b[1], who.mac.b[2], who.mac.b[3], who.mac.b[4], who.mac.b[5], static_cast<unsigned>(who.code),
             first ? ": it owns this satellite" : "");
    g_led = g_eng->hostUp() ? Led::Up : Led::Search;
}

// A board let this satellite go (UNPAIR), or its owner revoked one.
void evUnpaired(void*, uint8_t board) {
    if (board < Engine::kHosts) g_recv[board] = RECV_ALL;
    g_q.drop(board, [](uint16_t) {});
    g_group.forget(board);
    saveBoards();
    uint8_t left = 0;
    for (uint8_t i = 0; i < Engine::kHosts; ++i) left += g_eng->peerUsed(i);
    ESP_LOGW(TAG, "a board let this satellite go; %u left", left);
    if (!left) {
        g_paired = false;
        g_led = Led::Off;
        ESP_LOGW(TAG, "no board left: hold IO0 for 5 s, or reset, to pair again");
    }
}

// The owner opened this satellite to one more board.
void evShareOpened(void*, uint32_t secs) {
    g_shareUntil = ms() + secs * 1000u;
    g_led = Led::Pairing;
    ESP_LOGI(TAG, "the owner lets one more board pair for %u s: LINK PAIR on it now", static_cast<unsigned>(secs));
}

// Pairing: the code, worked out before the sysop answers at the board, so
// whoever has this console can compare the two.
void evPairAsk(void*, const PairInfo& who) {
    ESP_LOGW(TAG, "pairing with the board \"%s\": code %04u. The board shows the same code if nobody is in between.",
             who.name, static_cast<unsigned>(who.code));
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
// sendPicture: the finished picture to the board it is going to now, as one
// bulk message: the header (that board's request id and reason), then the
// JPEG, from the buffer the camera wrote.
void sendPicture() {
    Job& j = g_job;
    const sched::Req& to = j.to[j.cur];
    uint8_t* h = j.pic.buf;
    memset(h, 0, cam::kHead);
    put16(h, to.req);
    h[2] = to.reason;
    put16(h + 4, j.pic.w);
    put16(h + 6, j.pic.h);
    put32(h + 8, unixNow());
    const int r = g_eng->sendBulk(to.board, to.sess, FAM_CAMERA, CAM_PICTURE, h,
                                  static_cast<uint32_t>(cam::kHead + j.pic.len));
    if (r == 1) { j.ph.store(P_SENDING); g_sendAt = ms(); return; }
    if (r < 0) {
        ESP_LOGW(TAG, "the picture could not be queued for \"%s\"", g_eng->peerName(to.board));
        sendFail(to.board, to.sess, to.req, CE_NOMEM);
        g_eng->closeAfter(to.board, to.sess);
        j.to[j.cur].sess = 0;
        nextBoard();
    }
    // 0: not now, again next pass
}

// failJob: the camera gave no picture: every board it was for is told.
void failJob() {
    Job& j = g_job;
    for (uint8_t i = 0; i < j.n; ++i)
        if (j.to[i].sess) sendFail(j.to[i].board, j.to[i].sess, j.to[i].req, j.pic.failCode);
    jobDone();
}

// dispatch: with the camera free, the next thing to take: an EVENT's group
// once its answers are in, else the next SNAP in the queue, in turn.
void dispatch(uint32_t now) {
    if (g_job.ph.load() != P_IDLE) return;
    // Static: the link task's stack is 8 KB and a Req is about 236 bytes.
    static sched::Req to[sched::kBoards];
    if (g_group.active()) {
        if (g_group.ready(now)) {
            const uint8_t n = g_group.take(g_eng->ownerIndex(), to, [](uint8_t b, uint16_t s) {
                g_eng->closeSession(b, s);
            });
            if (n) { startJob(to, n, true); return; }
            g_group.end();
        } else if (g_group.expired(now)) {
            // Nobody answered: the boards let the EVENT go (hold-off, card).
            g_group.abandon([](uint8_t b, uint16_t s) { g_eng->closeSession(b, s); });
        }
    }
    if (g_q.next(to[0])) startJob(to, 1, false);
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
        ESP_LOGW(TAG, "IO0 held 5 s: forgetting every board and starting again");
        store::forget();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
}

void motionWatch(uint32_t now) {
    if (!g_set.motion || !g_eng->hostUp()) return;
#ifdef CAMSAT_BENCH_MOTION
    // The bench, with no PIR wired: "movement" for 5 s every 45 s.
    const bool high = (now % 45000) < 5000;
#else
    const bool high = gpio_get_level(static_cast<gpio_num_t>(g_set.motionPin)) != 0;
#endif
    const bool rise = high && !g_motionWas;
    g_motionWas = high;
    if (!rise || holdLeftUs()) return;
    g_motionAt = now;
    r_motionUs = rtcUs();
    r_motionSet = true;
    if (!sendEvent(CEV_MOTION)) { g_pendKind = CEV_MOTION; g_pendAt = now; }
}

// sleepWatch: with deep sleep on, sleep once nothing is going on: a picture
// sent, or a wake the board never answered. Awake for the first minute
// after a power-up, so the board can send settings and a sysop can pair.
// goSleep: into deep sleep, woken by the timelapse's timer, by the motion
// sensor, or by a short timer that only re-arms the sensor. The sensor is
// armed only when its hold-off has run out and its line is low: a PIR
// holds OUT high for seconds after it fires, and a level wake armed on a
// high line wakes the chip at once, which is how one visitor becomes a
// burst of wakes.
void goSleep() {
    const uint64_t tl = (static_cast<uint64_t>(g_set.tlMin) * 60u + g_set.tlSec) * 1000000ull;
    const bool motion = g_set.motion && motionPinOk(g_set.motionPin);
    const uint64_t now = rtcUs();
    uint64_t timerUs = 0;
    if (tl) {
        if (r_tlDueUs <= now) r_tlDueUs = now + tl;
        timerUs = r_tlDueUs - now;
    }
    bool armed = false;
    if (motion) {
        const gpio_num_t g = static_cast<gpio_num_t>(g_set.motionPin);
        const uint64_t left = holdLeftUs();
        const bool high = gpio_get_level(g) != 0;
        if (!left && !high) {
            // The line held low while asleep, so an unplugged sensor cannot
            // wake the satellite on noise: the pull lives in the RTC domain,
            // which stays powered for it.
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
            rtc_gpio_pullup_dis(g);
            rtc_gpio_pulldown_en(g);
            esp_sleep_enable_ext0_wakeup(g, 1);
            armed = true;
        } else {
            // Back when the hold-off ends, or in 5 s for a line still high,
            // only to arm the sensor.
            const uint64_t rearm = left ? left : 5000000ull;
            if (!timerUs || rearm < timerUs) timerUs = rearm;
        }
    }
    if (timerUs) esp_sleep_enable_timer_wakeup(timerUs);
    ESP_LOGI(TAG, "sleeping: timelapse %s, motion %s, timer %u ms",
             tl ? "on" : "off", armed ? "armed" : (motion ? "held" : "off"),
             static_cast<unsigned>(timerUs / 1000ull));
    ledSet(false);
    esp_deep_sleep_start();
}

void sleepWatch(uint32_t now) {
    if (!g_set.sleep || !g_paired || g_job.ph.load() != P_IDLE) return;
    const uint32_t tl = static_cast<uint32_t>(g_set.tlMin) * 60u + g_set.tlSec;
    const bool motion = g_set.motion && motionPinOk(g_set.motionPin);
    if (!tl && !motion) return;                          // nothing would wake it
    const bool cold = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED;
    if (cold && now - g_bootMs < 60000) return;
    // Awake while a board still owes an answer to the EVENT (its SNAP, or
    // the 10 s after which the EVENT is let go), while a SNAP waits its turn,
    // while the wake's EVENT has not gone yet (15 s to find the boards), and
    // for 2 s after the last thing that happened, so a picture's last
    // acknowledgements get out.
    if (g_group.active() || g_q.size() || g_pendKind) return;
    if (g_wakeKind && !g_wakeSent && now - g_bootMs < 15000) return;
    const uint32_t t = ms();                             // not the pass's now: g_quietSince may be newer
    if (static_cast<int32_t>(t - g_quietSince) < 2000) return;
    ESP_LOGI(TAG, "awake %u ms", static_cast<unsigned>(t - g_bootMs));
    goSleep();
}

void linkTask(void*) {
    for (;;) {
        const uint32_t now = ms();
        // Four polls a tick. Polling the whole millisecond while a picture
        // goes out was measured on link.6 and changed nothing (80-85 KB/s
        // either way): the pace is set by the board's acknowledgements.
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
                // The picture itself, in base64 between markers, for the bench to look at.
                static const char k64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                printf("-----BEGIN JPEG-----\n");
                char line[77];
                size_t at = 0;
                for (size_t i = 0; i < g_job.pic.len; i += 3) {
                    const uint32_t v = b[i] << 16 | (i + 1 < g_job.pic.len ? b[i + 1] << 8 : 0) |
                                       (i + 2 < g_job.pic.len ? b[i + 2] : 0);
                    line[at++] = k64[v >> 18 & 63];
                    line[at++] = k64[v >> 12 & 63];
                    line[at++] = i + 1 < g_job.pic.len ? k64[v >> 6 & 63] : '=';
                    line[at++] = i + 2 < g_job.pic.len ? k64[v & 63] : '=';
                    if (at == 76 || i + 3 >= g_job.pic.len) { line[at] = '\0'; printf("%s\n", line); at = 0; }
                }
                printf("-----END JPEG-----\n");
            }
            g_job.ph.store(P_IDLE);
            g_led = Led::Pairing;
#ifdef CAMSAT_SELFTEST_AGAIN
            // The bench: a settings save between pictures, then another one.
            static int again = 0;
            if (again++ < 2) {
                if (again == 1) { store::saveSettings(g_set); ESP_LOGI(TAG, "selftest: settings saved"); }
                vTaskDelay(pdMS_TO_TICKS(2000));
                g_job.ph.store(P_TAKING);
                xTaskNotifyGive(g_camTask);
            }
#endif
        }
        else
#endif
        if (ph == P_READY) sendPicture();
        else if (ph == P_FAILED) {
            const bool stuck = g_job.pic.stuck;
            failJob();
            if (stuck) {
                // The sensor answered earlier this boot and cannot be woken:
                // a restart clears it (the bench, 2026-09-26). The failure
                // goes first; the pairing and the channel are kept.
                ESP_LOGE(TAG, "the camera is stuck: restarting");
                for (int i = 0; i < 50; ++i) { g_eng->poll(); vTaskDelay(pdMS_TO_TICKS(10)); }
                esp_restart();
            }
        }
        // An EVENT the camera was too busy for goes first, so a steady queue
        // of SNAPs cannot starve it; after 10 s it is let go.
        if (g_pendKind) {
            if (static_cast<int32_t>(now - g_pendAt) > 10000) {
                ESP_LOGW(TAG, "a %s picture was never taken: the camera stayed busy",
                         g_pendKind == CEV_MOTION ? "motion" : "timelapse");
                g_pendKind = 0;
            } else if (g_eng->hostUp() && sendEvent(g_pendKind)) {
                g_pendKind = 0;
            }
        }
        dispatch(now);
        if (g_eng->hostUp()) {
            if (!g_statusAt || now - g_statusAt >= 60000) { g_statusAt = now; sendStatus(); }
            // The wake's EVENT once every board is up, or 2 s after the first:
            // they share a channel, so the rest are found at once or not at all.
            if (g_wakeKind && !g_wakeSent && (g_eng->peersUp() == g_eng->peerCount() ||
                                              static_cast<int32_t>(now - g_firstUpAt) >= 2000))
                g_wakeSent = sendEvent(g_wakeKind);
            // Awake, the timelapse is this satellite's own clock (1.2.0): one
            // EVENT, one picture for every board that wants it. Before 1.2.0
            // the board's clock asked, one board, one SNAP.
            const uint32_t tl = (static_cast<uint32_t>(g_set.tlMin) * 60u + g_set.tlSec) * 1000u;
            if (tl && !g_set.sleep) {
                if (!g_tlNextMs) g_tlNextMs = now + tl;
                else if (static_cast<int32_t>(now - g_tlNextMs) >= 0 && sendEvent(CEV_TIMELAPSE, true)) g_tlNextMs = now + tl;
            }
        }
        if (g_shareUntil && static_cast<int32_t>(now - g_shareUntil) >= 0) {
            g_shareUntil = 0;
            if (g_led == Led::Pairing) g_led = g_eng->hostUp() ? Led::Up : Led::Search;
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
        case ESP_SLEEP_WAKEUP_TIMER: {
            const bool tlDue = (g_set.tlMin || g_set.tlSec) && rtcUs() + 1000000ull >= r_tlDueUs;
            if (!tlDue) {
                // Only to arm the motion sensor: no radio, straight back.
                ESP_LOGI(TAG, "woken to arm the motion sensor");
                goSleep();
            }
            r_tlDueUs = 0;                                   // the next one from now
#ifdef CAMSAT_BENCH_MOTION
            // The bench, with no PIR wired: a timelapse wake stands in for a
            // motion wake, so the board's side of one after sleep is tested.
            g_wakeKind = CEV_MOTION;
#else
            g_wakeKind = CEV_TIMELAPSE;
#endif
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT0:
            g_wakeKind = CEV_MOTION;
            g_motionWas = true;
            r_motionUs = rtcUs();                            // the hold-off starts at the wake
            r_motionSet = true;
            break;
        default:
            r_motionSet = false;                             // a power-up or reset: nothing held
            r_tlDueUs = 0;
            break;
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
    ev.pairAsk = evPairAsk;
    ev.channel = evChannel;
    ev.clock = evClock;
    ev.unpaired = evUnpaired;
    ev.shareOpened = evShareOpened;
    static uint8_t win[4 * kPayloadMax];            // a satellite receives no bulk
    g_eng = new Engine(Role::Peer, g_radio, ev, 4, win);
    if (!g_eng->ok()) {
        ESP_LOGE(TAG, "the link engine could not start");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    g_eng->setIdentity(KIND_CAMSAT, CAMSAT_VERSION, 1u << FAM_CAMERA);
    g_eng->setFastRescan(true);

    for (uint8_t i = 0; i < Engine::kHosts; ++i) g_recv[i] = RECV_ALL;
    store::BoardRec boards[store::kBoards];
    const uint8_t nb = store::loadBoards(boards);
    for (uint8_t i = 0; i < nb; ++i) {
        Mac m;
        memcpy(m.b, boards[i].mac, 6);
        const int slot = g_eng->addPeer(m, boards[i].key, KIND_UNKNOWN, boards[i].ord,
                                        boards[i].name[0] ? boards[i].name : nullptr);
        if (slot >= 0 && slot < Engine::kHosts) g_recv[slot] = static_cast<uint8_t>(boards[i].recv & (RECV_ALL | RECV_SAID));
        if (slot >= 0) g_paired = true;
    }
    memset(boards, 0, sizeof(boards));
    if (g_paired) {
        g_led = Led::Search;
        ESP_LOGI(TAG, "paired with %u board%s; looking for them%s", nb, nb == 1 ? "" : "s", ch ? " where they were" : "");
    } else {
        g_eng->startPairing(KIND_CAMSAT, g_name, CAMSAT_VERSION);
        g_pairUntil = ms() + 5u * 60u * 1000u;
        g_led = Led::Pairing;
        ESP_LOGI(TAG, "not paired: pairing for 5 minutes. On the board: LINK PAIR");
    }
    g_quietSince = ms();

    xTaskCreatePinnedToCore(camTask, "camera", 8192, nullptr, 2, &g_camTask, 0);
    xTaskCreatePinnedToCore(linkTask, "link", 8192, nullptr, 5, nullptr, 1);
#ifdef CAMSAT_SELFTEST
    // One picture at boot with made-up words, on no session: the camera, the
    // correction and the watermark without a board.
    vTaskDelay(pdMS_TO_TICKS(1500));
    g_job.r = SnapReq();
#ifdef CAMSAT_SELFTEST_RAW
    g_job.r.mark = false;
    g_set.pic.levels = false;
#else
    g_job.r.mark = true;
#endif
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
