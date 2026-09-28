// ===========================================================================
//  µnleashed camsat: the board's half
// ===========================================================================
//
// File:         bbs/camsat.cpp
// Module:       Plugins / the camera satellite (built into the firmware from
//               this repository: LINK.md, "Plugins in their own repositories")
//
// Purpose:      A camera for any board. A satellite (an ESP32-CAM running
//               firmware/) pairs with the board over the µnleashed link
//               (LINK PAIR), and this plugin asks it for pictures and files
//               them in Photos exactly as a built-in camera's: named by
//               camrules (SNAP-date, date+handle or by handle; timelapse/TL-
//               and motion/MO- for the board's own), the same watermark and
//               comment, the same download offer.
//
//               Each paired satellite is a camera in the core's list
//               (photos::Camera, 1.2.0): SNAPSHOT and CAMERA are the core's,
//               one pair for every camera on the board, and so is the
//               per-caller budget (photos::budget, 10 an hour and 20 a day
//               across every camera, the sysop free of it). A satellite is
//               listed while it is paired, named as LINK names it, and says
//               so when it is not answering.
//
//               The board chooses every word and every name. The satellite
//               takes the picture and does the pixel work (levels, gamma, the
//               watermark), because a board with no camera has no codec for
//               it; the words it draws come from here, in SNAP.
//
//               Every picture is a SNAP from here: a caller's SNAPSHOT, the
//               timelapse this plugin keeps while a satellite stays awake, or
//               a satellite's EVENT (its motion sensor, or its own timer when
//               it deep-sleeps between pictures), which this answers with a
//               SNAP on the same session, or does not.
//
//               Where the work runs (Rule no. 1): messages on the loop (the
//               link's tick); the picture's bytes and its filing on the
//               runner (bulkData, bulkFinish), never the loop; the caller is
//               told from bulkEnd on the loop.
//
// Libraries:    none beyond the core
// Targets:      every board the core builds for (off until switched on)
// See also:     ../firmware/, the core's LINK.md and src/core/linkfam.h
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
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <new>
#include <strings.h>

#include "config.h"
#include "core/bbs.h"
#include "core/claims.h"
#include "core/clock.h"
#include "core/fx.h"
#include "core/linkfam.h"
#include "core/photos.h"
#include "core/plugin.h"
#include "core/runner.h"
#include "core/sysconfig.h"
#include "platform/platform.h"
#include "plugins/camera_rules.h"
#include "plugins/files.h"
#include "plugins/link.h"

UNLEASHED_PLUGIN_API(1, 1);

using namespace linkfam;

namespace {

constexpr const char kName[] = "camsat";
uint8_t g_index = 0xFF;
bool    g_running = false;

// ---------------------------------------------------------------------------
// Settings ([plugin:camsat], CONFIG camsat)
// ---------------------------------------------------------------------------
constexpr char kLevels[] = "all|users|staff|co2|co1|sysop";
constexpr char kSizes[]  = "qqvga|qvga|vga|svga|xga|sxga|uxga";   // linkfam CS_* 1..7
constexpr char kFlashes[] = "off|on|auto";
constexpr char kWbs[]     = "auto|sunny|cloudy|office|home";
constexpr char kGammas[]  = "0.6|0.7|0.8|0.9|1.0|1.1|1.2|1.4|1.6";
constexpr char kPins[]    = "13|14|15|2";

struct Settings {
    PlugLevel snap   = PlugLevel::Users;
    PlugLevel photos = PlugLevel::All;
    uint8_t  size    = CS_XGA;
    uint8_t  quality = 10;
    uint8_t  names   = camrules::NAME_DATE;
    bool     mark    = true;
    uint8_t  flash   = CF_OFF;
    uint16_t tlMin   = 0;
    uint8_t  tlSec   = 0;
    bool     motion  = false;
    uint16_t hold    = 60;
    uint8_t  pin     = 13;
    bool     sleep   = false;
    bool     flip = false, mirror = false;
    int8_t   bright  = 0;
    uint8_t  wb      = 0;
    bool     levels  = true;
    uint8_t  gamma   = 4;
};
Settings g_set;

int wordIndex(const char* list, const char* v) {
    int i = 0;
    for (const char* p = list; *p; ++i) {
        const char* bar = strchr(p, '|');
        const size_t n = bar ? static_cast<size_t>(bar - p) : strlen(p);
        if (strlen(v) == n && !strncasecmp(p, v, n)) return i;
        if (!bar) break;
        p = bar + 1;
    }
    return -1;
}

void wordAt(const char* list, int i, char* out, size_t n) {
    const char* p = list;
    for (; i > 0 && p; --i) { p = strchr(p, '|'); if (p) ++p; }
    if (!p) { snprintf(out, n, "?"); return; }
    const char* bar = strchr(p, '|');
    snprintf(out, n, "%.*s", static_cast<int>(bar ? bar - p : static_cast<long>(strlen(p))), p);
}

bool yes(const char* v) { return !strcasecmp(v, "yes") || !strcasecmp(v, "on") || !strcmp(v, "1"); }

bool num(const char* v, long lo, long hi, long& out) {
    char* end = nullptr;
    out = strtol(v, &end, 10);
    return end && end != v && !*end && out >= lo && out <= hi;
}

bool apply(Settings& g, const char* key, const char* v) {
    long n = 0;
    int w;
    PlugLevel l;
    if      (!strcmp(key, "snap"))        { if (!plugins::levelFromText(v, l)) return false; g.snap = l; }
    else if (!strcmp(key, "photos"))      { if (!plugins::levelFromText(v, l)) return false; g.photos = l; }
    else if (!strcmp(key, "size"))        { if ((w = wordIndex(kSizes, v)) < 0) return false; g.size = static_cast<uint8_t>(w + 1); }
    else if (!strcmp(key, "quality"))     { if (!num(v, 4, 40, n)) return false; g.quality = static_cast<uint8_t>(n); }
    else if (!strcmp(key, "names"))       { if ((w = wordIndex(camrules::kSchemes, v)) < 0) return false; g.names = static_cast<uint8_t>(w); }
    else if (!strcmp(key, "watermark"))   { g.mark = yes(v); }
    else if (!strcmp(key, "flash"))       { if ((w = wordIndex(kFlashes, v)) < 0) return false; g.flash = static_cast<uint8_t>(w); }
    else if (!strcmp(key, "sleep"))       { g.sleep = yes(v); }
    else if (!strcmp(key, "tl_min"))      { if (!num(v, 0, 1440, n)) return false; g.tlMin = static_cast<uint16_t>(n); }
    else if (!strcmp(key, "tl_sec"))      { if (!num(v, 0, 59, n)) return false; g.tlSec = static_cast<uint8_t>(n); }
    else if (!strcmp(key, "motion_on"))   { g.motion = yes(v); }
    else if (!strcmp(key, "motion_hold")) { if (!num(v, 10, 3600, n)) return false; g.hold = static_cast<uint16_t>(n); }
    else if (!strcmp(key, "motion_pin"))  { if (wordIndex(kPins, v) < 0) return false; g.pin = static_cast<uint8_t>(atoi(v)); }
    else if (!strcmp(key, "pic_flip"))    { g.flip = yes(v); }
    else if (!strcmp(key, "pic_mirror"))  { g.mirror = yes(v); }
    else if (!strcmp(key, "pic_bright"))  { if (!num(v, -2, 2, n)) return false; g.bright = static_cast<int8_t>(n); }
    else if (!strcmp(key, "pic_wb"))      { if ((w = wordIndex(kWbs, v)) < 0) return false; g.wb = static_cast<uint8_t>(w); }
    else if (!strcmp(key, "pic_levels"))  { g.levels = yes(v); }
    else if (!strcmp(key, "pic_gamma"))   { if ((w = wordIndex(kGammas, v)) < 0) return false; g.gamma = static_cast<uint8_t>(w); }
    else if (!strcmp(key, "tl") || !strcmp(key, "motion") || !strcmp(key, "pic")) {}   // the page buttons
    else return false;
    return true;
}

void readKey(void*, const char* key, const char* value) {
    if (!strcmp(key, "enabled") || !strcmp(key, "read") || !strcmp(key, "write") || !strcmp(key, "admin")) return;
    if (!apply(g_set, key, value))
        plat::log("camsat: %s = %s is not a value the satellite takes, keeping its own", key, value);
}

uint32_t tlEvery() {
    uint32_t e = static_cast<uint32_t>(g_set.tlMin) * 60u + g_set.tlSec;
    if (e && e < camrules::kTlMin) e = camrules::kTlMin;
    return e;
}

// ---------------------------------------------------------------------------
// The satellites the board has seen this boot, by link peer.
// ---------------------------------------------------------------------------
struct Sat {
    // Its entry in the board's camera list, kept here so the pointer the
    // registry holds lives as long as the listing.
    photos::Camera cam = {};
    char     name[17] = {};
    uint8_t  peer = 0;
    bool     listed = false;
    bool     known = false;
    char     sensor[12] = {};
    uint32_t heap = 0, psram = 0, uptime = 0;
    uint8_t  lastErr = 0;
    uint32_t pictures = 0;
    uint32_t lastAt = 0;                     // epoch of its last picture
    uint32_t eventAt = 0;                    // millis of its last EVENT answered
    // From its SETTINGS_OK (1.2.0, one satellite and several boards):
    bool     owned = true;                   // this board owns it: its settings are used
    bool     events = false;                 // it runs its own timelapse clock (EVENT)
    // What it runs, from its SETTINGS_OK, for SATS.
    bool     sleeps = false;
    uint16_t tlMin = 0;
    uint8_t  tlSec = 0;
    bool     motion = false;
    uint16_t hold = 0;
};
// On the heap from the first start, like the limits: a plugin that is off
// costs the board no static RAM (every official image carries this one).
Sat* g_sat = nullptr;

int satPeer(uint8_t n) { return linkp::peerOfKind(ulink::KIND_CAMSAT, n); }

bool peerUp(int peer) {
    ulink::Engine* e = linkp::engine();
    return e && peer >= 0 && e->peerUp(static_cast<uint8_t>(peer));
}

// firstUp: the first satellite whose link is up, or -1. clocked: only one
// that leaves its timelapse to this board's clock (before 1.2.0).
int firstUp(bool clocked = false) {
    for (uint8_t i = 0; i < ulink::Engine::kPeers; ++i) {
        const int p = satPeer(i);
        if (p < 0) break;
        if (clocked && g_sat && g_sat[p].events) continue;
        if (peerUp(p)) return p;
    }
    return -1;
}

// The photo each node was offered (the built-in's g_offer).
constexpr uint8_t kSlots = BBS_MAX_NODES + 2;
struct Offer { char rel[112]; };
Offer* g_offer = nullptr;

// ---------------------------------------------------------------------------
// The picture in flight: one at a time, board-wide.
// ---------------------------------------------------------------------------
enum : uint8_t { J_IDLE, J_ASKED, J_COMING, J_DONE };
enum : uint8_t { K_CALLER, K_SYSTEM };
struct Job {
    std::atomic<uint8_t> ph{ J_IDLE };
    uint8_t  kind = K_CALLER;
    uint8_t  peer = 0;
    uint16_t sess = 0;
    uint16_t req = 0;
    bool     ours = false;                   // this board opened the session
    uint8_t  node = 0xFF;                    // the caller waiting
    bool     waiting = false;
    char     rel[112] = {};                  // under Photos; the runner moves it a second on if taken
    char     desc[48] = {};                  // FILES.BBS, a caller's only
    char     handle[BBS_USER_MAX + 2] = {};
    uint32_t startedAt = 0, spinAt = 0;
    uint8_t  spin = 0;
    // the runner's side
    photos::Writer w;
    bool     writerOpen = false;
    uint8_t  head[kPictureHeader] = {};
    uint8_t  headGot = 0;
    uint32_t bytes = 0;
    uint16_t pw = 0, ph2 = 0;
    std::atomic<bool> filed{ false };
    char     err[72] = {};                   // the loop's reason
    uint8_t  busyAt = 0;                     // SNAP_FAIL busy: 0 none, 1 + the requests ahead, 0xFF full
    char     rerr[72] = {};                  // the runner's (bulkData, bulkFinish)
    // The part-file is the runner's while it is inside bulkData or
    // bulkFinish, and the loop's otherwise: wbusy is who holds it now. dead
    // tells the runner the loop has given the picture up, so it closes the
    // file itself; dying is the loop waiting to take it back (finish, from
    // the next tick).
    std::atomic<bool> wbusy{ false };
    std::atomic<bool> dead{ false };
    bool     dying = false;
    bool     dyingOk = false;
    // The part-file of a picture given up on the loop (finish, a CONFIG save
    // mid-picture), closed and removed on the runner: on the loop that was a
    // 51-87 ms slow pass on the card (camsat bench, 2026-09-27). Until it is
    // gone no new picture starts, since the next would open the same name.
    struct Closer : runner::Job { photos::Writer w; };
    Closer   closer;
};
Job* g_jobp = nullptr;                     // the heap, from the first start
inline Job& job() { return *g_jobp; }
uint16_t g_reqNext = 1;
uint32_t g_tlSlot = 0;
bool     g_tlPrimed = false;
// A finished picture's session is let go a little after the picture, not at
// once: the engine's last acknowledgement to the satellite goes after this
// board has what it needs, and a session forgotten first leaves the
// satellite retrying into "no such session" (the bench, 2026-09-26).
struct Closing { bool on = false; uint8_t peer = 0; uint16_t sess = 0; uint32_t at = 0; };
Closing  g_closing;
char     g_last[112] = {};
char     g_lastBy[BBS_USER_MAX + 8] = {};

bool busy() { return g_jobp && (g_jobp->ph.load() != J_IDLE || !runner::idle(g_jobp->closer)); }

void closeWork(runner::Job& self) { photos::abandon(static_cast<Job::Closer&>(self).w); }

// abandonLater (loop): the part-file to the runner to close and remove. On
// the loop only if the runner will not take it (its queue is full).
void abandonLater(Job& j) {
    if (runner::done(j.closer)) runner::collect(j.closer);
    if (runner::idle(j.closer)) {
        j.closer.w = j.w;
        j.w = photos::Writer();
        j.closer.work = closeWork;
        j.closer.name = "camsat close";
        if (runner::post(j.closer)) return;
        j.w = j.closer.w;                       // not taken: back, and done here
        j.closer.w = photos::Writer();
    }
    photos::abandon(j.w);
}

bool takeWriter() { bool e = false; return job().wbusy.compare_exchange_strong(e, true); }
void giveWriter() { job().wbusy.store(false); }

// dropWriter (loop): close a picture's part-file, but only while the runner
// is not inside it. False: the runner has it now; it sees dead and closes
// the file itself, and the loop comes back next tick. Closing it under the
// runner's write was a FILE freed while in use (the bench, 2026-09-27: a
// session reset or a CONFIG save while a picture was coming).
bool dropWriter() {
    Job& j = job();
    j.dead.store(true);
    if (!takeWriter()) return false;
    if (j.writerOpen) { abandonLater(j); j.writerOpen = false; }
    giveWriter();
    return true;
}

Session* waiter() {
    Job& j = job();
    if (!j.waiting || j.node == 0xFF) return nullptr;
    Session* s = nullptr;
    struct Find { uint8_t id; Session** out; } f{ j.node, &s };
    Bbs::instance().eachSession([](void* ctx, Session& x) {
        Find* ff = static_cast<Find*>(ctx);
        if (x.id == ff->id && x.loggedIn) *ff->out = &x;
    }, &f);
    if (!s || !Bbs::instance().owns(*s, g_index)) return nullptr;
    return s;
}

void say(Session& s, Color c, const char* text) {
    s.term.color(s.tl, c);
    s.term.text(s.tl, text);
}

bool localNow(struct tm& t) {
    if (!clk::valid()) return false;
    const time_t e = static_cast<time_t>(clk::epoch());
    localtime_r(&e, &t);
    return true;
}

void put16(uint8_t* p, uint16_t v) { p[0] = static_cast<uint8_t>(v); p[1] = static_cast<uint8_t>(v >> 8); }
uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | p[1] << 8); }
uint32_t get32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }

// ---------------------------------------------------------------------------
// To the satellite
// ---------------------------------------------------------------------------
void sendSettings(uint8_t peer) {
    ulink::Engine* e = linkp::engine();
    if (!e) return;
    const uint16_t sess = e->openSession(peer, ulink::FAM_CAMERA);
    if (!sess) return;
    uint8_t b[24] = {};
    put16(b, g_set.tlMin);
    b[2] = g_set.tlSec;
    b[3] = g_set.motion;
    put16(b + 4, g_set.hold);
    b[6] = g_set.size;
    b[7] = g_set.quality;
    b[8] = g_set.flash;
    b[9] = g_set.sleep;
    b[10] = g_set.flip;
    b[11] = g_set.mirror;
    b[12] = static_cast<uint8_t>(g_set.bright);
    b[16] = g_set.wb;
    b[18] = g_set.levels;
    b[19] = g_set.gamma;
    b[20] = g_set.pin;
    // What this board takes (1.2.0): its own choice, whoever owns the
    // satellite. Bytes 0 to 20 count only from the owner.
    b[kSetRecv] = static_cast<uint8_t>(linkp::peerRecv(peer) | RECV_SAID);
    e->send(peer, sess, ulink::FAM_CAMERA, CAM_SETTINGS, b, sizeof(b));
    e->closeAfter(peer, sess);
}

// snapMsg: SNAP with the words the satellite draws and writes.
size_t snapMsg(uint8_t* b, size_t cap, uint16_t req, uint8_t reason, const struct tm& t, const char* who) {
    const SysConfig& c = syscfg::get();
    memset(b, 0, cap);
    put16(b, req);
    b[2] = 0;                                   // the satellite's own size and quality
    b[3] = 0;
    b[4] = 0xFF;                                // and its own flash
    b[5] = reason;
    b[6] = g_set.mark;
    snprintf(reinterpret_cast<char*>(b + 7), 20, "%.19s", c.boardName[0] ? c.boardName : BBS_NAME);
    strftime(reinterpret_cast<char*>(b + 27), 17, "%Y-%m-%d %H:%M", &t);
    snprintf(reinterpret_cast<char*>(b + 43), 24, "%s", who);
    char full[24], com[160];
    strftime(full, sizeof(full), "%Y-%m-%d %H:%M:%S", &t);
    const int n = snprintf(com, sizeof(com), "%s: %s, %s %s. %s %s", c.boardName[0] ? c.boardName : BBS_NAME, full,
                           reason == CR_CALLER ? "snapped by" : "taken by the board,", who, BBS_NAME,
                           BBS_VERSION_SHOWN);
    size_t k = n < 0 ? 0 : static_cast<size_t>(n);
    if (k > 150) k = 150;
    if (k > cap - 67) k = cap - 67;
    memcpy(b + 67, com, k);
    return 67 + k;
}

// ---------------------------------------------------------------------------
// The picture's bytes: the runner's (bulkData, bulkFinish)
// ---------------------------------------------------------------------------
bool bulkBegin(uint8_t peer, uint16_t sess, uint8_t type, uint32_t total) {
    Job& j = job();
    if (type != CAM_PICTURE || j.ph.load() != J_ASKED || peer != j.peer || sess != j.sess) return false;
    if (total <= kPictureHeader || total > ulink::kBulkMax) return false;
    j.headGot = 0;
    j.bytes = 0;
    j.writerOpen = false;
    j.filed.store(false);
    j.rerr[0] = '\0';
    j.dead.store(false);
    // Only from ASKED: the loop may have ended the job since the check above
    // (a CONFIG save, the 30 s timeout), and a plain store would bring it back.
    uint8_t asked = J_ASKED;
    return j.ph.compare_exchange_strong(asked, J_COMING);
}

bool bulkWrite(Job& j, const uint8_t* p, size_t n) {
    while (n && j.headGot < kPictureHeader) { j.head[j.headGot++] = *p++; --n; }
    if (!n) return true;
    if (!j.writerOpen) {
        if (!photos::open(j.w, ".sat0.tmp")) { snprintf(j.rerr, sizeof(j.rerr), "the card would not take the photo"); return false; }
        j.writerOpen = true;
    }
    if (!photos::write(j.w, p, n)) { snprintf(j.rerr, sizeof(j.rerr), "the card would not take the photo"); return false; }
    j.bytes += static_cast<uint32_t>(n);
    return true;
}

bool bulkData(uint8_t, uint16_t sess, const uint8_t* p, size_t n) {
    Job& j = job();
    if (sess != j.sess || j.ph.load() != J_COMING) return false;
    if (!takeWriter()) return false;                   // the loop is closing it
    bool ok = false;
    if (j.dead.load()) {
        if (j.writerOpen) { photos::abandon(j.w); j.writerOpen = false; }
    } else {
        ok = bulkWrite(j, p, n);
        if (j.dead.load() && j.writerOpen) { photos::abandon(j.w); j.writerOpen = false; ok = false; }
    }
    giveWriter();
    return ok;
}

void bulkFinish(uint8_t, uint16_t sess, bool ok) {
    Job& j = job();
    if (sess != j.sess || !takeWriter()) return;
    if (j.writerOpen) {
        j.writerOpen = false;
        if (!ok || j.dead.load()) {
            photos::abandon(j.w);
            if (!ok) snprintf(j.rerr, sizeof(j.rerr), "the picture came damaged");
        } else {
            j.pw = get16(j.head + 4);
            j.ph2 = get16(j.head + 6);
            // Under the next second's name, up to five on, when another camera
            // filed one in the same second (the built-in camera, another
            // satellite's motion picture): 1.2.0's fileAs, which also puts the
            // name it used in j.rel for the offer and the log.
            if (photos::fileAs(j.w, j.rel, sizeof(j.rel), j.kind == K_CALLER ? j.desc : nullptr)) j.filed.store(true);
            else snprintf(j.rerr, sizeof(j.rerr), "a photo with that name is already there, or the card refused it");
        }
    }
    giveWriter();
}

// ---------------------------------------------------------------------------
// The loop's side
// ---------------------------------------------------------------------------
void notifyStaff(uint8_t fromNode) {
    struct Ctx { uint8_t from; } c{ fromNode };
    Bbs::instance().eachSession([](void* ctx, Session& x) {
        const Ctx* cc = static_cast<const Ctx*>(ctx);
        if (!x.loggedIn || x.id == cc->from || !plugins::mayUse(x, PlugLevel::Staff)) return;
        char msg[48];
        snprintf(msg, sizeof(msg), "Node %u took a photo.", static_cast<unsigned>(cc->from));
        Bbs::instance().notify(x, msg);
    }, &c);
}

// finish: the picture is filed or not; tell whoever is waiting, go idle.
void finish(bool ok) {
    Job& j = job();
    if (!dropWriter()) { j.dying = true; j.dyingOk = ok; return; }   // the runner's: next tick
    j.dying = false;
    // The runner may have filed it before the loop failed the job (a reset
    // or a CONFIG save in the pass between bulkFinish and bulkEnd): it is in
    // Photos and counted, so say so.
    ok = ok || j.filed.load();
    const char* why = j.err[0] ? j.err : j.rerr;
    ulink::Engine* e = linkp::engine();
    if (e) {                                   // ours or the satellite's (an EVENT): let it go
        if (g_closing.on) e->closeAfter(g_closing.peer, g_closing.sess);   // the one before, now
        g_closing = Closing{ true, j.peer, j.sess, plat::millis() + 3000 };
    }
    if (ok) {
        snprintf(g_last, sizeof(g_last), "%.111s", j.rel);
        snprintf(g_lastBy, sizeof(g_lastBy), "%.21s", j.kind == K_CALLER ? j.handle : "the board");
        if (j.peer < ulink::Engine::kPeers) { g_sat[j.peer].pictures++; g_sat[j.peer].lastAt = clk::epoch(); }
        plat::log("camsat: %s %ux%u %u bytes from \"%s\" in %u ms", j.rel, static_cast<unsigned>(j.pw),
                  static_cast<unsigned>(j.ph2), static_cast<unsigned>(j.bytes), linkp::peerName(j.peer),
                  static_cast<unsigned>(plat::millis() - j.startedAt));
    } else {
        plat::log("camsat: %s failed: %s", j.rel[0] ? j.rel : "a picture", why[0] ? why : "no reason given");
    }
    Session* s = waiter();
    const uint8_t kind = j.kind, node = j.node;
    j.node = 0xFF;
    j.waiting = false;
    j.ph.store(J_IDLE);
    if (kind == K_CALLER && ok) notifyStaff(node);
    if (!s) return;

    Bbs& b = Bbs::instance();
    s->term.left(s->tl, 1);
    s->term.text(s->tl, " ");
    s->term.cursor(s->tl, true);
    s->term.nl(s->tl);
    char buf[160];
    const uint8_t busyAt = j.busyAt;
    j.busyAt = 0;
    if (!ok && busyAt) {
        const char* nm = j.peer < ulink::Engine::kPeers ? g_sat[j.peer].name : "satellite";
        const bool wide = b.rowWidth(*s) >= 60;
        if (busyAt == 0xFF)
            snprintf(buf, sizeof(buf), wide ? "The %s camera is busy and its queue is full. Try again in a moment."
                                            : "The %s camera's queue is full.\n    Try again in a moment.", nm);
        else if (busyAt == 1)
            snprintf(buf, sizeof(buf), wide ? "The %s camera is busy. Try again in a moment."
                                            : "The %s camera is busy.\n    Try again in a moment.", nm);
        else
            snprintf(buf, sizeof(buf), wide ? "The %s camera is busy, %u ahead of you. Try again in a moment."
                                            : "The %s camera is busy,\n    %u ahead of you: try in a moment.",
                     nm, static_cast<unsigned>(busyAt - 1));
        // Two lines at 40, each whole (a wrap would break "2 ahead / of you").
        char* nl = strchr(buf, '\n');
        if (nl) *nl = '\0';
        // The board's voice, "--> ", as the built-in camera's "in use" line.
        s->term.color(s->tl, Color::Cyan);
        s->term.text(s->tl, "--> ");
        say(*s, Color::Yellow, buf);
        if (nl) { s->term.nl(s->tl); say(*s, Color::Yellow, nl + 1); }
        b.release(*s);
        return;
    }
    if (!ok) {
        snprintf(buf, sizeof(buf), "No photo: %.71s.", why[0] ? why : "the satellite gave none");
        say(*s, Color::LightRed, buf);
        b.release(*s);
        return;
    }
    snprintf(buf, sizeof(buf), "Photo saved: %.111s (FILES, area 12)", j.rel);
    say(*s, Color::LightGreen, buf);
    s->term.nl(s->tl);
    const uint8_t fi = plugins::indexOf("files");
    const bool filesOn = fi != 0xFF && plugins::running(fi);
    const camrules::Offer o = camrules::offerFor(true, true, filesOn && plugins::mayUse(*s, g_set.photos),
                                                 claims::held(claims::Res::Transfer));
    if (o == camrules::Offer::Ask && g_offer && s->id < kSlots) {
        snprintf(g_offer[s->id].rel, sizeof(g_offer[0].rel), "%.111s", j.rel);
        say(*s, Color::Cyan, "Download it now?  [Y]es  [X]modem  [N]o ");
        s->term.color(s->tl, Color::White);
        s->ownerData = 1;
        return;
    }
    if (o == camrules::Offer::Busy) say(*s, Color::Grey, "Somebody is transferring right now: it is in FILES, area 12.");
    else                            say(*s, Color::Grey, "It is kept in the Photos area.");
    b.release(*s);
}

void bulkEnd(uint8_t peer, uint16_t sess, bool) {
    Job& j = job();
    if (peer != j.peer || sess != j.sess || j.ph.load() != J_COMING) return;
    // bulkFinish has run on the runner by now: it filed the picture or said why not.
    finish(j.filed.load());
}

void failWith(const char* why) {
    snprintf(job().err, sizeof(job().err), "%s", why);
    finish(false);
}

bool message(uint8_t peer, uint16_t sess, uint8_t type, const uint8_t* p, size_t n);
void reconcile();

// startSystem: a picture the board takes for itself (timelapse, motion),
// on a session this board opens, or on the satellite's own (an EVENT).
bool startSystem(int peer, uint16_t sess, uint8_t reason) {
    ulink::Engine* e = linkp::engine();
    struct tm t;
    if (!e || busy() || peer < 0 || !plat::sdBase()[0] || !localNow(t)) return false;
    Job& j = job();
    const char* folder = reason == CR_MOTION ? camrules::kMotionFolder : camrules::kTlFolder;
    const char* prefix = reason == CR_MOTION ? "MO" : camrules::kTlPrefix;
    if (!camrules::systemName(folder, prefix, t, j.rel, sizeof(j.rel))) return false;
    j.ours = !sess;
    if (!sess) sess = e->openSession(static_cast<uint8_t>(peer), ulink::FAM_CAMERA);
    if (!sess) return false;
    uint8_t b[ulink::kPayloadMax];
    j.req = g_reqNext++;
    const size_t len = snapMsg(b, sizeof(b), j.req, reason, t, folder);
    if (e->send(static_cast<uint8_t>(peer), sess, ulink::FAM_CAMERA, CAM_SNAP, b, len) != 1) {
        if (j.ours) e->closeSession(static_cast<uint8_t>(peer), sess);
        return false;
    }
    j.kind = K_SYSTEM;
    j.peer = static_cast<uint8_t>(peer);
    j.sess = sess;
    j.node = 0xFF;
    j.waiting = false;
    j.err[0] = '\0';
    j.rerr[0] = '\0';
    j.filed.store(false);
    snprintf(j.handle, sizeof(j.handle), "%s", folder);
    j.startedAt = plat::millis();
    j.ph.store(J_ASKED);
    return true;
}

bool message(uint8_t peer, uint16_t sess, uint8_t type, const uint8_t* p, size_t n) {
    ulink::Engine* e = linkp::engine();
    if (!e || e->peerKind(peer) != ulink::KIND_CAMSAT || peer >= ulink::Engine::kPeers) return true;
    Sat& s = g_sat[peer];
    switch (type) {
        case CAM_STATUS:
            if (n >= 28) {
                s.known = true;
                s.heap = get32(p + 2);
                s.psram = get32(p + 6);
                s.uptime = get32(p + 10);
                s.lastErr = p[14];
                snprintf(s.sensor, sizeof(s.sensor), "%.11s", reinterpret_cast<const char*>(p + 16));
            }
            e->closeAfter(peer, sess);                 // one message a session: the table holds 16
            return true;
        case CAM_SNAP_FAIL:
            if (n >= 3 && peer == job().peer && sess == job().sess && job().ph.load() == J_ASKED) {
                static const char* const kWhy[] = { "the satellite failed", "the satellite has no camera",
                                                    "the satellite is out of memory", "the satellite is busy",
                                                    "the satellite's flash failed", "the satellite's camera gave no picture" };
                // Busy (1.2.0): the requests ahead in its queue, or full.
                if (p[2] == CE_BUSY) job().busyAt = n >= 4 ? (p[3] == kQueueFull ? 0xFF : static_cast<uint8_t>(p[3] + 1)) : 1;
                failWith(kWhy[p[2] < 6 ? p[2] : 0]);
            }
            return true;
        case CAM_EVENT: {
            // The satellite asks for a picture: motion, or its own timer.
            const uint8_t reason = n && p[0] == CEV_MOTION ? CR_MOTION : CR_TIMELAPSE;
            const uint32_t now = plat::millis();
            const bool tooSoon = reason == CR_MOTION && s.eventAt && now - s.eventAt < 10000;
            if (!tooSoon && startSystem(peer, sess, reason)) s.eventAt = now;
            else e->closeAfter(peer, sess);
            return true;
        }
        case CAM_SETTINGS_OK:
            // A satellite from 1.2.0 says whether this board owns it and that
            // it keeps its own timelapse clock; one before says neither, and
            // is this board's alone, on this board's clock.
            if (n >= 23) {
                s.owned = (p[kSetOwner] & SO_OWNER) || !(p[kSetOwner] & SO_EVENTS);
                s.events = (p[kSetOwner] & SO_EVENTS) != 0;
            }
            if (n >= 21) {
                s.tlMin = get16(p + kSetTimelapseMin);
                s.tlSec = p[kSetTimelapseSec];
                s.motion = p[kSetMotion] != 0;
                s.hold = get16(p + kSetHoldoff);
                s.sleeps = p[kSetSleep] != 0;
            }
            e->closeAfter(peer, sess);
            return true;
        default:
            return true;
    }
}

void reset(uint8_t peer, uint16_t sess, uint8_t reason) {
    if (peer != job().peer || sess != job().sess || !busy() || job().dying) return;
    char why[48];
    snprintf(why, sizeof(why), "the link to the satellite dropped (%u)", static_cast<unsigned>(reason));
    failWith(why);
}

void peerState(uint8_t peer, bool up) {
    ulink::Engine* e = linkp::engine();
    if (!e || e->peerKind(peer) != ulink::KIND_CAMSAT) return;
    if (up && g_running) sendSettings(peer);
    if (!up && busy() && job().peer == peer && job().ph.load() == J_ASKED) failWith("the satellite went quiet");
}

const linkp::Family kFamily = [] {
    linkp::Family f;
    f.id = ulink::FAM_CAMERA;
    f.name = "camera";
    f.message = message;
    f.bulkBegin = bulkBegin;
    f.bulkData = bulkData;
    f.bulkEnd = bulkEnd;
    f.reset = reset;
    f.peerState = peerState;
    f.bulkFinish = bulkFinish;
    // CONFIG sats changed what this board takes, or a satellite's number:
    // the satellite is told now, and the camera list follows (1.2.0).
    f.settingsChanged = [](uint8_t peer) {
        ulink::Engine* e = linkp::engine();
        if (g_running && e && e->peerUp(peer) && e->peerKind(peer) == ulink::KIND_CAMSAT) sendSettings(peer);
        reconcile();
    };
    return f;
}();

// ---------------------------------------------------------------------------
// tick: every 20 ms (PF_FAST). The spinner, the timeouts, the timelapse.
// ---------------------------------------------------------------------------
void tick(uint32_t now) {
    static uint32_t listedAt = 0;
    if (static_cast<int32_t>(now - listedAt) >= 1000) { listedAt = now; reconcile(); }
    Job& j = job();
    if (runner::done(j.closer)) runner::collect(j.closer);   // a given-up part-file is gone
    if (g_closing.on && static_cast<int32_t>(now - g_closing.at) >= 0) {
        if (ulink::Engine* e = linkp::engine()) e->closeAfter(g_closing.peer, g_closing.sess);
        g_closing.on = false;
    }
    // A finish the runner held off: again.
    if (j.dying) { finish(j.dyingOk); return; }
    const uint8_t ph = j.ph.load();
    if (ph == J_IDLE) {
        const uint32_t every = tlEvery();
        struct tm t;
        if (every && !g_set.sleep && localNow(t)) {
            const uint32_t local = static_cast<uint32_t>(t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec) +
                                   static_cast<uint32_t>(t.tm_yday) * camrules::kDay;
            if (camrules::tlDue(local, every, g_tlSlot, g_tlPrimed)) {
                const int p = firstUp(true);
                if (p >= 0) startSystem(p, 0, CR_TIMELAPSE);
            }
        }
        return;
    }
    // Not for ever: a satellite that went away mid-picture. Signed: a job
    // started from the link's message in this same pass has a start later
    // than this tick's now (the bench: every EVENT failed at once).
    const int32_t age = static_cast<int32_t>(now - j.startedAt);
    // 60 s: a satellite shared by several boards (1.2.0) queues a SNAP
    // behind the other boards' pictures, a few seconds each.
    if (ph == J_ASKED && age > 60000) { failWith("the satellite did not answer"); return; }
    if (ph == J_COMING && age > 120000) {
        const uint8_t peer = j.peer;
        const uint16_t sess = j.sess;
        failWith("the picture took too long to come");   // first: the reset below then finds it ended
        if (ulink::Engine* e = linkp::engine()) e->resetSession(peer, sess, ulink::R_CLOSED);
        return;
    }
    Session* s = waiter();
    if (s && static_cast<int32_t>(now - j.spinAt) >= 0 && s->tl.empty()) {
        s->term.left(s->tl, 1);
        s->term.color(s->tl, Color::Yellow);
        fx::spinFrame(s->term, s->tl, fx::Spin::Line, ++j.spin);
        j.spinAt = now + 150;
    }
}

// ---------------------------------------------------------------------------
// SNAPSHOT: a picture from one satellite, as the core's SNAPSHOT [n|name]
// picks it (photos::Camera::snap).
// ---------------------------------------------------------------------------
void refuse(Bbs& b, Session& s, const char* why) {
    say(s, Color::LightRed, why);
    b.prompt(s);
}

void snapFrom(Bbs& b, Session& s, int peer, uint32_t now) {
    ulink::Engine* e = linkp::engine();
    if (!g_running || !plat::sdBase()[0]) { refuse(b, s, "The camera needs the SD card in."); return; }
    if (!e) { refuse(b, s, "The camera satellite needs the link: LINK."); return; }
    if (!plugins::mayUse(s, g_set.snap)) { refuse(b, s, "Taking photos is not open to you here."); return; }
    if (!clk::valid()) { refuse(b, s, "The board's clock is not set yet, and a photo is named by it."); return; }
    if (!peerUp(peer)) { refuse(b, s, "That camera is not answering. Try again in a moment."); return; }
    const uint32_t epoch = clk::epoch();
    const bool sysop = plugins::mayUse(s, PlugLevel::Sysop);
    photos::Budget v;
    if (!sysop) {
        v = photos::budget(s, epoch);
        if (!v.ok) {
            char at[8], buf[96];
            clk::fmtEpoch(at, sizeof(at), "%H:%M", v.nextAt);
            snprintf(buf, sizeof(buf), "That is %u %s; the next one is allowed at %s.",
                     static_cast<unsigned>(v.byDay ? v.perDay : v.perHour),
                     v.byDay ? "today" : "this hour", at);
            refuse(b, s, buf);
            return;
        }
    }
    if (busy()) { refuse(b, s, "The camera is busy. Try again in a moment."); return; }
    struct tm t;
    localNow(t);
    Job& j = job();
    if (!camrules::callerName(g_set.names, t, s.user, s.guest, j.rel, sizeof(j.rel))) {
        refuse(b, s, "That photo's name would be too long.");
        return;
    }
    snprintf(j.handle, sizeof(j.handle), "%s%s", s.guest ? "*" : "", s.user);
    snprintf(j.desc, sizeof(j.desc), "Taken by %.30s", j.handle);
    const uint16_t sess = e->openSession(static_cast<uint8_t>(peer), ulink::FAM_CAMERA);
    if (!sess) { refuse(b, s, "The camera is busy. Try again in a moment."); return; }
    uint8_t m[ulink::kPayloadMax];
    j.req = g_reqNext++;
    const size_t len = snapMsg(m, sizeof(m), j.req, CR_CALLER, t, j.handle);
    if (e->send(static_cast<uint8_t>(peer), sess, ulink::FAM_CAMERA, CAM_SNAP, m, len) != 1) {
        e->closeSession(static_cast<uint8_t>(peer), sess);
        refuse(b, s, "The camera is busy. Try again in a moment.");
        return;
    }
    if (!b.own(s, g_index)) { e->closeSession(static_cast<uint8_t>(peer), sess); b.prompt(s); return; }
    b.setDoing(s, "SNAPSHOT");
    s.ownerData = 0;
    j.kind = K_CALLER;
    j.peer = static_cast<uint8_t>(peer);
    j.sess = sess;
    j.ours = true;
    j.busyAt = 0;
    j.node = s.id;
    j.waiting = true;
    j.err[0] = '\0';
    j.rerr[0] = '\0';
    j.filed.store(false);
    j.startedAt = now;
    j.spinAt = now;
    j.ph.store(J_ASKED);
    if (!sysop) {
        photos::spend(s, epoch);
        char buf[80];
        snprintf(buf, sizeof(buf), "Snapshot %u of %u this hour, %u of %u today.",
                 static_cast<unsigned>(v.hour + 1), static_cast<unsigned>(v.perHour),
                 static_cast<unsigned>(v.day + 1), static_cast<unsigned>(v.perDay));
        say(s, Color::Grey, buf);
        s.term.nl(s.tl);
    }
    s.term.cursor(s.tl, false);
    s.term.color(s.tl, Color::Yellow);
    fx::spinFrame(s.term, s.tl, fx::Spin::Line, 0);
}

// onKey: the download question, and nothing else.
void onKey(Session& s, int k, uint32_t now) {
    Bbs& b = Bbs::instance();
    if (s.ownerData != 1) return;
    s.ownerData = 0;
    s.term.cursor(s.tl, true);
    s.term.nl(s.tl);
    if (k == 'y' || k == 'Y' || k == 'x' || k == 'X') {
        char rel[112] = "";
        if (g_offer && s.id < kSlots) snprintf(rel, sizeof(rel), "%s", g_offer[s.id].rel);
        b.release(s);
        if (!files::sendPhoto(b, s, rel, k == 'x' || k == 'X', now)) b.prompt(s);
        return;
    }
    say(s, Color::Grey, "It is kept in the Photos area.");
    b.release(s);
}

void onLogoff(Session& s) {
    if (g_jobp && g_jobp->node == s.id) g_jobp->waiting = false;       // the photo is still filed and counted
}

// ---------------------------------------------------------------------------
// Each satellite in the board's camera list (photos::Camera). ctx is its Sat.
// ---------------------------------------------------------------------------
bool satUp(void* ctx) { return g_running && peerUp(static_cast<Sat*>(ctx)->peer); }

bool satBusy(void* ctx) {
    return busy() && g_jobp && g_jobp->peer == static_cast<Sat*>(ctx)->peer;
}

void satSnap(void* ctx, Bbs& b, Session& s, uint32_t now) {
    snapFrom(b, s, static_cast<Sat*>(ctx)->peer, now);
}

// For SATS (1.2.0): who may see it, and what it says of itself.
void satLevels(void*, PlugLevel& see, PlugLevel& snap) {
    see = g_set.photos;
    snap = g_set.snap;
}

bool satFacts(void* ctx, photos::CamFacts& f) {
    const Sat& x = *static_cast<Sat*>(ctx);
    const bool up = peerUp(x.peer);
    // Asleep only while it has been heard within twice its timelapse and 5
    // minutes (a day for one that only wakes on motion): a sleeper whose
    // battery died is not answering, not asleep for ever (code review).
    bool dozing = false;
    if (!up && x.sleeps) {
        ulink::Engine* e = linkp::engine();
        const uint32_t heard = e ? e->peerStats(x.peer).lastHeard : 0;
        const uint32_t tl = (static_cast<uint32_t>(x.tlMin) * 60u + x.tlSec) * 1000u;
        const uint32_t grace = tl ? 2u * tl + 300000u : 86400000u;
        dozing = heard && plat::millis() - heard < grace;
    }
    f.state = busy() && job().peer == x.peer ? photos::CST_BUSY
            : up ? photos::CST_AWAKE : dozing ? photos::CST_ASLEEP : photos::CST_NOANSWER;
    f.lastAt = x.lastAt;
    f.pictures = x.pictures;
    f.uptime = up ? x.uptime : 0;
    snprintf(f.sensor, sizeof(f.sensor), "%s", x.sensor);
    f.sleeps = x.sleeps;
    f.tlMin = x.tlMin;
    f.tlSec = x.tlSec;
    f.motion = x.motion;
    f.hold = x.hold;
    return true;
}

// satLine: CAMERA's list line: the sensor, the signal, the pictures taken.
void satLine(void* ctx, char* out, size_t n) {
    const Sat& x = *static_cast<Sat*>(ctx);
    ulink::Engine* e = linkp::engine();
    if (!e || !e->peerUp(x.peer)) { snprintf(out, n, "satellite, not answering"); return; }
    snprintf(out, n, "satellite, %s, %d dBm, %lu photo%s", x.sensor[0] ? x.sensor : "sensor not said",
             static_cast<int>(e->peerStats(x.peer).rssi), static_cast<unsigned long>(x.pictures),
             x.pictures == 1 ? "" : "s");
}

// satCommand: CAMERA <this satellite>: what the board knows of it, for staff.
void satCommand(void* ctx, Bbs& b, Session& s, const char*, uint32_t) {
    if (!plugins::mayUse(s, plugins::levelFor(g_index, 1))) {
        refuse(b, s, "The camera's details are for staff.");
        return;
    }
    const Sat& x = *static_cast<Sat*>(ctx);
    ulink::Engine* e = linkp::engine();
    char line[160];
    b.rowTitle(s, x.name);
    if (!e) {
        b.rowText(s, Color::Grey, "The link is off: CONFIG link.");
        b.rowRule(s);
        b.prompt(s);
        return;
    }
    const ulink::PeerStats& st = e->peerStats(x.peer);
    snprintf(line, sizeof(line), "Link      %s, %d dBm here, %d dBm there", e->peerUp(x.peer) ? "up" : "quiet",
             static_cast<int>(st.rssi), static_cast<int>(st.farRssi));
    b.rowText(s, Color::White, line);
    snprintf(line, sizeof(line), "Sensor    %s", x.sensor[0] ? x.sensor : "not said yet");
    b.rowText(s, Color::White, line);
    if (x.known) {
        snprintf(line, sizeof(line), "Memory    %lu KB internal, %lu KB PSRAM free",
                 static_cast<unsigned long>(x.heap / 1024), static_cast<unsigned long>(x.psram / 1024));
        b.rowText(s, Color::White, line);
        snprintf(line, sizeof(line), "Up        %lu min", static_cast<unsigned long>(x.uptime / 60));
        b.rowText(s, Color::White, line);
    }
    snprintf(line, sizeof(line), "Photos    %lu since the board started", static_cast<unsigned long>(x.pictures));
    b.rowText(s, Color::White, line);
    if (g_last[0]) {
        snprintf(line, sizeof(line), "Last      %.100s by %s", g_last, g_lastBy);
        b.rowText(s, Color::Grey, line);
    }
    b.rowRule(s);
    b.prompt(s);
}

// listed: a satellite is in the camera list while it is paired, under the
// name LINK gives it. Checked a second at a time from tick, so a pairing, a
// LINK NAME and a LINK FORGET each follow without a hook of their own.
void photosFollow(bool any);

void reconcile() {
    if (!g_sat) return;
    ulink::Engine* e = linkp::engine();
    for (uint8_t p = 0; p < ulink::Engine::kPeers; ++p) {
        Sat& x = g_sat[p];
        const bool want = g_running && e && e->peerUsed(p) && e->peerKind(p) == ulink::KIND_CAMSAT;
        const char* name = want ? linkp::peerName(p) : "";
        if (x.listed && (!want || strcmp(x.name, name ? name : ""))) {
            photos::removeCamera(x.cam);
            x.listed = false;
        }
        // Its camera number (CONFIG sats, 1.2.0) followed without re-listing.
        if (x.listed && x.cam.number != linkp::peerCamNo(p)) {
            x.cam.number = linkp::peerCamNo(p);
            photos::renumber();
        }
        if (want && !x.listed) {
            x.peer = p;
            snprintf(x.name, sizeof(x.name), "%s", name && *name ? name : "satellite");
            x.cam = photos::Camera{ x.name, static_cast<uint8_t>(1 + p), &x, satUp, satBusy, satSnap, satLine,
                                    satCommand };
            x.cam.number = linkp::peerCamNo(p);
            x.cam.pairing = static_cast<int8_t>(p);
            x.cam.levels = satLevels;
            x.cam.facts = satFacts;
            x.listed = photos::addCamera(x.cam);
            if (!x.listed) plat::log("camsat: the board's camera list is full; \"%s\" is not in it", x.name);
        }
    }
    bool any = false;
    for (uint8_t p = 0; p < ulink::Engine::kPeers; ++p) any = any || g_sat[p].listed;
    photosFollow(any);
}

void unlistAll() {
    if (!g_sat) return;
    for (uint8_t p = 0; p < ulink::Engine::kPeers; ++p)
        if (g_sat[p].listed) { photos::removeCamera(g_sat[p].cam); g_sat[p].listed = false; }
}

// ---------------------------------------------------------------------------
// CONFIG camsat
// ---------------------------------------------------------------------------
const PluginSetting kConfig[] = {
    { "snap",      "Snap",      PS_CYCLE, 0, 0, 6, "Who may take a photo.", kLevels, "Who may take a photo" },
    { "photos",    "Photos",    PS_CYCLE, 0, 0, 6, "Who may see and download photos.", kLevels, "Who may see photos" },
    { "size",      "Size",      PS_CYCLE, 0, 0, 5, "The satellite's picture size.", kSizes, "Resolution" },
    { "quality",   "Quality",   PS_NUM,   4, 40, 2, "Lower: sharper, bigger. 10-12 is good.", nullptr, "JPEG quality" },
    { "names",     "Names",     PS_CYCLE, 0, 0, 12, "SNAP-date, with the handle, or by handle.", camrules::kSchemes,
      "Name snaps" },
    { "watermark", "Watermark", PS_YESNO, 0, 0, 3, "Board, date and who, in a corner.", nullptr, "Watermark" },
    { "flash",     "Flash",     PS_CYCLE, 0, 0, 4, "The white LED: auto lights it in the dark.", kFlashes, "Flash" },
    { "sleep",     "Sleep",     PS_YESNO, 0, 0, 3, "Deep sleep between photos (battery).", nullptr,
      "Sleep between photos", "The satellite sleeps until its timer or its motion sensor: no photo on request." },
    { "tl",        "Timelapse", PS_PAGE,  0, 0, 16, "A photo every so often.", nullptr, "Timelapse" },
    { "motion",    "Motion",    PS_PAGE,  0, 0, 16, "A photo when the sensor sees movement.", nullptr, "Motion sensor" },
    { "pic",       "Picture",   PS_PAGE,  0, 0, 12, "Levels, gamma, white balance, turn.", nullptr, "Picture settings" },

    { "tl_min",    "Every min", PS_NUM,   0, 1440, 4, "Minutes; 1440 is a day. 0 0 is off.", nullptr, "Every (minutes)" },
    { "tl_sec",    "and sec",   PS_NUM,   0, 59, 2, "Seconds; the least is 10 in all.", nullptr, "and seconds" },

    { "motion_on",   "Sensor",  PS_YESNO, 0, 0, 3, "A PIR's output on the satellite.", nullptr, "Motion sensor" },
    { "motion_hold", "Hold s",  PS_NUM,  10, 3600, 4, "Seconds between motion photos.", nullptr, "Hold-off (seconds)" },
    { "motion_pin",  "Pin",     PS_CYCLE, 0, 0, 2, "The satellite's GPIO for the PIR.", kPins, "Satellite pin" },

    { "pic_flip",   "Flip",     PS_YESNO, 0, 0, 3, nullptr, nullptr, "Upside down" },
    { "pic_mirror", "Mirror",   PS_YESNO, 0, 0, 3, nullptr, nullptr, "Mirror" },
    { "pic_bright", "Bright",   PS_NUM,  -2, 2, 2, "-2 to 2.", nullptr, "Brightness" },
    { "pic_wb",     "White",    PS_CYCLE, 0, 0, 7, nullptr, kWbs, "White balance" },
    { "pic_levels", "Levels",   PS_YESNO, 0, 0, 3, "Stretch to full black and white.", nullptr, "Auto levels" },
    { "pic_gamma",  "Gamma",    PS_CYCLE, 0, 0, 3, "1.0 none; lower darkens the middle.", kGammas, "Gamma" },
};

void setting(const char* key, char* out, size_t n) {
    char w[16];
    const Settings& g = g_set;
    if      (!strcmp(key, "snap"))        snprintf(out, n, "%s", plugins::levelName(g.snap));
    else if (!strcmp(key, "photos"))      snprintf(out, n, "%s", plugins::levelName(g.photos));
    else if (!strcmp(key, "size"))        { wordAt(kSizes, g.size - 1, w, sizeof(w)); snprintf(out, n, "%s", w); }
    else if (!strcmp(key, "quality"))     snprintf(out, n, "%u", static_cast<unsigned>(g.quality));
    else if (!strcmp(key, "names"))       { wordAt(camrules::kSchemes, g.names, w, sizeof(w)); snprintf(out, n, "%s", w); }
    else if (!strcmp(key, "watermark"))   snprintf(out, n, "%s", g.mark ? "yes" : "no");
    else if (!strcmp(key, "flash"))       { wordAt(kFlashes, g.flash, w, sizeof(w)); snprintf(out, n, "%s", w); }
    else if (!strcmp(key, "sleep"))       snprintf(out, n, "%s", g.sleep ? "yes" : "no");
    else if (!strcmp(key, "tl"))          { if (tlEvery()) snprintf(out, n, "every %u s", static_cast<unsigned>(tlEvery())); else snprintf(out, n, "off"); }
    else if (!strcmp(key, "motion"))      snprintf(out, n, "%s", g.motion ? "on" : "off");
    else if (!strcmp(key, "pic"))         snprintf(out, n, "%s", (g.flip || g.mirror || g.bright || g.wb || !g.levels || g.gamma != 4)
                                                                   ? "adjusted" : "as it comes");
    else if (!strcmp(key, "tl_min"))      snprintf(out, n, "%u", static_cast<unsigned>(g.tlMin));
    else if (!strcmp(key, "tl_sec"))      snprintf(out, n, "%u", static_cast<unsigned>(g.tlSec));
    else if (!strcmp(key, "motion_on"))   snprintf(out, n, "%s", g.motion ? "yes" : "no");
    else if (!strcmp(key, "motion_hold")) snprintf(out, n, "%u", static_cast<unsigned>(g.hold));
    else if (!strcmp(key, "motion_pin"))  snprintf(out, n, "%u", static_cast<unsigned>(g.pin));
    else if (!strcmp(key, "pic_flip"))    snprintf(out, n, "%s", g.flip ? "yes" : "no");
    else if (!strcmp(key, "pic_mirror"))  snprintf(out, n, "%s", g.mirror ? "yes" : "no");
    else if (!strcmp(key, "pic_bright"))  snprintf(out, n, "%d", g.bright);
    else if (!strcmp(key, "pic_wb"))      { wordAt(kWbs, g.wb, w, sizeof(w)); snprintf(out, n, "%s", w); }
    else if (!strcmp(key, "pic_levels"))  snprintf(out, n, "%s", g.levels ? "yes" : "no");
    else if (!strcmp(key, "pic_gamma"))   { wordAt(kGammas, g.gamma, w, sizeof(w)); snprintf(out, n, "%s", w); }
}

// ---------------------------------------------------------------------------
// start / stop
// ---------------------------------------------------------------------------
// Photos is offered on a board with no camera of its own only while a
// satellite is paired to fill it (photos::provide, 1.2.0).
bool g_provided = false;
bool providing() { return g_running && g_provided; }
void photoLevels(PlugLevel& see, PlugLevel& removeLevel) {
    see = g_set.photos;
    removeLevel = g_index != 0xFF ? plugins::levelFor(g_index, 2) : PlugLevel::Sysop;
}
const photos::Provider kProvider = { kName, providing, photoLevels };

void photosFollow(bool any) {
    if (any && !g_provided) g_provided = photos::provide(kProvider);
    else if (!any && g_provided) { photos::withdraw(kProvider); g_provided = false; }
}

bool start(Bbs&) {
    g_index = plugins::indexOf(kName);
    g_set = Settings();
    plugins::forEachKey(g_index, readKey, nullptr);
    if (!g_jobp) {                                         // once, and kept: a CONFIG save restarts plugins
        g_jobp = new (std::nothrow) Job();
        g_sat = new (std::nothrow) Sat[ulink::Engine::kPeers]();
        if (!g_jobp || !g_sat) {
            delete g_jobp;
            delete[] g_sat;
            g_jobp = nullptr;
            g_sat = nullptr;
            plat::log("camsat: not enough memory to start");
            return false;
        }
    }
    if (!linkp::registerFamily(kFamily)) {
        plat::log("camsat: the camera family is taken (another camera plugin?)");
        return false;
    }
    if (!g_offer) g_offer = static_cast<Offer*>(calloc(kSlots, sizeof(Offer)));
    g_tlPrimed = false;
    g_running = true;
    reconcile();                                           // SNAPSHOT and CAMERA reach them
    // Satellites already up get the settings now (a CONFIG save).
    for (uint8_t i = 0; i < ulink::Engine::kPeers; ++i) {
        const int p = satPeer(i);
        if (p < 0) break;
        if (peerUp(p)) sendSettings(static_cast<uint8_t>(p));
    }
    plat::log("camsat: on, %s", linkp::engine() ? "the link is up" : "waiting for the link");
    return true;
}

void stop() {
    g_running = false;
    unlistAll();
    if (busy()) failWith("the camera was switched off");
    photosFollow(false);
    linkp::unregisterFamily(ulink::FAM_CAMERA);
}

const char* status() {
    static char line[48];
    ulink::Engine* e = linkp::engine();
    if (!e) return "waiting for the link";
    if (busy()) return "taking a photo";
    int paired = 0, up = 0;
    for (uint8_t i = 0; i < ulink::Engine::kPeers; ++i) {
        const int p = satPeer(i);
        if (p < 0) break;
        ++paired;
        if (e->peerUp(static_cast<uint8_t>(p))) ++up;
    }
    snprintf(line, sizeof(line), "%d satellite%s, %d up", paired, paired == 1 ? "" : "s", up);
    return line;
}

}  // namespace

extern const Plugin kCamsatPlugin = {
    { kName, "Camera satellite", "1.1.0", 4096, 0, PF_SD | PF_FAST,
      PlugLevel::All, PlugLevel::Staff, PlugLevel::Sysop },
    start,
    stop,
    tick,
    nullptr,                 // onConnect
    nullptr,                 // onLogin
    onLogoff,
    onKey,
    status,
    nullptr,                 // commands: SNAPSHOT and CAMERA are the core's
    0,
    kConfig,
    sizeof(kConfig) / sizeof(kConfig[0]),
    setting,
    nullptr,                 // rows
    nullptr,                 // onPresence
    nullptr,                 // onBytes
    nullptr,                 // onRename: the core carries a caller's budget
    nullptr,                 // listDone
};
