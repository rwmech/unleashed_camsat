// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/satsched.h
// Module:       The satellite / who gets which picture (1.2.0)
//
// Purpose:      One satellite, several boards (the core's LINK.md, "One
//               satellite, several boards"; its design record
//               internal/link-multiboard-2026-09-27.md). Pure logic, no
//               ESP-IDF, so the host tests it (firmware/test/test_sched.cpp):
//
//                 Queue  a board's SNAP waits here while the camera works:
//                        two a board and eight in all, served round-robin by
//                        board so one busy board cannot starve the rest. A
//                        SNAP past either limit is refused with its place:
//                        the requests ahead, or kFull.
//                 Group  a timelapse or motion picture. The satellite sends
//                        EVENT to every board that wants that kind; each
//                        answers with a SNAP. The group collects them for
//                        kCollectMs after the first, then the picture is
//                        taken once and sent to each, the owner's texts on
//                        it when the owner is among them.
//
//               Not sched.h: that is the C library's, and pthread.h includes
//               it, so a header of that name here shadowed it.
//
//               A request carries the SNAP's bytes as they came, so the
//               caller parses them the way it always has.
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
#include <cstring>

namespace sched {

constexpr uint8_t  kBoards    = 5;       // the link's kHosts
constexpr uint8_t  kPerBoard  = 2;
constexpr uint8_t  kQueue     = 8;
constexpr uint8_t  kFull      = 0xFF;    // the busy answer's place when the queue is full
constexpr uint32_t kCollectMs = 300;     // SNAPs for one EVENT, after the first
constexpr uint32_t kEventMs   = 10000;   // an EVENT nobody answered is let go
constexpr uint8_t  kSnapMax   = 222;     // one frame

inline bool reached(uint32_t now, uint32_t t) { return static_cast<int32_t>(now - t) >= 0; }

// One SNAP, as it came: which board, its session, and its bytes.
struct Req {
    bool     used = false;
    uint8_t  board = 0;
    uint16_t sess = 0;
    uint16_t req = 0;           // bytes 0-1, for SNAP_FAIL and the picture header
    uint8_t  reason = 0;        // byte 5
    uint32_t seq = 0;           // arrival order
    uint8_t  n = 0;
    uint8_t  p[kSnapMax] = {};

    void set(uint8_t b, uint16_t s, const uint8_t* bytes, size_t len) {
        used = true;
        board = b;
        sess = s;
        n = static_cast<uint8_t>(len < kSnapMax ? len : kSnapMax);
        memcpy(p, bytes, n);
        req = n >= 2 ? static_cast<uint16_t>(p[0] | p[1] << 8) : 0;
        reason = n >= 6 ? p[5] : 0;
    }
};

// ---------------------------------------------------------------------------
// Queue
// ---------------------------------------------------------------------------
class Queue {
public:
    uint8_t size() const {
        uint8_t k = 0;
        for (const Req& r : q_) k += r.used;
        return k;
    }
    uint8_t ofBoard(uint8_t b) const {
        uint8_t k = 0;
        for (const Req& r : q_) k += r.used && r.board == b;
        return k;
    }
    // add: true, queued. False: refused, and place is what the busy answer
    // carries: the requests ahead, or kFull when the whole queue is.
    bool add(uint8_t board, uint16_t sess, const uint8_t* p, size_t n, uint8_t& place) {
        const uint8_t all = size();
        if (all >= kQueue) { place = kFull; return false; }
        if (ofBoard(board) >= kPerBoard) { place = all; return false; }
        for (Req& r : q_) {
            if (r.used) continue;
            r.set(board, sess, p, n);
            r.seq = ++seq_;
            return true;
        }
        place = kFull;
        return false;
    }
    // next: the next request, round-robin by board from the one after the
    // last served, the oldest of that board's. False when empty.
    bool next(Req& out) {
        for (uint8_t k = 0; k < kBoards; ++k) {
            const uint8_t b = static_cast<uint8_t>((rr_ + k) % kBoards);
            Req* best = nullptr;
            for (Req& r : q_)
                if (r.used && r.board == b && (!best || r.seq < best->seq)) best = &r;
            if (!best) continue;
            out = *best;
            best->used = false;
            rr_ = static_cast<uint8_t>((b + 1) % kBoards);
            return true;
        }
        return false;
    }
    // drop: a board's requests go (it went quiet or was forgotten); each
    // session is handed to f to let go.
    template <class F> void drop(uint8_t board, F f) {
        for (Req& r : q_)
            if (r.used && r.board == board) { r.used = false; f(r.sess); }
    }
    // dropSess: the one request on this board's session (it was reset:
    // the board gave up waiting), so it is not taken for nobody.
    bool dropSess(uint8_t board, uint16_t sess) {
        for (Req& r : q_)
            if (r.used && r.board == board && r.sess == sess) { r.used = false; return true; }
        return false;
    }
    // has: a request on this session is queued.
    bool has(uint8_t board, uint16_t sess) const {
        for (const Req& r : q_) if (r.used && r.board == board && r.sess == sess) return true;
        return false;
    }

private:
    Req      q_[kQueue];
    uint32_t seq_ = 0;
    uint8_t  rr_ = 0;
};

// ---------------------------------------------------------------------------
// Group: one EVENT, the SNAPs that answer it, one picture for them all.
// ---------------------------------------------------------------------------
class Group {
public:
    bool active() const { return active_; }
    uint8_t kind() const { return kind_; }

    void begin(uint8_t kind, uint32_t now) {
        *this = Group();
        active_ = true;
        kind_ = kind;
        at_ = now;
    }
    // asked: an EVENT went to this board on this session.
    void asked(uint8_t board, uint16_t sess) {
        if (board >= kBoards) return;
        sess_[board] = sess;
        asked_ |= static_cast<uint8_t>(1u << board);
    }
    bool askedAny() const { return asked_ != 0; }
    bool ours(uint8_t board, uint16_t sess) const {
        return active_ && board < kBoards && (asked_ & (1u << board)) && sess_[board] == sess;
    }
    // answer: the SNAP that answers this board's EVENT. False if it is not
    // one (another session), or the group has already taken its picture.
    bool answer(uint8_t board, uint16_t sess, const uint8_t* p, size_t n, uint32_t now) {
        if (!ours(board, sess) || taken_ || (answered_ & (1u << board))) return false;
        ans_[board].set(board, sess, p, n);
        answered_ |= static_cast<uint8_t>(1u << board);
        if (!firstAt_) firstAt_ = now ? now : 1;
        return true;
    }
    // ready: take the picture now: every board asked has answered, or the
    // collection window after the first answer has passed.
    bool ready(uint32_t now) const {
        if (!active_ || taken_ || !answered_) return false;
        return answered_ == asked_ || reached(now, firstAt_ + kCollectMs);
    }
    // expired: nobody answered: the EVENT is let go.
    bool expired(uint32_t now) const {
        return active_ && !answered_ && reached(now, at_ + kEventMs);
    }
    // take: the boards the picture goes to, the owner's first when it is
    // among them (its texts go on the picture), then by board. Marks the
    // group taken; sessions asked and never answered go to f.
    template <class F> uint8_t take(int owner, Req* out, F unanswered) {
        taken_ = true;
        uint8_t k = 0;
        if (owner >= 0 && owner < kBoards && (answered_ & (1u << owner))) out[k++] = ans_[owner];
        for (uint8_t b = 0; b < kBoards; ++b) {
            if (static_cast<int>(b) == owner) continue;
            if (answered_ & (1u << b)) out[k++] = ans_[b];
            else if (asked_ & (1u << b)) unanswered(b, sess_[b]);
        }
        if (owner >= 0 && owner < kBoards && !(answered_ & (1u << owner)) && (asked_ & (1u << owner)))
            unanswered(static_cast<uint8_t>(owner), sess_[owner]);
        return k;
    }
    // abandon: nobody answered: each session asked goes to f, and the group
    // ends.
    template <class F> void abandon(F f) {
        for (uint8_t b = 0; b < kBoards; ++b) if (asked_ & (1u << b)) f(b, sess_[b]);
        end();
    }
    // forget: a board that went quiet is not waited for.
    void forget(uint8_t board) {
        if (board >= kBoards) return;
        asked_ = static_cast<uint8_t>(asked_ & ~(1u << board));
        answered_ = static_cast<uint8_t>(answered_ & ~(1u << board));
    }
    void end() { *this = Group(); }

private:
    bool     active_ = false;
    bool     taken_ = false;
    uint8_t  kind_ = 0;
    uint32_t at_ = 0, firstAt_ = 0;
    uint8_t  asked_ = 0, answered_ = 0;
    uint16_t sess_[kBoards] = {};
    Req      ans_[kBoards];
};

// wants: a board's receive bits (SETTINGS byte 21) against an EVENT kind
// (CEV_MOTION 1, CEV_TIMELAPSE 2 in linkfam): RECV_TIMELAPSE 1, RECV_MOTION 2.
inline bool wants(uint8_t recv, uint8_t eventKind) {
    return eventKind == 1 ? (recv & 2) != 0 : (recv & 1) != 0;
}

}  // namespace sched
