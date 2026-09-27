// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/test/test_sched.cpp
// Module:       Host test / the satellite's multi-board scheduling (1.2.0)
//
// Purpose:      firmware/src/satsched.h on the host, no board and no radio.
//               Build and run (Linux, WSL), from firmware/test:
//                 g++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined
//                     -I../src -o test_sched test_sched.cpp
//                 ./test_sched
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
#include <cstdio>
#include <vector>

#include "satsched.h"

namespace {
int g_pass = 0, g_fail = 0;
void check(const char* what, bool ok) {
    printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    ++(ok ? g_pass : g_fail);
}
// A SNAP's bytes: req, then size, quality, flash, reason.
std::vector<uint8_t> snap(uint16_t req, uint8_t reason = 1) {
    return { static_cast<uint8_t>(req), static_cast<uint8_t>(req >> 8), 0, 0, 0xFF, reason };
}
}  // namespace

int main() {
    printf("The queue: two a board, eight in all, round-robin\n");
    {
        sched::Queue q;
        uint8_t place = 0;
        auto a = snap(1);
        check("a board's first request is queued", q.add(0, 100, a.data(), a.size(), place));
        auto b = snap(2);
        check("and its second", q.add(0, 101, b.data(), b.size(), place));
        auto c = snap(3);
        check("its third is refused", !q.add(0, 102, c.data(), c.size(), place));
        check("with the requests ahead as its place", place == 2);
        for (uint8_t bd = 1; bd <= 3; ++bd)
            for (uint8_t k = 0; k < 2; ++k) {
                auto s = snap(static_cast<uint16_t>(10 * bd + k));
                q.add(bd, static_cast<uint16_t>(200 + 10 * bd + k), s.data(), s.size(), place);
            }
        check("eight in all", q.size() == 8);
        auto d = snap(99);
        check("a ninth, from a board with none queued, is refused", !q.add(4, 300, d.data(), d.size(), place));
        check("with the queue-full place", place == sched::kFull);
        sched::Req r;
        std::vector<uint8_t> order;
        while (q.next(r)) order.push_back(r.board);
        check("served one a board in turn, not a board's two together",
              order.size() == 8 && order[0] == 0 && order[1] == 1 && order[2] == 2 && order[3] == 3 &&
              order[4] == 0 && order[5] == 1);
        check("and empty after", q.size() == 0 && !q.next(r));
    }
    {
        sched::Queue q;
        uint8_t place = 0;
        auto a = snap(7);
        q.add(2, 50, a.data(), a.size(), place);
        sched::Req r;
        check("a request keeps its bytes, req and reason",
              q.next(r) && r.board == 2 && r.sess == 50 && r.req == 7 && r.reason == 1 && r.n == 6);
        auto b = snap(8);
        auto c = snap(9);
        q.add(3, 60, b.data(), b.size(), place);
        q.add(1, 61, c.data(), c.size(), place);
        int dropped = 0;
        q.drop(3, [&](uint16_t s) { dropped += s == 60; });
        check("a board that goes has its requests dropped, each session handed back",
              dropped == 1 && q.size() == 1 && !q.has(3, 60) && q.has(1, 61));
    }

    printf("The group: one EVENT, one picture, every board that answered\n");
    {
        sched::Group g;
        g.begin(1, 1000);
        g.asked(0, 0x8001);
        g.asked(2, 0x8002);
        g.asked(3, 0x8003);
        auto a = snap(5, 4);
        check("nothing to take before an answer", !g.ready(1000) && !g.ready(5000));
        check("a SNAP on another session is not an answer", !g.answer(0, 0x8009, a.data(), a.size(), 1100));
        check("board 2 answers", g.answer(2, 0x8002, a.data(), a.size(), 1100));
        check("not ready inside the collection window", !g.ready(1100 + sched::kCollectMs - 1));
        auto b = snap(6, 4);
        check("the owner (board 0) answers too", g.answer(0, 0x8001, b.data(), b.size(), 1200));
        check("a board does not answer twice", !g.answer(0, 0x8001, b.data(), b.size(), 1250));
        check("ready once the window after the first answer has passed", g.ready(1100 + sched::kCollectMs));
        sched::Req out[sched::kBoards];
        std::vector<uint16_t> let;
        const uint8_t n = g.take(0, out, [&](uint8_t, uint16_t s) { let.push_back(s); });
        check("the picture goes to the two that answered", n == 2);
        check("the owner's first, so its texts go on the picture", out[0].board == 0 && out[1].board == 2);
        check("the board that did not answer is let go", let.size() == 1 && let[0] == 0x8003);
        check("a late answer is no longer the group's", !g.answer(3, 0x8003, a.data(), a.size(), 1500));
    }
    {
        sched::Group g;
        g.begin(2, 0);
        g.asked(1, 0x8010);
        g.asked(4, 0x8011);
        auto a = snap(1, 2);
        g.answer(4, 0x8011, a.data(), a.size(), 50);
        g.answer(1, 0x8010, a.data(), a.size(), 60);
        check("every board answered: ready at once", g.ready(61));
        sched::Req out[sched::kBoards];
        const uint8_t n = g.take(0, out, [](uint8_t, uint16_t) {});
        check("owner not among them: in board order", n == 2 && out[0].board == 1 && out[1].board == 4);
    }
    {
        sched::Group g;
        g.begin(1, 0xFFFFFF00u);                           // the millisecond clock about to wrap
        g.asked(0, 0x8020);
        check("an EVENT nobody answers expires", !g.expired(0xFFFFFF00u + 100) &&
                                                   g.expired(0xFFFFFF00u + sched::kEventMs));
        g.forget(0);
        check("a board that went quiet is not waited for", !g.askedAny());
    }
    {
        sched::Group g;
        g.begin(1, 0);
        g.asked(1, 0x8030);
        g.asked(3, 0x8031);
        std::vector<uint16_t> let;
        g.abandon([&](uint8_t, uint16_t s) { let.push_back(s); });
        check("an EVENT nobody answered lets every session go, and ends",
              let.size() == 2 && !g.active());
    }
    {
        sched::Queue q;
        uint8_t place = 0;
        auto a = snap(1);
        q.add(2, 70, a.data(), a.size(), place);
        q.add(2, 71, a.data(), a.size(), place);
        check("a board that gave up one SNAP loses that one only",
              q.dropSess(2, 70) && !q.has(2, 70) && q.has(2, 71) && !q.dropSess(2, 70));
    }
    printf("What a board wants\n");
    check("timelapse only: timelapse yes, motion no", sched::wants(1, 2) && !sched::wants(1, 1));
    check("motion only: motion yes, timelapse no", sched::wants(2, 1) && !sched::wants(2, 2));
    check("both", sched::wants(3, 1) && sched::wants(3, 2));

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
