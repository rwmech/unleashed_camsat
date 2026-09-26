// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/board.h
// Module:       The AI-Thinker ESP32-CAM's pins
//
// Purpose:      Every pin the satellite uses, from the AI-Thinker schematic
//               (as the core's ESPCAM profile, src/board.h, has them), and
//               the pins the rest of the board leaves free.
//
//               No SD card: the slot's pins (2, 4, 12, 13, 14, 15) are free
//               apart from GPIO 4, the white flash LED, which lights on a
//               floating pin and is held low from the first instruction.
//               GPIO 12 is a strapping pin (the flash voltage) and GPIO 16
//               is the PSRAM's chip select: neither is offered for motion.
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

#define CAMSAT_VERSION   "1.0.0"
#define CAMSAT_NAME      "camsat"

// The red LED on the back, 3V3 through the LED to the pin: lit when low.
#define PIN_LED          33
// The white flash LED's transistor. Held low except while a picture is lit.
#define PIN_FLASH        4
// The IO0 button on the ESP32-CAM-MB (and XCLK once the camera is up). Held
// low for 5 s while the camera is off: forget the pairing.
#define PIN_FORGET       0
// A motion sensor's output (a PIR goes high on motion). An RTC pin, so it
// can wake the satellite from deep sleep. The host's SETTINGS may move it.
#define PIN_MOTION_DEF   13

// The camera (OV2640 on the AI-Thinker board)
#define CAM_PWDN         32
#define CAM_RESET        -1
#define CAM_XCLK         0
#define CAM_SIOD         26
#define CAM_SIOC         27
#define CAM_D7           35
#define CAM_D6           34
#define CAM_D5           39
#define CAM_D4           36
#define CAM_D3           21
#define CAM_D2           19
#define CAM_D1           18
#define CAM_D0           5
#define CAM_VSYNC        25
#define CAM_HREF         23
#define CAM_PCLK         22

// motionPinOk: a pin a PIR may use: free on this board, an input, and one
// that can wake the chip (RTC GPIO), so the same pin works asleep.
inline bool motionPinOk(int p) {
    return p == 2 || p == 13 || p == 14 || p == 15;
}
