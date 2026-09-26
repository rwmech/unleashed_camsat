# µnleashed camsat

A camera for any [µnleashed BBS](https://github.com/rwmech/unleashed_BBS) board, on a few dollars of hardware.

camsat is a cheap ESP32-CAM running its own small firmware. It pairs with your board over the µnleashed link (ESP-NOW, no Wi-Fi network needed between them), sits anywhere in radio range (a window, the garden, the workbench) and takes a picture whenever the board asks. The board files it in its Photos area exactly as it would a snap from a built-in camera: the same names, the same watermark, the same limits for callers.

Your board needs no camera of its own, only an SD card for the photos.

This repository holds both halves:

| Folder | What it is |
|---|---|
| `firmware/` | The satellite's firmware, a PlatformIO project for the ESP32-CAM |
| `bbs/` | The board's half, a plugin built into the µnleashed firmware from this repository |

## What it does

- **Photos on request.** A caller types `SNAPSHOT` at the board. The satellite wakes its sensor, lets the exposure and white balance settle, takes the picture, corrects it (Auto levels and gamma), stamps the watermark and sends it. About nine seconds from command to filed photo at XGA (1024x768), measured on the bench.
- **Timelapse.** A picture every so often, on the board's clock.
- **Motion.** Wire a PIR sensor to the satellite and it asks the board for a picture when something moves, with a hold-off so one visitor is not fifty photos.
- **Battery friendly.** With deep sleep on, the satellite sleeps between pictures and wakes on its timer or on motion.
- **The board decides everything.** Every name, date, limit and word on the watermark comes from the board. The satellite only takes the picture.

## Hardware

- An **AI-Thinker ESP32-CAM** (ESP32, 4 MB flash, PSRAM, OV2640). **No SD card needed**: leave the slot empty.
- An **ESP32-CAM-MB** programmer board (the USB board it plugs into) makes flashing one click. A bare ESP32-CAM works too with any 3.3 V USB-serial adapter.
- Optional: a PIR motion sensor on GPIO 13 (or 14, 15 or 2), powered from the board's 3.3 V or 5 V as the sensor wants.
- 5 V power where you mount it. The camera and radio draw a few hundred mA while taking a picture.

The red LED on the back says what the satellite is doing:

| Red LED | Meaning |
|---|---|
| double blink every second | pairing: waiting for a board |
| slow blink | paired, looking for its board |
| short blip every 3 s | connected |
| lit | taking a picture |
| fast flicker | IO0 held: about to forget its pairing |

The bright white LED is the flash. It stays off unless the board's settings ask for it.

## Flashing the satellite

You need [PlatformIO](https://platformio.org/) (the VS Code extension or the command line) and git.

```
git clone https://github.com/rwmech/unleashed_camsat
cd unleashed_camsat/firmware
pio run -t upload --upload-port COM5
```

Use your own port name (`/dev/ttyUSB0` on Linux). The build fetches the link code from µnleashed BBS itself, at the version `core.lock` names, so the satellite always speaks exactly what the board speaks.

On the ESP32-CAM-MB the upload resets the board by itself. On a bare ESP32-CAM, hold IO0 to GND while you reset it, then release it once the upload starts.

To watch what it is doing:

```
pio device monitor --port COM5
```

## Pairing

Pairing needs you at both ends, inside a two-minute window. Nothing pairs from the air alone.

1. On the board, as the sysop, switch on the link (`CONFIG link`) and the camera satellite plugin (`CONFIG camsat`). An SD card must be in.
2. Type `LINK PAIR`. The board opens pairing for 2 minutes.
3. Power up the satellite. A satellite that is not paired to anything waits 5 minutes for a board (the red LED double-blinks).
4. The board asks: `Pair camsat "camsat-6cc8" 20:50:0d:18:6c:c8, code 4821? (y/N)`. Answer Y.
5. The satellite prints the same four-digit code on its serial console. If you can see it, check it matches and tell the board so. Matching codes rule out anybody in the middle.

From then on the satellite finds its board by itself after every power cut, and follows it if the router changes channel.

**Starting again:** hold IO0 (the button on the MB board) for 5 seconds while the satellite is running. It forgets its pairing and restarts, ready to pair. Holding IO0 while powering up does something else: that is the ESP32's flashing mode.

## On the board

Each paired satellite is a camera in the board's own camera list, under the name LINK gives it (`LINK NAME` changes it). The board's commands work the same whether a camera is built in or a satellite:

| Command | Who | What |
|---|---|---|
| `SNAPSHOT [n\|name]` | callers (as the sysop sets) | take a photo with the default camera, or camera `n` or `name` |
| `CAMERA` | staff | every camera on the board: up, busy or down, and what each is doing |
| `CAMERA <n\|name>` | staff | one satellite: the link and its signal, the sensor, memory, uptime, photos taken |
| `LINK` | staff | every paired device on the link |
| `CONFIG cameras` | sysop | which camera `SNAPSHOT` uses when none is named |
| `CONFIG camsat` | sysop | size, quality, names, watermark, flash, timelapse, motion, picture settings, deep sleep |

With one camera on the board, `SNAPSHOT` is simply that camera and `CAMERA` shows it directly.

Photos land in the Photos area (FILES), named like a built-in camera's: `SNAP-20260926-114523.JPG` for a caller's, `timelapse/TL-...` and `motion/MO-...` for the board's own.

Callers get one allowance across every camera on the board: 10 photos an hour and 20 a day each, the sysop exempt.

This needs µnleashed BBS 1.2.0 or later, which carries the link and the camera list.

## How it works

The satellite and the board talk over the µnleashed link: ESP-NOW frames of 250 bytes, sealed with AES-128-CCM under a key agreed at pairing, with sessions, acknowledgements and retries on top. A photo travels as one message in fragments and is written to the card as it arrives, never held in the board's RAM. The whole protocol is specified in [LINK.md](https://github.com/rwmech/unleashed_BBS/blob/main/LINK.md) in the µnleashed BBS repository (1.2.0 and later).

The satellite does the pixel work (levels, gamma, the watermark) because it has the PSRAM and nothing else to do, so a board with no camera still gets exactly the picture a camera board takes.

## Licence

GNU General Public License v3 or later (GPL-3.0-or-later). See [LICENSE](LICENSE).

Copyright 2026 Robert Mech.

It uses Espressif's [esp32-camera](https://github.com/espressif/esp32-camera) driver (Apache-2.0) and ESP-IDF (Apache-2.0).
