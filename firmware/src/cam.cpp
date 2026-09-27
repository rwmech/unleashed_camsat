// ===========================================================================
//  µnleashed camsat
// ===========================================================================
//
// File:         firmware/src/cam.cpp
// Module:       Taking the picture
//
// Purpose:      See cam.h. The bring-up, the settle and the picture work are
//               the core's camera plugin's (src/plugins/camera.cpp and
//               platform_esp32.cpp, 1.1.1), brought here so a board with no
//               camera and no codec gets the same picture a camera board
//               takes: the OV2640's AWB gain on whatever the white balance
//               mode (the 1.1.0 green cast), frames thrown away until the
//               exposure holds still, Auto levels and gamma, the watermark,
//               and the comment.
//
// Libraries:    esp32-camera 2.1.7 (Apache-2.0): the driver, and its jpge
//               encoder; the ROM's TJpgDec
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
#include "cam.h"

#include <cstdio>
#include <cstring>
#include <new>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensor.h"

#include "esp32/rom/tjpgd.h"
#include "jpge.h"

#include "board.h"
#include "camera_mark.h"
#include "camera_pic.h"
#include "camera_rules.h"
#include "linkfam.h"

namespace {

const char* TAG = "cam";

// The largest picture the link carries (ulink::kBulkMax, 512 KB) less the
// header. A picture bigger than this is encoded again, lower.
constexpr size_t kOutCap = 512u * 1024u - cam::kHead;
uint8_t* g_out = nullptr;
char     g_sensor[12] = "";
uint8_t  g_maxSize = 0;
uint16_t g_pid = 0;
// The same two, kept through deep sleep for the status a wake sends before
// its camera comes up (the board's CAMERA said "Sensor not said yet" for a
// sleeping satellite for ever). g_sensor alone still means "answered this
// boot", which is what a stuck sensor's restart is decided on.
RTC_DATA_ATTR char    r_sensor[12] = "";
RTC_DATA_ATTR uint8_t r_maxSize = 0;

uint32_t ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

void* palloc(size_t n) {
    void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : heap_caps_malloc(n, MALLOC_CAP_8BIT);
}

void flash(bool on) { gpio_set_level(static_cast<gpio_num_t>(PIN_FLASH), on ? 1 : 0); }

framesize_t frameOf(uint8_t cs) {
    switch (cs) {
        case linkfam::CS_QQVGA: return FRAMESIZE_QQVGA;
        case linkfam::CS_QVGA:  return FRAMESIZE_QVGA;
        case linkfam::CS_VGA:   return FRAMESIZE_VGA;
        case linkfam::CS_SVGA:  return FRAMESIZE_SVGA;
        case linkfam::CS_SXGA:  return FRAMESIZE_SXGA;
        case linkfam::CS_UXGA:  return FRAMESIZE_UXGA;
        default:                return FRAMESIZE_XGA;
    }
}

uint8_t csOf(framesize_t fs) {
    if (fs >= FRAMESIZE_UXGA) return linkfam::CS_UXGA;
    if (fs >= FRAMESIZE_SXGA) return linkfam::CS_SXGA;
    if (fs >= FRAMESIZE_XGA)  return linkfam::CS_XGA;
    if (fs >= FRAMESIZE_SVGA) return linkfam::CS_SVGA;
    if (fs >= FRAMESIZE_VGA)  return linkfam::CS_VGA;
    if (fs >= FRAMESIZE_QVGA) return linkfam::CS_QVGA;
    return linkfam::CS_QQVGA;
}

// open: the sensor up, JPEG, at the size and quality asked, with every
// automatic control on (the core's camOpen, 1.1.1).
bool open(framesize_t fs, uint8_t quality, const PicSettings& s, char* err, size_t en) {
    camera_config_t c = {};
    c.pin_pwdn = CAM_PWDN;     c.pin_reset = CAM_RESET;  c.pin_xclk = CAM_XCLK;
    c.pin_sccb_sda = CAM_SIOD; c.pin_sccb_scl = CAM_SIOC;
    c.pin_d7 = CAM_D7; c.pin_d6 = CAM_D6; c.pin_d5 = CAM_D5; c.pin_d4 = CAM_D4;
    c.pin_d3 = CAM_D3; c.pin_d2 = CAM_D2; c.pin_d1 = CAM_D1; c.pin_d0 = CAM_D0;
    c.pin_vsync = CAM_VSYNC;   c.pin_href = CAM_HREF;    c.pin_pclk = CAM_PCLK;
    c.xclk_freq_hz = 20000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = fs;
    c.jpeg_quality = quality;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.sccb_i2c_port = -1;
    const esp_err_t e = esp_camera_init(&c);
    if (e != ESP_OK) {
        if (esp_camera_sensor_get()) esp_camera_deinit();
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(e));
        snprintf(err, en, "%s", e == ESP_ERR_NOT_SUPPORTED || e == ESP_ERR_NOT_FOUND ||
                                        e == ESP_ERR_CAMERA_NOT_DETECTED
                                    ? "no camera found: check the ribbon" : "the camera would not start");
        return false;
    }
    sensor_t* x = esp_camera_sensor_get();
    if (x) {
        camera_sensor_info_t* info = esp_camera_sensor_get_info(&x->id);
        snprintf(g_sensor, sizeof(g_sensor), "%s", info ? info->name : "unknown");
        g_pid = x->id.PID;
        if (info) g_maxSize = csOf(info->max_size);
        memcpy(r_sensor, g_sensor, sizeof(r_sensor));
        r_maxSize = g_maxSize;
        if (x->set_vflip)          x->set_vflip(x, s.flip ? 1 : 0);
        if (x->set_hmirror)        x->set_hmirror(x, s.mirror ? 1 : 0);
        if (x->set_brightness)     x->set_brightness(x, s.bright);
        if (x->set_contrast)       x->set_contrast(x, s.contrast);
        if (x->set_saturation)     x->set_saturation(x, s.sat);
        if (x->set_ae_level)       x->set_ae_level(x, s.exposure);
        // Every automatic control on. The AWB gain on whatever the mode: off,
        // the OV2640 measures the white balance and never applies it.
        if (x->set_exposure_ctrl)  x->set_exposure_ctrl(x, 1);
        if (x->set_aec2)           x->set_aec2(x, 1);
        if (x->set_gain_ctrl)      x->set_gain_ctrl(x, 1);
        if (x->set_whitebal)       x->set_whitebal(x, 1);
        if (x->set_awb_gain)       x->set_awb_gain(x, 1);
        if (x->set_wb_mode)        x->set_wb_mode(x, s.wb);
        if (x->set_lenc)           x->set_lenc(x, 1);
        if (x->set_raw_gma)        x->set_raw_gma(x, 1);
        if (x->set_bpc)            x->set_bpc(x, 1);
        if (x->set_wpc)            x->set_wpc(x, 1);
        if (x->set_dcw)            x->set_dcw(x, 1);
        if (x->set_special_effect) x->set_special_effect(x, s.effect);
        if (g_pid == GC0308_PID && x->set_reg) {
            campic::Reg regs[6];
            const uint8_t n = campic::gc0308Regs(s.bright, s.contrast, s.sat, s.exposure, regs);
            for (uint8_t i = 0; i < n; ++i) x->set_reg(x, regs[i].reg, 0xFF, regs[i].val);
        }
    }
    return true;
}

void close() {
    if (esp_camera_sensor_get()) esp_camera_deinit();
}

// unstick: the sensor stopped answering on its SCCB bus (seen on the bench,
// now and then, after a picture that went fine: every later bring-up then
// finds nothing until the satellite restarts). Clock SDA free if the sensor
// is holding it, and power the sensor down and up again through PWDN.
void unstick() {
    const gpio_num_t sda = static_cast<gpio_num_t>(CAM_SIOD), scl = static_cast<gpio_num_t>(CAM_SIOC);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    for (int i = 0; i < 9 && gpio_get_level(sda) == 0; ++i) {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(10);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(10);
    }
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    const gpio_num_t pwdn = static_cast<gpio_num_t>(CAM_PWDN);
    gpio_reset_pin(pwdn);
    gpio_set_direction(pwdn, GPIO_MODE_OUTPUT);
    gpio_set_level(pwdn, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(pwdn, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

// meter: the OV2640's exposure and gain, or the GC0308's average and
// target (the core's camMeter). 0 none, 1 luma, 2 exposure.
uint8_t meter(uint16_t& a, uint16_t& b) {
    sensor_t* s = esp_camera_sensor_get();
    if (!s || !s->get_reg) return 0;
    if (g_pid == GC0308_PID && s->set_reg) {
        if (s->set_reg(s, 0xFE, 0xFF, 0x00) < 0) return 0;
        const int y = s->get_reg(s, 0xD4, 0xFF), t = s->get_reg(s, 0xD3, 0xFF);
        if (y < 0 || t < 0) return 0;
        a = static_cast<uint16_t>(y);
        b = static_cast<uint16_t>(t);
        return 1;
    }
    if (g_pid == OV2640_PID) {
        const int hi = s->get_reg(s, 0x145, 0x3F), mid = s->get_reg(s, 0x110, 0xFF);
        const int lo = s->get_reg(s, 0x104, 0x03), gain = s->get_reg(s, 0x100, 0xFF);
        if (hi < 0 || mid < 0 || lo < 0 || gain < 0) return 0;
        a = static_cast<uint16_t>(hi << 10 | mid << 2 | lo);
        b = static_cast<uint16_t>(gain);
        return 2;
    }
    return 0;
}

void dropFrame() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
}

// settle: frames thrown away until the sensor says it has settled.
uint8_t settle() {
    campic::Settle st;
    campic::Steady sd;
    const uint32_t from = ms();
    uint8_t frames = 0;
    for (int i = 0; i < 60; ++i) {
        dropFrame();
        ++frames;
        uint16_t a = 0, b = 0;
        const uint8_t kind = meter(a, b);
        const uint32_t took = ms() - from;
        if (kind == 1) {
            if (campic::settled(st, static_cast<uint8_t>(a), static_cast<uint8_t>(b), took) ||
                took >= campic::kSettleMaxMs) break;
        } else if (kind == 2) {
            if (campic::steady(sd, a, b) || took >= campic::kSteadyMaxMs) break;
        } else if (i >= 2) {
            break;
        }
    }
    return frames;
}

// ---------------------------------------------------------------------------
// The output: the JPEG as it is made, into the buffer behind the header.
// ---------------------------------------------------------------------------
struct Out {
    uint8_t* p;
    size_t   len, cap;
    bool     ok;
};

bool outPut(void* ctx, const uint8_t* p, size_t n) {
    Out* o = static_cast<Out*>(ctx);
    if (!o->ok || o->len + n > o->cap) { o->ok = false; return false; }
    memcpy(o->p + o->len, p, n);
    o->len += n;
    return true;
}

// ---------------------------------------------------------------------------
// Decode a strip at a time with the ROM's TJpgDec, correct and mark it, and
// encode it again with jpge (the core's jpegMark), and the eighth-size
// decode for the histogram (the core's jpegHist).
// ---------------------------------------------------------------------------
class Stream : public jpge::output_stream {
public:
    Stream(camrules::ComSink& s) : s_(s) {}
    bool put_buf(const void* p, int len) override {
        if (!ok_) return false;
        ok_ = s_.put(static_cast<const uint8_t*>(p), static_cast<size_t>(len));
        size_ += static_cast<uint>(len);
        return ok_;
    }
    uint get_size() const override { return size_; }
    bool ok() const { return ok_; }
private:
    camrules::ComSink& s_;
    uint size_ = 0;
    bool ok_ = true;
};

struct Draw {
    const uint8_t (*lut)[256];
    const char*   text;               // null: no watermark
    cammark::Box  box;
    bool          laid;
    uint16_t      h;
};

void drawRows(Draw& d, uint8_t* rgb, uint16_t width, uint16_t y0, uint16_t rows) {
    if (d.lut) campic::apply(d.lut, rgb, static_cast<size_t>(width) * rows);
    if (d.text) {
        if (!d.laid) { d.box = cammark::layout(width, d.h, d.text); d.laid = true; }
        cammark::drawRows(rgb, width, y0, rows, d.box, d.text);
    }
}

struct MarkJob {
    const uint8_t*      src;
    size_t              len, pos;
    uint8_t*            strip;
    uint16_t            w, h, stripY;
    jpge::jpeg_encoder* enc;
    Draw*               draw;
    bool                ok;
};

UINT markIn(JDEC* jd, BYTE* buf, UINT n) {
    MarkJob* j = static_cast<MarkJob*>(jd->device);
    const size_t left = j->len - j->pos;
    if (n > left) n = static_cast<UINT>(left);
    if (buf) memcpy(buf, j->src + j->pos, n);
    j->pos += n;
    return n;
}

UINT markOut(JDEC* jd, void* bitmap, JRECT* r) {
    MarkJob* j = static_cast<MarkJob*>(jd->device);
    const uint8_t* px = static_cast<const uint8_t*>(bitmap);
    const size_t bw = static_cast<size_t>(r->right - r->left + 1) * 3u;
    for (uint16_t y = r->top; y <= r->bottom; ++y) {
        memcpy(j->strip + (static_cast<size_t>(y - j->stripY) * j->w + r->left) * 3u, px, bw);
        px += bw;
    }
    if (r->right + 1u < j->w) return 1;
    const uint16_t rows = static_cast<uint16_t>(r->bottom - j->stripY + 1);
    drawRows(*j->draw, j->strip, j->w, j->stripY, rows);
    for (uint16_t i = 0; i < rows; ++i)
        if (!j->enc->process_scanline(j->strip + static_cast<size_t>(i) * j->w * 3u)) { j->ok = false; return 0; }
    j->stripY = static_cast<uint16_t>(j->stripY + rows);
    return 1;
}

bool reencode(const uint8_t* jpg, size_t len, uint8_t quality, Draw& d, camrules::ComSink& sink) {
    constexpr size_t kPool = 3100;
    void* pool = palloc(kPool);
    JDEC* jd = static_cast<JDEC*>(palloc(sizeof(JDEC)));
    MarkJob job = {};
    job.src = jpg; job.len = len; job.ok = true; job.draw = &d;
    bool done = false;
    uint8_t* strip = nullptr;
    void* encMem = nullptr;
    Stream stream(sink);
    if (pool && jd && jd_prepare(jd, markIn, pool, kPool, &job) == JDR_OK) {
        job.w = static_cast<uint16_t>(jd->width);
        job.h = static_cast<uint16_t>(jd->height);
        d.h = job.h;
        strip = static_cast<uint8_t*>(palloc(static_cast<size_t>(job.w) * jd->msy * 8u * 3u));
        encMem = palloc(sizeof(jpge::jpeg_encoder));
        if (strip && encMem) {
            jpge::jpeg_encoder* enc = new (encMem) jpge::jpeg_encoder();
            jpge::params p;
            p.m_quality = quality < 1 ? 1 : quality > 100 ? 100 : quality;
            p.m_subsampling = jpge::H2V1;
            job.enc = enc;
            job.strip = strip;
            if (enc->init(&stream, job.w, job.h, 3, p)) {
                const JRESULT r = jd_decomp(jd, markOut, 0);
                done = r == JDR_OK && job.ok && job.stripY == job.h && enc->process_scanline(nullptr) &&
                       stream.ok();
            }
            enc->deinit();
            enc->~jpeg_encoder();
        }
    }
    heap_caps_free(encMem);
    heap_caps_free(strip);
    heap_caps_free(jd);
    heap_caps_free(pool);
    return done;
}

struct HistJob {
    const uint8_t* src;
    size_t         len, pos;
    campic::Hist*  h;
};

UINT histIn(JDEC* jd, BYTE* buf, UINT n) {
    HistJob* j = static_cast<HistJob*>(jd->device);
    const size_t left = j->len - j->pos;
    if (n > left) n = static_cast<UINT>(left);
    if (buf) memcpy(buf, j->src + j->pos, n);
    j->pos += n;
    return n;
}

UINT histOut(JDEC* jd, void* bitmap, JRECT* r) {
    HistJob* j = static_cast<HistJob*>(jd->device);
    const size_t px = static_cast<size_t>(r->right - r->left + 1) * (r->bottom - r->top + 1);
    campic::histRgb888(static_cast<const uint8_t*>(bitmap), px, *j->h);
    return 1;
}

bool hist(const uint8_t* jpg, size_t len, campic::Hist& h) {
    constexpr size_t kPool = 3100;
    void* pool = palloc(kPool);
    JDEC* jd = static_cast<JDEC*>(palloc(sizeof(JDEC)));
    HistJob job{ jpg, len, 0, &h };
    const bool ok = pool && jd && jd_prepare(jd, histIn, pool, kPool, &job) == JDR_OK &&
                    jd_decomp(jd, histOut, 3) == JDR_OK;
    heap_caps_free(jd);
    heap_caps_free(pool);
    return ok;
}

}  // namespace

namespace cam {

bool begin() {
    gpio_reset_pin(static_cast<gpio_num_t>(PIN_FLASH));
    gpio_set_direction(static_cast<gpio_num_t>(PIN_FLASH), GPIO_MODE_OUTPUT);
    flash(false);
    if (!g_out) g_out = static_cast<uint8_t*>(heap_caps_malloc(kOutCap + kHead, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return g_out != nullptr;
}

const char* sensor() { return g_sensor[0] ? g_sensor : r_sensor; }
uint8_t maxSize() { return g_sensor[0] ? g_maxSize : r_maxSize; }

bool snap(const SnapReq& r, const PicSettings& s, Pic& pic) {
    pic = Pic();
    pic.buf = g_out;
    pic.cap = kOutCap;
    auto fail = [&](uint8_t code, const char* why) {
        pic.failCode = code;
        snprintf(pic.err, sizeof(pic.err), "%s", why);
        flash(false);
        return false;
    };
    if (!g_out) return fail(linkfam::CE_NOMEM, "no memory for the picture");

    const uint8_t size = r.size ? r.size : s.size;
    uint8_t q = r.quality ? r.quality : s.quality;
    if (q < 4) q = 4;
    if (q > 63) q = 63;
    const uint8_t fl = r.flash != 0xFF ? r.flash : s.flash;

    const uint32_t t0 = ms();
    char why[48] = "";
    if (!open(frameOf(size), q, s, why, sizeof(why))) {
        // Once more after freeing the bus and power-cycling the sensor. A
        // sensor that answered earlier this boot and still does not is the
        // satellite's to restart (main.cpp), which the bench found clears it.
        ESP_LOGW(TAG, "the sensor did not answer: freeing its bus and trying again");
        unstick();
        if (!open(frameOf(size), q, s, why, sizeof(why))) {
            pic.stuck = g_sensor[0] != '\0';
            return fail(linkfam::CE_NOSENSOR, why);
        }
        ESP_LOGW(TAG, "the sensor answered the second time");
    }
    if (fl == linkfam::CF_ON) { flash(true); pic.flashed = true; }
    pic.settleFrames = settle();
    if (fl == linkfam::CF_AUTO && !pic.flashed) {
        // Dark: the OV2640's gain is well up. Light it and let the exposure
        // come back down before the picture.
        uint16_t a = 0, b = 0;
        if (meter(a, b) == 2 && b >= 0x30) {
            flash(true);
            pic.flashed = true;
            pic.settleFrames = static_cast<uint8_t>(pic.settleFrames + settle());
        }
    }
    pic.msUp = ms() - t0;

    // The frame being filled as the flash came on began before it, so it
    // goes; the next is the picture. A JPEG that is not whole twice running
    // at this quality may not fit the frame buffer: one step down.
    const uint32_t t1 = ms();
    uint8_t* src = nullptr;
    size_t srcLen = 0;
    for (int tries = 0; tries < 4 && !src; ++tries) {
        if (tries == 2 && q < 40) {
            sensor_t* x = esp_camera_sensor_get();
            if (x && x->set_quality && x->set_quality(x, q + 2) == 0) q = static_cast<uint8_t>(q + 2);
        }
        dropFrame();
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) continue;
        size_t n = fb->len;
        if (camrules::jpegWhole(fb->buf, n)) {
            // The frame, copied behind where the output will grow to, so the
            // sensor's buffer goes back before the pixel work.
            src = static_cast<uint8_t*>(palloc(n));
            if (src) { memcpy(src, fb->buf, n); srcLen = n; pic.w = fb->width; pic.h = fb->height; }
        }
        esp_camera_fb_return(fb);
    }
    flash(false);
    close();
    pic.msShot = ms() - t1;
    if (!src) return fail(linkfam::CE_CAPTURE, "the camera gave no whole picture");
    pic.srcBytes = static_cast<uint32_t>(srcLen);

    // The pixel work: the correction's tables, then the watermark on top.
    const uint32_t t2 = ms();
    uint8_t (*lut)[256] = static_cast<uint8_t (*)[256]>(palloc(3 * 256));
    bool fix = false;
    if (lut) {
        campic::Hist* h = nullptr;
        if (s.levels) {
            h = static_cast<campic::Hist*>(palloc(sizeof(campic::Hist)));
            if (h) { memset(h, 0, sizeof(*h)); if (!hist(src, srcLen, *h)) h->n = 0; }
        }
        uint8_t lo[3], hi[3];
        campic::buildTables(h, s.levels, campic::gammaTenths(s.gamma), lut, lo, hi);
        heap_caps_free(h);
        fix = !campic::identity(lut);
    }
    char text[96] = "";
    if (r.mark && pic.w && pic.h) {
        char board[48], who[32];
        cammark::toFont(r.board, board, sizeof(board));
        cammark::toFont(r.who, who, sizeof(who));
        int max = cammark::maxGlyphs(pic.w, pic.h);
        if (max > static_cast<int>(sizeof(text)) - 1) max = static_cast<int>(sizeof(text)) - 1;
        cammark::fitText(board, r.when, who, max, text, sizeof(text));
    }
    const bool mark = text[0] != '\0';

    bool ok = false;
    uint8_t jq = campic::reencodeQuality(q);
    for (int pass = 0; pass < 3 && !ok; ++pass) {
        Out o{ g_out + kHead, 0, kOutCap, true };
        camrules::ComSink sink(outPut, &o, r.comment);
        if (mark || fix) {
            Draw d{ fix ? lut : nullptr, mark ? text : nullptr, {}, false, pic.h };
            ok = reencode(src, srcLen, jq, d, sink) && o.ok;
            if (ok) { pic.marked = mark; pic.fixed = fix; }
        }
        if (!ok && o.ok && !(mark || fix)) ok = sink.put(src, srcLen) && o.ok;
        if (!ok && !o.ok) {                      // too big: again, lower
            ESP_LOGW(TAG, "picture over %u bytes at quality %u; again lower", static_cast<unsigned>(kOutCap),
                     static_cast<unsigned>(jq));
            jq = static_cast<uint8_t>(jq > 65 ? jq - 15 : 50);
            continue;
        }
        if (!ok) {                               // the re-encode failed: as the sensor gave it
            o = Out{ g_out + kHead, 0, kOutCap, true };
            camrules::ComSink raw(outPut, &o, r.comment);
            ok = raw.put(src, srcLen) && o.ok;
            pic.marked = pic.fixed = false;
        }
        if (ok) pic.len = o.len;
    }
    heap_caps_free(lut);
    heap_caps_free(src);
    pic.msWork = ms() - t2;
    if (!ok) return fail(linkfam::CE_NOMEM, "the picture would not fit");
    return true;
}

}  // namespace cam
