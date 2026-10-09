/*
 * ui.cpp - Cardputer LCD for ELRS watch mode.
 *
 * Only C++ file in the project, because M5GFX is C++. Everything it draws
 * comes from plain C structs (ui.h).
 *
 * Draws directly to the panel, no full-screen sprite: the RF engine reserves
 * 192 KiB of SRAM and a 240x135x16-bit frame buffer (64 KiB) may not fit
 * beside it. Call only BETWEEN capture slices: the engine masks interrupts,
 * and an SPI DMA transfer must not be in flight when it does (waitDMA()).
 *
 * Layout (240 x 135, rotation 1):
 *   y   0  header: mode, LO, "2.4GHz only"
 *   y  12  80-channel activity strip, 3 px per ELRS channel
 *   y  48  verdict line (colour coded)
 *   y  62  metrics
 *   y 112  permanent scope caveat
 *   y 124  key hints
 */
#include <M5GFX.h>
#include <cstdio>

#include "ui.h"

static M5GFX display;

static constexpr uint16_t COL_BG = 0x0000;
static constexpr uint16_t COL_FG = 0xFFFF;
static constexpr uint16_t COL_DIM = 0x8410;
static constexpr uint16_t COL_OK = 0x07E0;
static constexpr uint16_t COL_WARN = 0xFFE0;
static constexpr uint16_t COL_ALERT = 0xF800;
static constexpr uint16_t COL_BAR = 0x05FF;
static constexpr uint16_t COL_HIDDEN = 0x2104;

static constexpr int STRIP_Y = 12;
static constexpr int STRIP_H = 32;

static void chrome(void)
{
    display.fillScreen(COL_BG);
    display.setTextSize(1);
    display.setTextColor(COL_DIM, COL_BG);
    display.setCursor(2, 112);
    display.print("2.4GHz only. QUIET != clear. No 868/915, no 5.8");
    display.setCursor(2, 124);
    display.print("[c]lear [+/-]thr [i]nvert [t]est");
}

extern "C" void ui_init(void)
{
    display.init();
    display.setRotation(1);
    display.setBrightness(128);
    chrome();
}

extern "C" void ui_message(const char *line1, const char *line2)
{
    display.fillRect(0, 48, display.width(), 60, COL_BG);
    display.setTextColor(COL_WARN, COL_BG);
    display.setCursor(2, 50);
    display.print(line1 ? line1 : "");
    display.setCursor(2, 62);
    display.print(line2 ? line2 : "");
    display.waitDMA();
}

static uint16_t state_colour(elrs_state_t s)
{
    return s == ELRS_LIKELY ? COL_ALERT : s == ELRS_HOPPER ? COL_WARN : COL_OK;
}

extern "C" void ui_update(const elrs_status_t *st, const ui_info_t *info)
{
    char line[64];
    display.startWrite();

    /* header */
    display.fillRect(0, 0, display.width(), 10, COL_BG);
    display.setTextColor(COL_FG, COL_BG);
    display.setCursor(2, 1);
    snprintf(line, sizeof(line), "ELRS WATCH %uMHz +-%u", info->center_mhz, info->span_mhz / 2);
    display.print(line);

    /* activity strip: one 3 px column per ELRS channel 2400.4 + k MHz */
    display.fillRect(0, STRIP_Y, display.width(), STRIP_H, COL_BG);
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        const int x = (int)ch * 3;
        const int h = (st->activity[ch] * (STRIP_H - 2)) / 255;
        if (h > 0) {
            display.fillRect(x, STRIP_Y + STRIP_H - 1 - h, 2, h, COL_BAR);
        } else {
            display.drawFastHLine(x, STRIP_Y + STRIP_H - 1, 2, COL_HIDDEN);
        }
    }
    /* sync channel marker */
    display.drawFastVLine((int)ELRS_SYNC_CHANNEL * 3 + 1, STRIP_Y, 3, COL_DIM);

    /* verdict */
    display.fillRect(0, 48, display.width(), 62, COL_BG);
    display.setTextSize(1);
    display.setTextColor(state_colour(st->state), COL_BG);
    display.setCursor(2, 50);
    snprintf(line, sizeof(line), "%-11s score %.2f", elrs_state_name(st->state), (double)st->score);
    display.print(line);

    /* metrics */
    display.setTextColor(COL_FG, COL_BG);
    display.setCursor(2, 62);
    snprintf(line, sizeof(line), "grid ch %u/%u  on-grid %.0f%%", st->channels_seen,
             st->channels_visible, (double)(st->on_grid_ratio * 100.0f));
    display.print(line);
    display.setCursor(2, 74);
    snprintf(line, sizeof(line), "narrow %.0f/s wide %.0f/s dwell~%.0fms", (double)st->narrow_per_s,
             (double)st->wide_per_s, (double)st->dwell_ms);
    display.print(line);
    display.setCursor(2, 86);
    if (st->last_seen_us > 0 && info->uptime_us >= st->last_seen_us) {
        snprintf(line, sizeof(line), "last on-grid %.1fs ago  flat %.2f",
                 (double)(info->uptime_us - st->last_seen_us) / 1e6, (double)st->uniformity_cv);
    } else {
        snprintf(line, sizeof(line), "no on-grid bursts yet");
    }
    display.print(line);
    display.setTextColor(COL_DIM, COL_BG);
    display.setCursor(2, 98);
    snprintf(line, sizeof(line), "flr %d pk %d thr %u%s sl %ums drop %u crc %u", st->floor_db,
             st->peak_db, info->threshold_db, info->invert ? " INV" : "", (unsigned)info->slice_ms,
             (unsigned)info->drops, (unsigned)info->crc_bad);
    display.print(line);

    display.endWrite();
    display.waitDMA(); /* nothing in flight when the next slice masks IRQs */
}

extern "C" void ui_alert(void)
{
    display.fillScreen(COL_ALERT);
    display.waitDMA();
    chrome();
}
