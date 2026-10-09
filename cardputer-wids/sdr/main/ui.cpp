/*
 * ui.cpp - Cardputer LCD for ELRS watch mode.
 *
 * Only C++ file in the project, because M5GFX is C++. Everything it draws
 * comes from plain C structs (ui.h).
 *
 * Main view: one big answer.
 *     ELRS DETECTED   (red)    - detector verdict ELRS_LIKELY
 *     NOT DETECTED    (green)  - anything else
 * The line under it says WHICH kind of "not detected" it is, because
 * "not detected" is never "all clear": the device sees 2.4 GHz only, and a
 * link below the noise or outside the visible channels is simply missed.
 *
 * Press 'd' for the details view (channel strip and detector numbers).
 *
 * Draws directly to the panel, no full-screen sprite: the RF engine reserves
 * 192 KiB of SRAM and a 240x135x16-bit frame buffer (64 KiB) may not fit
 * beside it. Call only BETWEEN capture slices: the engine masks interrupts,
 * so nothing may be left in flight (waitDMA()).
 */
#include <M5GFX.h>
#include <cstdio>
#include <cstring>

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

static constexpr int BANNER_Y = 14;
static constexpr int BANNER_H = 62;

/* What is currently on the panel, so unchanged parts are not redrawn. */
static int s_drawn_state = -1;
static bool s_drawn_details = false;

static void footer(void)
{
    display.fillRect(0, 112, display.width(), 23, COL_BG);
    display.setTextSize(1);
    display.setTextColor(COL_DIM, COL_BG);
    display.setCursor(2, 112);
    display.print("2.4GHz only. Not detected != all clear");
    display.setCursor(2, 124);
    display.print("[d]etails [c]lear [+/-]thr [i]nv [t]est");
}

static void clear_all(void)
{
    display.fillScreen(COL_BG);
    footer();
    s_drawn_state = -1;
}

extern "C" void ui_init(void)
{
    display.init();
    display.setRotation(1);
    display.setBrightness(128);
    clear_all();
}

extern "C" void ui_message(const char *line1, const char *line2)
{
    display.fillRect(0, BANNER_Y, display.width(), 96, COL_BG);
    display.setTextSize(1);
    display.setTextColor(COL_WARN, COL_BG);
    display.setCursor(2, 50);
    display.print(line1 ? line1 : "");
    display.setCursor(2, 62);
    display.print(line2 ? line2 : "");
    display.waitDMA();
    s_drawn_state = -1;
}

/* Centre a string horizontally at the current text size. */
static void centred(const char *s, int y)
{
    const int w = display.textWidth(s);
    display.setCursor((display.width() - w) / 2, y);
    display.print(s);
}

static void draw_banner(elrs_state_t state)
{
    const bool detected = state == ELRS_LIKELY;
    const uint16_t bg = detected ? COL_ALERT : COL_BG;
    const uint16_t fg = detected ? COL_FG : COL_OK;

    display.fillRect(0, BANNER_Y, display.width(), BANNER_H, bg);
    if (!detected) {
        display.drawRect(4, BANNER_Y + 2, display.width() - 8, BANNER_H - 4, COL_OK);
    }
    display.setTextColor(fg, bg);
    display.setTextSize(2);
    centred("ELRS", BANNER_Y + 8);
    display.setTextSize(3);
    centred(detected ? "DETECTED" : "NOT DETECTED", BANNER_Y + 30);
    display.setTextSize(1);
}

/* One honest line under the banner: which kind of answer this is. */
static void draw_reason(const elrs_status_t *st, const ui_info_t *info)
{
    char line[64];
    display.fillRect(0, 80, display.width(), 30, COL_BG);
    display.setTextSize(1);

    switch (st->state) {
    case ELRS_LIKELY:
        display.setTextColor(COL_ALERT, COL_BG);
        snprintf(line, sizeof(line), "hopping on ELRS grid, %u channels", st->channels_seen);
        break;
    case ELRS_HOPPER:
        display.setTextColor(COL_WARN, COL_BG);
        snprintf(line, sizeof(line), "other hopping signal (not ELRS grid)");
        break;
    default:
        display.setTextColor(COL_DIM, COL_BG);
        snprintf(line, sizeof(line), "no hopping signal seen on 2.4 GHz");
        break;
    }
    centred(line, 82);

    display.setTextColor(COL_DIM, COL_BG);
    if (st->last_seen_us > 0 && info->uptime_us >= st->last_seen_us) {
        const double ago = (double)(info->uptime_us - st->last_seen_us) / 1e6;
        if (ago < 600.0) {
            snprintf(line, sizeof(line), "last ELRS-grid burst %.0f s ago", ago);
        } else {
            snprintf(line, sizeof(line), "last ELRS-grid burst %.0f min ago", ago / 60.0);
        }
    } else {
        snprintf(line, sizeof(line), "listening %u MHz +-%u", info->center_mhz, info->span_mhz / 2);
    }
    centred(line, 96);
}

static void draw_details(const elrs_status_t *st, const ui_info_t *info)
{
    char line[64];
    display.fillRect(0, 0, display.width(), 112, COL_BG);
    display.setTextSize(1);

    display.setTextColor(st->state == ELRS_LIKELY ? COL_ALERT : COL_OK, COL_BG);
    display.setCursor(2, 1);
    snprintf(line, sizeof(line), "%s  score %.2f", elrs_state_name(st->state), (double)st->score);
    display.print(line);

    /* activity strip: one 3 px column per ELRS channel 2400.4 + k MHz */
    const int y0 = 12, h0 = 32;
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        const int x = (int)ch * 3;
        const int h = (st->activity[ch] * (h0 - 2)) / 255;
        if (h > 0) {
            display.fillRect(x, y0 + h0 - 1 - h, 2, h, COL_BAR);
        } else {
            display.drawFastHLine(x, y0 + h0 - 1, 2, COL_HIDDEN);
        }
    }
    display.drawFastVLine((int)ELRS_SYNC_CHANNEL * 3 + 1, y0, 3, COL_DIM);

    display.setTextColor(COL_FG, COL_BG);
    display.setCursor(2, 50);
    snprintf(line, sizeof(line), "grid ch %u/%u  on-grid %.0f%%", st->channels_seen,
             st->channels_visible, (double)(st->on_grid_ratio * 100.0f));
    display.print(line);
    display.setCursor(2, 62);
    snprintf(line, sizeof(line), "narrow %.0f/s wide %.0f/s dwell~%.0fms", (double)st->narrow_per_s,
             (double)st->wide_per_s, (double)st->dwell_ms);
    display.print(line);
    display.setCursor(2, 74);
    snprintf(line, sizeof(line), "flatness %.2f  hops %u MHz +-%u", (double)st->uniformity_cv,
             info->center_mhz, info->span_mhz / 2);
    display.print(line);
    display.setTextColor(COL_DIM, COL_BG);
    display.setCursor(2, 86);
    snprintf(line, sizeof(line), "floor %d dB peak %d dB thr %u%s", st->floor_db, st->peak_db,
             info->threshold_db, info->invert ? " INV" : "");
    display.print(line);
    display.setCursor(2, 98);
    snprintf(line, sizeof(line), "slice %ums drops %u crc %u", (unsigned)info->slice_ms,
             (unsigned)info->drops, (unsigned)info->crc_bad);
    display.print(line);
}

extern "C" void ui_update(const elrs_status_t *st, const ui_info_t *info)
{
    display.startWrite();

    if (info->details != s_drawn_details) {
        display.fillRect(0, 0, display.width(), 112, COL_BG);
        s_drawn_details = info->details;
        s_drawn_state = -1;
    }

    if (info->details) {
        draw_details(st, info);
    } else {
        /* header */
        display.fillRect(0, 0, display.width(), 12, COL_BG);
        display.setTextSize(1);
        display.setTextColor(COL_DIM, COL_BG);
        display.setCursor(2, 2);
        display.print("ELRS WATCH  receive only");
        /* The banner only changes when the verdict does: no flicker. */
        if ((int)st->state != s_drawn_state) {
            draw_banner(st->state);
            s_drawn_state = (int)st->state;
        }
        draw_reason(st, info);
    }

    display.endWrite();
    display.waitDMA(); /* nothing in flight when the next slice masks IRQs */
}

extern "C" void ui_alert(void)
{
    display.fillScreen(COL_ALERT);
    display.waitDMA();
    clear_all();
}
