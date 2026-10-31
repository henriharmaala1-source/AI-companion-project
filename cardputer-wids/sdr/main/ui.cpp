/*
 * ui.cpp - Cardputer LCD for ELRS watch mode.
 *
 * Only C++ file in the project, because M5GFX is C++. Everything it draws
 * comes from plain C structs (ui.h).
 *
 * Main view, ELRS detected: the big red banner, why, how strong, and up to
 * two more lines for anything else seen at the same time.
 * Main view, no ELRS: a small green "ELRS NOT DETECTED" header and a list of
 * everything else recognised (targets.c), most important first: DJI-like
 * video links, fixed LoRa, other hoppers, wide signals, microwave ovens,
 * Wi-Fi. "Not detected" is never "all clear": the device sees 2.4 GHz only,
 * and a link below the noise or outside the visible channels is missed.
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

#include "chirp_watch.h"
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
static constexpr uint16_t COL_ORANGE = 0xFD20;
static constexpr uint16_t COL_CYAN = 0x07FF;

/* Main view, ELRS detected: the banner, then four short lines. */
static constexpr int BANNER_Y = 6;
static constexpr int BANNER_H = 70;
static constexpr int LINES_Y = 82;      /* text area below the banner */
/* Main view, no ELRS: a header box (the simulator reads its outline), then
 * the target list and two status lines. */
static constexpr int HEAD_Y = 2;
static constexpr int HEAD_H = 30;
static constexpr int LIST_Y = 36;
static constexpr int ROW_H = 12;
static constexpr unsigned LIST_ROWS = 6;
static constexpr int STATUS_Y = 112;

/* What is currently on the panel, so unchanged parts are not redrawn. */
static int s_drawn_state = -1;
static bool s_drawn_details = false;

static void clear_all(void)
{
    display.fillScreen(COL_BG);
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
    display.fillScreen(COL_BG);
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

static void draw_banner_detected(void)
{
    display.fillRect(0, BANNER_Y, display.width(), BANNER_H, COL_ALERT);
    display.setTextColor(COL_FG, COL_ALERT);
    display.setTextSize(2);
    centred("ELRS", BANNER_Y + 10);
    display.setTextSize(3);
    centred("DETECTED", BANNER_Y + 36);
    display.setTextSize(1);
}

static void draw_header_quiet(void)
{
    display.fillRect(0, 0, display.width(), HEAD_Y + HEAD_H + 2, COL_BG);
    display.drawRect(4, HEAD_Y, display.width() - 8, HEAD_H, COL_OK);
    display.setTextColor(COL_OK, COL_BG);
    display.setTextSize(2);
    centred("ELRS NOT DETECTED", HEAD_Y + 8);
    display.setTextSize(1);
}

static uint16_t level_colour(uint8_t level)
{
    switch (level) {
    case UI_LVL_WARN: return COL_ORANGE;
    case UI_LVL_NOTE: return COL_WARN;
    case UI_LVL_INFO: return COL_CYAN;
    default: return COL_DIM;
    }
}

/* One target line: a colour mark, then the text. */
static void draw_target(const ui_target_t *t, int y)
{
    const uint16_t c = level_colour(t->level);
    display.fillRect(2, y + 1, 5, 6, c);
    display.setTextColor(t->level == UI_LVL_DIM ? COL_DIM : COL_FG, COL_BG);
    display.setCursor(10, y);
    display.print(t->text);
}

/* Up to `rows` targets from y; if more exist, the last row says how many. */
static void draw_targets(const ui_info_t *info, int y, unsigned rows)
{
    const unsigned n = info->n_targets;
    for (unsigned i = 0; i < n && i < rows; i++) {
        if (i + 1 == rows && n > rows) {
            /* No room: name what is left, one word each. */
            char line[UI_TARGET_TEXT];
            size_t w = (size_t)snprintf(line, sizeof(line), "also:");
            for (unsigned j = i; j < n && w < sizeof(line); j++) {
                const int k = snprintf(line + w, sizeof(line) - w, "%s %s", j > i ? "," : "",
                                       info->targets[j].tag ? info->targets[j].tag : "?");
                w = k < 0 ? sizeof(line) : w + (size_t)k;
            }
            display.setTextColor(COL_DIM, COL_BG);
            display.setCursor(10, y);
            display.print(line);
            return;
        }
        draw_target(&info->targets[i], y);
        y += ROW_H;
    }
}

/* "When was ELRS last DETECTED", from the verdict (not the last on-grid
 * burst: one noise spike can land on the grid). */
static void status_line(const ui_info_t *info, char *line, size_t cap)
{
    if (info->last_detected_us > 0 && info->uptime_us >= info->last_detected_us) {
        const double ago = (double)(info->uptime_us - info->last_detected_us) / 1e6;
        if (ago < 600.0) {
            snprintf(line, cap, "ELRS last detected %.0f s ago", ago);
        } else {
            snprintf(line, cap, "ELRS last detected %.0f min ago", ago / 60.0);
        }
    } else {
        snprintf(line, cap, "listening %u MHz +-%u, 2.4 GHz only", info->center_mhz, info->span_mhz / 2);
    }
}

static void draw_quiet(const elrs_status_t *st, const ui_info_t *info)
{
    (void)st;
    char line[64];
    display.fillRect(0, LIST_Y - 2, display.width(), display.height() - LIST_Y + 2, COL_BG);
    display.setTextSize(1);
    if (info->n_targets == 0) {
        display.setTextColor(COL_DIM, COL_BG);
        centred("no hopping signal seen on 2.4 GHz", LIST_Y + 4);
        centred("nothing else recognised either", LIST_Y + 4 + ROW_H);
    } else {
        draw_targets(info, LIST_Y, LIST_ROWS);
    }
    display.setTextColor(COL_DIM, COL_BG);
    status_line(info, line, sizeof(line));
    centred(line, STATUS_Y);
    if (info->gain >= 0) {
        /* Not the default: keep it in sight while the arrows change it. */
        snprintf(line, sizeof(line), "fixed gain %d  [g] back to AGC", info->gain);
        centred(line, STATUS_Y + ROW_H);
    }
}

/* Below the red banner: why, how strong, then anything else seen. */
static void draw_detected(const elrs_status_t *st, const ui_info_t *info)
{
    char line[64];
    display.fillRect(0, LINES_Y, display.width(), display.height() - LINES_Y, COL_BG);
    display.setTextSize(1);
    display.setTextColor(COL_ALERT, COL_BG);
    if (info->chirp_active) {
        /* "weak": the verdict rests on several weak snapshots agreeing
         * (lora_detect.h), a lower tier of evidence: say so. */
        snprintf(line, sizeof(line), "LoRa chirps SF%u: ELRS %s%s", info->chirp_sf,
                 chirp_elrs_rate(info->chirp_sf), info->chirp_weak ? " weak" : "");
    } else {
        snprintf(line, sizeof(line), "hopping on ELRS grid, %u channels", st->channels_seen);
    }
    centred(line, LINES_Y + 4);
    if (info->chirp_active) {
        /* The AGC-proof SNR and how it moved since the detection began
         * (raw dBFS is in the details view). */
        display.setTextColor(info->chirp_delta_db >= 0.0f ? COL_WARN : COL_FG, COL_BG);
        snprintf(line, sizeof(line), "SNR %.0f dB  %+.1f dB since first", (double)info->chirp_snr_db,
                 (double)info->chirp_delta_db);
    } else {
        display.setTextColor(COL_DIM, COL_BG);
        snprintf(line, sizeof(line), "dwell ~%.0f ms, flatness %.2f", (double)st->dwell_ms,
                 (double)st->uniformity_cv);
    }
    centred(line, LINES_Y + 4 + ROW_H);
    draw_targets(info, LINES_Y + 4 + 2 * ROW_H, 2);
}

static void draw_details(const elrs_status_t *st, const ui_info_t *info)
{
    char line[64];
    display.fillScreen(COL_BG);
    display.setTextSize(1);

    display.setTextColor(st->state == ELRS_LIKELY ? COL_ALERT : COL_OK, COL_BG);
    display.setCursor(2, 1);
    snprintf(line, sizeof(line), "%s  score %.2f  snd %s", elrs_state_name(st->state), (double)st->score,
             info->sound ? info->sound : "?");
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
    /* The chirp's raw level: only comparable at the same gain index. */
    snprintf(line, sizeof(line), "grid ch %u/%u on %.0f%% raw %.0fdBFS", st->channels_seen,
             st->channels_visible, (double)(st->on_grid_ratio * 100.0f), (double)info->chirp_dbfs);
    display.print(line);
    display.setCursor(2, 62);
    snprintf(line, sizeof(line), "narrow %.0f/s wide %.0f/s dwell~%.0fms", (double)st->narrow_per_s,
             (double)st->wide_per_s, (double)st->dwell_ms);
    display.print(line);
    display.setCursor(2, 74);
    snprintf(line, sizeof(line), "chirp SF%u %u/%u snaps%s ch%u %s", info->chirp_sf, info->chirp_hits,
             info->chirp_snaps, info->chirp_weak ? " weak" : "", info->chirp_clusters,
             info->chirp_fixed ? "FIXED" : "hop");
    display.print(line);
    display.setTextColor(COL_DIM, COL_BG);
    display.setCursor(2, 86);
    snprintf(line, sizeof(line), "floor %d dB peak %d dB thr %u%s", st->floor_db, st->peak_db,
             info->threshold_db, info->invert ? " INV" : "");
    display.print(line);
    display.setCursor(2, 98);
    char gain[12];   /* "AGC" or any int */
    if (info->gain < 0) {
        snprintf(gain, sizeof(gain), "AGC");
    } else {
        snprintf(gain, sizeof(gain), "%d", info->gain);
    }
    /* Filter width and clipping: the two receiver limits esp-sdr documents
     * (docs/rx-controls.md). Slice length and drops are in the log. */
    char flt[12];
    if (info->rx_filter_mhz) {
        snprintf(flt, sizeof(flt), "%u%s", info->rx_filter_mhz, info->rx_filter_wide ? "W" : "");
    } else {
        snprintf(flt, sizeof(flt), "?");
    }
    snprintf(line, sizeof(line), "gain %s flt %sMHz clip %.0f%% crc %u", gain, flt,
             (double)(100.0f * info->chirp_clip), (unsigned)info->crc_bad);
    display.print(line);
    /* Key reference lives here, off the main view. */
    display.setCursor(2, 112);
    display.print("[d]etails [c]lear [+/-]thr [t]est [m]");
    display.setCursor(2, 124);
    display.print("[i]nv [g]AGC [;/.]gain [w]ide [r]aw");
}

extern "C" void ui_update(const elrs_status_t *st, const ui_info_t *info)
{
    display.startWrite();

    if (info->details != s_drawn_details) {
        display.fillScreen(COL_BG);
        s_drawn_details = info->details;
        s_drawn_state = -1;
    }

    if (info->details) {
        draw_details(st, info);
    } else {
        /* The banner / header only changes when the verdict does: no flicker. */
        const bool detected = st->state == ELRS_LIKELY;
        if ((int)detected != s_drawn_state) {
            display.fillScreen(COL_BG);
            if (detected) {
                draw_banner_detected();
            } else {
                draw_header_quiet();
            }
            s_drawn_state = (int)detected;
        }
        if (detected) {
            draw_detected(st, info);
        } else {
            draw_quiet(st, info);
        }
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
