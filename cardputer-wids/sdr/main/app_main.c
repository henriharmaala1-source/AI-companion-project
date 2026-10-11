/*
 * app_main.c - ELRS watch mode for the Cardputer.
 *
 * RECEIVE ONLY, PRESENCE ONLY, 2.4 GHz ONLY. See docs/ELRS_SDR_PLAN.md.
 *
 * The loop alternates two phases, because the RF engine masks interrupts
 * while it captures:
 *
 *   [ capture slice, 100 ms, IRQs masked ] -> ~2 ms frames land in a RAM sink
 *   [ decode + detect + draw + keys + log, interrupts on ]
 *
 * The host simulator (sim/) measures 93 % of wall time spent listening.
 * ELRS holds each channel for 2-40 ms and revisits every channel once per 80
 * hops, so the gaps cost detection latency, not detection.
 *
 * Log lines are NDJSON in the same shape as the Wi-Fi monitor
 * ({"t":<us since boot>,"ev":...}) on the USB Serial/JTAG port.
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "chirp_watch.h"
#include "cp_keyboard.h"
#include "elrs_detect.h"
#include "sdr_engine.h"
#include "spc1.h"
#include "ui.h"
#include "wids_log.h"

/* Receiver plan. 2442 MHz is Wi-Fi channel 7, so upstream tunes it through
 * the PHY's own channel path. +-33 MHz usable covers ELRS channels ~8..75. */
#define CENTER_MHZ        2442u
#define RATE_CODE         SDR_RATE_80MSPS
#define NFFT              256u
/* One max-hold frame per 2 ms: as short as the fastest ELRS dwell (F1000).
 * The sink's pace sets this, not the engine; see sdr_sink_pace(). */
#define FRAME_US          2000u
#define SLICE_MS          100u

/* The sink must take a whole slice of frames, plus the one being drained and
 * the one closed when the slice ends. */
_Static_assert((SLICE_MS * 1000u / FRAME_US + 2u) * SDR_FRAME_BYTES(NFFT) <= SDR_SINK_BYTES,
               "sink too small for one slice of frames");
#define UI_PERIOD_US      250000
#define STATS_PERIOD_US   5000000
/* Diagnostic stream for tuning on real hardware: where bursts land ("diag",
 * every 2 s) and the strongest raw spectrum ("frame", every 5 s, two lines
 * because a log line holds 512 characters). */
#define DIAG_PERIOD_US    2000000
#define FRAME_PERIOD_US   5000000
#define FRAME_PART_BINS   128u

typedef struct {
    elrs_detector_t det;
    spc1_decoder_t  dec;
    elrs_config_t   cfg;
    int64_t         slice_start_us;
    unsigned        fs_hz;
    /* strongest frame since the last "frame" record */
    uint8_t         peak_codes[NFFT];
    uint8_t         peak_max;
    uint64_t        peak_t_us;
    uint32_t        peak_pairs;
    uint16_t        peak_ffts;
    uint8_t         peak_gain, peak_flags;
    /* gain metadata seen since the last "diag" record (AGC movement) */
    uint8_t         gain_lo, gain_hi;
    bool            gain_seen;
    int             gain;          /* -1 = hardware AGC, else fixed index */
} app_t;

static app_t *s_app; /* heap, not BSS: static data must end below the RF ring */

/* ------------------------------------------------------------ logging */

static void usb_sink(const char *data, size_t len)
{
    /* Non-blocking: with no host attached the bytes are simply dropped. */
    (void)usb_serial_jtag_write_bytes(data, len, 0);
}

/* ------------------------------------------------------------ frames */

static void on_frame(const spc1_frame_t *f, void *ctx)
{
    app_t *a = ctx;
    /* pair_index restarts at 0 every slice; anchor it to wall time. */
    const uint64_t t_us = (uint64_t)a->slice_start_us + f->pair_index * 1000000ull / a->fs_hz;
    const uint32_t dur_us = (uint32_t)((uint64_t)f->pairs * 1000000ull / a->fs_hz);
    elrs_push_frame(&a->det, f->codes, f->bins, f->db_step, t_us, dur_us);

    if (!a->gain_seen || f->gain < a->gain_lo) {
        a->gain_lo = f->gain;
    }
    if (!a->gain_seen || f->gain > a->gain_hi) {
        a->gain_hi = f->gain;
    }
    a->gain_seen = true;

    if (f->bins == NFFT) {
        uint8_t max = 0;
        for (unsigned i = 0; i < NFFT; i++) {
            max = f->codes[i] > max ? f->codes[i] : max;
        }
        if (max > a->peak_max) {
            memcpy(a->peak_codes, f->codes, NFFT);
            a->peak_max = max;
            a->peak_t_us = t_us;
            a->peak_pairs = f->pairs;
            a->peak_ffts = f->ffts;
            a->peak_gain = f->gain;
            a->peak_flags = f->flags;
        }
    }
}

static void drain_sink(app_t *a)
{
    uint8_t buf[512];
    size_t n;
    while ((n = sdr_engine_read(buf, sizeof(buf))) > 0) {
        spc1_feed(&a->dec, buf, n);
    }
}

/* ------------------------------------------------------------ helpers */

static void restart_detector(app_t *a, const char *why)
{
    elrs_init(&a->det, &a->cfg);
    wids_log_event("elrs_config",
                   "\"why\":\"%s\",\"center_khz\":%" PRIu32 ",\"fs\":%" PRIu32
                   ",\"threshold_db\":%u,\"invert\":%s,\"gain\":%d,\"band\":\"2.4GHz-only\",\"tx\":\"none\"",
                   why, a->cfg.center_khz, a->cfg.sample_rate_hz, (unsigned)a->cfg.threshold_db,
                   a->cfg.invert ? "true" : "false", a->gain);
}

/* "[1,2,3]" into out; always NUL-terminated. */
static void fmt_counts(char *out, size_t cap, const uint32_t *v, size_t n)
{
    size_t w = 0;
    out[0] = '\0';
    for (size_t i = 0; i < n && w + 1 < cap; i++) {
        const int k = snprintf(out + w, cap - w, "%c%" PRIu32, i ? ',' : '[', v[i]);
        if (k < 0) {
            return;
        }
        w = w + (size_t)k < cap ? w + (size_t)k : cap - 1;
    }
    if (w + 1 < cap) {
        out[w++] = ']';
        out[w] = '\0';
    }
}

static void log_diag(app_t *a)
{
    elrs_diag_t dg;
    char phase[160], width[128];
    elrs_diag_take(&a->det, &dg);
    fmt_counts(phase, sizeof(phase), dg.phase, ELRS_DIAG_PHASE_BUCKETS);
    fmt_counts(width, sizeof(width), dg.width, ELRS_DIAG_WIDTH_BUCKETS);
    wids_log_event("diag",
                   "\"phase_from_khz\":%d,\"phase_step_khz\":%d,\"phase\":%s,\"width_bins\":%s,"
                   "\"clipped\":%" PRIu32 ",\"inv\":%s,\"thr_db\":%u,\"gain\":%d,"
                   "\"gain_meta_lo\":%u,\"gain_meta_hi\":%u",
                   -500, ELRS_DIAG_PHASE_KHZ, phase, width, dg.clipped, a->cfg.invert ? "true" : "false",
                   (unsigned)a->cfg.threshold_db, a->gain, a->gain_lo, a->gain_hi);
    a->gain_seen = false;
}

/* The strongest frame of the period, raw codes in natural FFT order (bin 0 =
 * LO, then +f, then -f), 0.5 dB per code. */
static void log_peak_frame(app_t *a)
{
    if (a->peak_max == 0) {
        return;
    }
    char hex[2 * FRAME_PART_BINS + 1];
    for (unsigned part = 0; part < NFFT / FRAME_PART_BINS; part++) {
        wids_hex_encode(a->peak_codes + part * FRAME_PART_BINS, FRAME_PART_BINS, hex, sizeof(hex));
        wids_log_event("frame",
                       "\"t_frame\":%" PRIu64 ",\"part\":%u,\"of\":%u,\"bins\":%u,\"pairs\":%" PRIu32
                       ",\"ffts\":%u,\"gain_meta\":%u,\"flags\":%u,\"gain\":%d,\"order\":\"natural\","
                       "\"hex\":\"%s\"",
                       a->peak_t_us, part, NFFT / FRAME_PART_BINS, NFFT, a->peak_pairs, a->peak_ffts,
                       a->peak_gain, a->peak_flags, a->gain, hex);
    }
    a->peak_max = 0;
}

static void fatal(const char *a, const char *b)
{
    ui_message(a, b);
    wids_log_event("error", "\"msg\":\"%s %s\"", a, b);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ------------------------------------------------------------ main */

void app_main(void)
{
    /* Upstream keeps ESP-IDF logging silent so the serial line carries only
     * protocol bytes; here it carries only NDJSON. */
    esp_log_level_set("*", ESP_LOG_NONE);

    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    usb_serial_jtag_driver_config_t usb = { .tx_buffer_size = 4096, .rx_buffer_size = 256 };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    wids_log_init();
    wids_log_add_sink(usb_sink);

    ui_init();
    cp_kb_init();
    ui_message("starting receiver...", "receive only, 2.4 GHz");

    s_app = heap_caps_calloc(1, sizeof(app_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_app == NULL) {
        fatal("out of memory", "detector state");
    }
    app_t *a = s_app;
    a->gain = -1;   /* hardware AGC until 'g' is pressed */

    if (sdr_engine_init(CENTER_MHZ) != ESP_OK) {
        fatal("radio init failed", "check IDF pin / build");
    }
    a->fs_hz = sdr_engine_rate_hz(RATE_CODE);
    chirp_watch_init(RATE_CODE);
    elrs_default_config(&a->cfg, CENTER_MHZ * 1000u, a->fs_hz);
    spc1_init(&a->dec, on_frame, a);
    restart_detector(a, "boot");

    wids_log_event("boot",
                   "\"mode\":\"elrs_watch\",\"dual_core\":%s,\"heap_free\":%u,\"heap_largest\":%u,"
                   "\"sink_bytes\":%u",
                   sdr_engine_dual_core() ? "true" : "false",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                   (unsigned)sdr_engine_sink_capacity());

    const sdr_spec_cfg_t spec = {
        .rate_code = RATE_CODE, .nfft = NFFT, .frame_us = FRAME_US, .max_hold = true,
    };
    const uint32_t slice_ms = SLICE_MS;
    uint32_t drops_total = 0;
    int64_t next_ui = 0, next_stats = 0, next_diag = 0, next_frame = 0;
    elrs_status_t st;
    memset(&st, 0, sizeof(st));
    bool details = false;
    elrs_state_t shown_prev = ELRS_QUIET;
    uint64_t last_detected_us = 0;

    for (;;) {
        sdr_slice_result_t r;
        a->slice_start_us = esp_timer_get_time();
        sdr_engine_run_slice(&spec, slice_ms, &r);  /* interrupts masked in here */
        drain_sink(a);
        drops_total += r.drops;

        if (r.status != 0) {
            wids_log_event("sdr_error", "\"status\":%" PRIu32 ",\"detail\":%" PRIu32, r.status, r.detail);
        }
        /* Raw snapshot + chirp test, in the sink's RAM while it is empty. */
        chirp_watch_step((uint64_t)esp_timer_get_time());

        const int64_t now = esp_timer_get_time();
        if (elrs_evaluate(&a->det, (uint64_t)now, &st)) {
            wids_log_event("elrs",
                           "\"state\":\"%s\",\"score\":%.2f,\"grid_seen\":%u,\"grid_visible\":%u,"
                           "\"on_grid\":%.2f,\"int_grid\":%.2f,\"cv\":%.2f,\"dwell_ms\":%.0f,"
                           "\"band\":\"2.4GHz-only\"",
                           elrs_state_name(st.state), (double)st.score, st.channels_seen,
                           st.channels_visible, (double)st.on_grid_ratio, (double)st.int_grid_ratio,
                           (double)st.uniformity_cv, (double)st.dwell_ms);
        }

        /* What the screen says: ELRS if either the grid detector or the
         * chirp detector says so. */
        chirp_status_t cs;
        chirp_watch_status((uint64_t)now, &cs);
        elrs_status_t shown = st;
        if (cs.active) {
            shown.state = ELRS_LIKELY;
        }
        if (shown.state != shown_prev) {
            const bool by_grid = st.state == ELRS_LIKELY;
            wids_log_event("verdict", "\"state\":\"%s\",\"by\":\"%s\",\"chirp_sf\":%u,\"band\":\"2.4GHz-only\"",
                           elrs_state_name(shown.state),
                           cs.active && by_grid ? "chirp+grid" : cs.active ? "chirp" : by_grid ? "grid" : "none",
                           cs.active ? cs.sf : 0u);
            if (shown.state == ELRS_LIKELY) {
                ui_alert();
                next_ui = 0;
            }
            shown_prev = shown.state;
        }
        if (shown.state == ELRS_LIKELY) {
            last_detected_us = (uint64_t)now;
        }

        switch (cp_kb_poll()) {
        case 'c':
            restart_detector(a, "clear");
            break;
        case '=': /* the '+' key unshifted */
        case '+':
            if (a->cfg.threshold_db < 30) {
                a->cfg.threshold_db++;
                restart_detector(a, "threshold");
            }
            break;
        case '-':
            if (a->cfg.threshold_db > 4) {
                a->cfg.threshold_db--;
                restart_detector(a, "threshold");
            }
            break;
        case 'i': /* spectrum orientation: confirm on hardware (plan M0) */
            a->cfg.invert = !a->cfg.invert;
            restart_detector(a, "invert");
            break;
        case 'g': { /* AGC -> fixed 10, 20 ... 60 -> AGC: find a gain that does not overload */
            static const int k_gains[] = { -1, 10, 20, 30, 40, 50, 60 };
            const size_t n = sizeof(k_gains) / sizeof(k_gains[0]);
            size_t i = 0;
            while (i < n && k_gains[i] != a->gain) {
                i++;
            }
            a->gain = sdr_engine_set_gain(k_gains[(i + 1) % n]);
            restart_detector(a, "gain");
            next_ui = 0;
            break;
        }
        case 'r': /* log the next raw snapshot, to check the dechirp offline */
            chirp_watch_request_dump();
            break;
        case 't':
            ui_alert();
            next_ui = 0;
            break;
        case 'd': /* big answer <-> detector numbers */
            details = !details;
            next_ui = 0;
            break;
        default:
            break;
        }

        if (now >= next_ui) {
            const ui_info_t info = {
                .center_mhz = CENTER_MHZ, .span_mhz = a->cfg.usable_khz / 1000u,
                .threshold_db = a->cfg.threshold_db, .invert = a->cfg.invert, .slice_ms = slice_ms,
                .drops = drops_total, .crc_bad = a->dec.crc_bad, .uptime_us = (uint64_t)now, .gain = a->gain,
                .details = details, .chirp_active = cs.active, .chirp_sf = cs.sf, .chirp_hits = cs.hits,
                .chirp_snaps = cs.snaps, .chirp_last_us = cs.last_hit_us, .last_detected_us = last_detected_us,
            };
            ui_update(&shown, &info);
            next_ui = now + UI_PERIOD_US;
        }
        if (now >= next_stats) {
            wids_log_event("sdr_stats",
                           "\"frames\":%" PRIu32 ",\"drops\":%" PRIu32 ",\"abandoned\":%" PRIu32
                           ",\"ffts\":%" PRIu32 ",\"slice_ms\":%" PRIu32 ",\"crc_bad\":%" PRIu32
                           ",\"elapsed_us\":%" PRIu64 ",\"sink_used\":%u",
                           r.frames, r.drops, r.abandoned, r.ffts, slice_ms, a->dec.crc_bad,
                           r.elapsed_us, (unsigned)r.sink_used);
            next_stats = now + STATS_PERIOD_US;
        }
        if (now >= next_diag) {
            log_diag(a);
            chirp_watch_log((uint64_t)now);
            next_diag = now + DIAG_PERIOD_US;
        }
        if (now >= next_frame) {
            log_peak_frame(a);
            next_frame = now + FRAME_PERIOD_US;
        }

        /* Let the USB driver, esp_timer task and idle task run: the tick
         * was frozen for the whole slice. */
        vTaskDelay(1);
    }
}
