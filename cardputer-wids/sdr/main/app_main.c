/*
 * app_main.c - ELRS watch mode for the Cardputer.
 *
 * RECEIVE ONLY, PRESENCE ONLY, 2.4 GHz ONLY. See docs/ELRS_SDR_PLAN.md.
 *
 * The loop alternates two phases, because the RF engine masks interrupts
 * while it captures:
 *
 *   [ capture slice, ~100 ms, IRQs masked ] -> frames land in a RAM sink
 *   [ decode + detect + draw + keys + log, interrupts on ]
 *
 * Roughly 80-85 % of wall time is spent listening. ELRS holds each channel for
 * 2-40 ms and revisits every channel once per 80 hops, so the gaps cost
 * detection latency, not detection.
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
#define UNITS_PER_FRAME   12u      /* ~1.8 ms per frame at 80 MS/s */

#define SLICE_MS_START    100u
#define SLICE_MS_MIN      30u
#define SLICE_MS_MAX      150u
#define UI_PERIOD_US      250000
#define STATS_PERIOD_US   5000000

typedef struct {
    elrs_detector_t det;
    spc1_decoder_t  dec;
    elrs_config_t   cfg;
    int64_t         slice_start_us;
    unsigned        fs_hz;
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
                   ",\"threshold_db\":%u,\"invert\":%s,\"band\":\"2.4GHz-only\",\"tx\":\"none\"",
                   why, a->cfg.center_khz, a->cfg.sample_rate_hz, (unsigned)a->cfg.threshold_db,
                   a->cfg.invert ? "true" : "false");
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

    if (sdr_engine_init(CENTER_MHZ) != ESP_OK) {
        fatal("radio init failed", "check IDF pin / build");
    }
    a->fs_hz = sdr_engine_rate_hz(RATE_CODE);
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
        .rate_code = RATE_CODE, .nfft = NFFT, .units_per_frame = UNITS_PER_FRAME, .max_hold = true,
    };
    uint32_t slice_ms = SLICE_MS_START;
    uint32_t drops_total = 0;
    int64_t next_ui = 0, next_stats = 0;
    elrs_status_t st;
    memset(&st, 0, sizeof(st));
    bool details = false;

    for (;;) {
        sdr_slice_result_t r;
        a->slice_start_us = esp_timer_get_time();
        sdr_engine_run_slice(&spec, slice_ms, &r);  /* interrupts masked in here */
        drain_sink(a);
        drops_total += r.drops;

        if (r.status != 0) {
            wids_log_event("sdr_error", "\"status\":%" PRIu32 ",\"detail\":%" PRIu32, r.status, r.detail);
        }

        /* Size the slice to what the sink can hold: shorter when frames were
         * dropped, slowly longer when there was room to spare. */
        if (r.drops > 0 && slice_ms > SLICE_MS_MIN) {
            slice_ms -= 10;
        } else if (r.drops == 0 && r.sink_used < sdr_engine_sink_capacity() / 2 && slice_ms < SLICE_MS_MAX) {
            slice_ms += 5;
        }

        const int64_t now = esp_timer_get_time();
        if (elrs_evaluate(&a->det, (uint64_t)now, &st)) {
            wids_log_event("elrs",
                           "\"state\":\"%s\",\"score\":%.2f,\"grid_seen\":%u,\"grid_visible\":%u,"
                           "\"on_grid\":%.2f,\"cv\":%.2f,\"dwell_ms\":%.0f,\"band\":\"2.4GHz-only\"",
                           elrs_state_name(st.state), (double)st.score, st.channels_seen,
                           st.channels_visible, (double)st.on_grid_ratio, (double)st.uniformity_cv,
                           (double)st.dwell_ms);
            if (st.state == ELRS_LIKELY) {
                ui_alert();
                next_ui = 0;
            }
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
                .drops = drops_total, .crc_bad = a->dec.crc_bad, .uptime_us = (uint64_t)now,
                .details = details,
            };
            ui_update(&st, &info);
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

        /* Let the USB driver, esp_timer task and idle task run: the tick
         * was frozen for the whole slice. */
        vTaskDelay(1);
    }
}
