/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * s3_rx.c - ESP32-S3 receiver bring-up for the Cardputer.
 *
 * Extracted from esp-sdr main/targets/esp32s3/receiver.c at commit
 * e74f2a470972ec163c247f1fe88d32e08579242a (https://github.com/ESPARGOS/esp-sdr).
 * Kept: memory reservation, tuning, gain, receive-path setup, app_main's
 * radio init sequence. Dropped: the serial command protocol, the single-shot
 * capture path, GPIO control, and every diagnostic/probe build option.
 *
 * Statements copied from upstream are kept as close to the original as
 * possible so a future upstream diff stays readable. Do not "tidy" them: the
 * order of these PHY calls is load-bearing and only validated by upstream on
 * their pinned IDF.
 *
 * RECEIVE ONLY. Nothing here enables the transmitter; prepare_rx() explicitly
 * stops any test tone and powers the TX path down, as upstream does.
 */
#include <stdbool.h>
#include <string.h>

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "heap_memory_layout.h"
#include "soc/soc.h"

#include "ring_capture.h"
#include "rx_lo.h"
#include "rx_recalibration.h"
#include "sdr_engine.h"
#include "sdr_sink.h"

/* Bytes of spectrum the app can buffer per slice. 256-bin frames are 288 B,
 * so this holds ~42 frames; the engine's own 16 KiB queue holds more. */

/* The RF writer owns three 64 KiB SRAM banks during capture. Keep them out of
 * the heap and of static sections (upstream sram_guard.ld enforces the
 * latter). This object is always linked because sdr_engine_init() lives in it;
 * if it were dropped the heap would silently overlap the RF ring. */
SOC_RESERVE_MEMORY_REGION(RING_BANK_BASE, RING_BANK_END, s3_rf_dump);

/* PHY / ROM entry points used by upstream (no public headers). */
extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void set_rf_freq_offset(unsigned, unsigned, int);
extern void force_rx_gain(unsigned, unsigned, unsigned);

/* Required by the stock RF test archive; no shell is exposed. (upstream) */
int cmd_parse(char *cmd, char *name, int *argc, char **argv)
{
    (void)cmd; (void)name; (void)argc; (void)argv;
    return -1;
}

static unsigned s_frequency_mhz = 2442;
static bool s_rx_ready;

/* ---- tuning (upstream s3_tune, with FOFS fixed at 0) -------------------- */
static void s3_tune(unsigned mhz)
{
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    /* Wi-Fi channel centres use the PHY's own channel path; anything else
     * goes through the PLL offset. 2442 MHz (channel 7) takes the first. */
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    rx_lo_select(false);
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) {
        set_rf_freq_offset(0, plan.mhz, plan.offset_khz);
    }
}

/* ---- gain (upstream burst_gain.h, hardware AGC mode only) --------------- */
#define BURST_GAIN_REG 0x6001c02cu
static unsigned gain_max(void)
{
    unsigned maximum = (REG_READ(BURST_GAIN_REG) >> 8) & 127u;
    return maximum <= 82u ? maximum : 0u; /* never expose uncalibrated slots */
}
static void gain_apply(void)
{
    unsigned code = 40;
    if (code > gain_max()) {
        code = gain_max();
    }
    force_rx_gain(0 /* hardware AGC */, code, 0);
}

/* ---- receive path (upstream prepare_rx, default rx_prep = 3) ------------ */
static void prepare_rx(void)
{
    if (s_rx_ready && !rx_recalibration_stale()) {
        return;
    }
    s3_tune(s_frequency_mhz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    gain_apply();
    rx_lo_select(rx_lo_plan(s_frequency_mhz).alternate);
    esp_rom_delay_us(3000);
    s_rx_ready = true;
}

/* ---- public API --------------------------------------------------------- */

#define TRY(x) do { esp_err_t err_ = (x); if (err_ != ESP_OK) { return err_; } } while (0)

esp_err_t sdr_engine_init(unsigned center_mhz)
{
    s_frequency_mhz = center_mhz;
    if (!sdr_sink_init(SDR_SINK_BYTES)) {
        return ESP_ERR_NO_MEM;
    }
    /* upstream app_main radio sequence: NULL mode, promiscuous on, ch 1. */
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        return e;
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    TRY(esp_wifi_init(&cfg));
    TRY(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    TRY(esp_wifi_set_mode(WIFI_MODE_NULL));
    TRY(esp_wifi_start());
    TRY(esp_wifi_set_ps(WIFI_PS_NONE));
    TRY(esp_wifi_set_promiscuous(true));
    TRY(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    ring_capture_init();
    return ring_capture_valid_nfft(256) ? ESP_OK : ESP_FAIL;
}

void sdr_engine_run_slice(const sdr_spec_cfg_t *cfg, uint32_t duration_ms, sdr_slice_result_t *out)
{
    ring_config_t c;
    memset(&c, 0, sizeof(c));
    c.mode = RING_MODE_SPEC;
    c.rate = cfg->rate_code;
    c.nfft = cfg->nfft;
    c.duration_ms = duration_ms ? duration_ms : 1; /* 0 would mean "until stopped" */
    c.stride = 1;
    c.units_per_frame = 1;   /* must be non-zero; the S3 path ignores it */
    c.max_hold = cfg->max_hold;
    c.stats = false;

    ring_result_t r;
    prepare_rx();
    sdr_sink_pace(SDR_FRAME_BYTES(cfg->nfft), cfg->frame_us);
    ring_capture_run(&c, &r);

    memset(out, 0, sizeof(*out));
    out->status = r.status;
    out->detail = r.detail;
    out->frames = r.frames;
    out->drops = r.drops;
    out->abandoned = r.abandoned;
    out->ffts = r.ffts;
    out->pairs = r.pairs;
    out->elapsed_us = r.elapsed_us;
    out->sink_used = sdr_sink_used();
}

size_t sdr_engine_read(uint8_t *dst, size_t cap) { return sdr_sink_read(dst, cap); }
size_t sdr_engine_sink_capacity(void) { return sdr_sink_capacity(); }
unsigned sdr_engine_rate_hz(unsigned rate_code) { return ring_capture_rate_hz(rate_code); }
bool sdr_engine_dual_core(void) { return ring_capture_dual_active(); }
unsigned sdr_engine_center_mhz(void) { return s_frequency_mhz; }
