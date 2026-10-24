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
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);

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
static bool     s_gain_manual;      /* false: hardware AGC */
static unsigned s_gain_code = 40;   /* upstream's default index */

static void gain_apply(void)
{
    if (s_gain_code > gain_max()) {
        s_gain_code = gain_max();
    }
    force_rx_gain(s_gain_manual ? 1u : 0u, s_gain_code, 0);
}

/* ---- analog filter (upstream rx_filter_apply / rx_filter_restore) -------
 * BBTOP block 0x67, registers 4 and 5, low 6 bits. Applied only around a
 * run and restored after it, because calibration reads these registers. */
static int      s_rx_filter = -1;      /* -1: PHY-calibrated setting */
static unsigned s_rx_filter_saved[2];
static int      s_rx_filter_seen = -1; /* code in force during the last run */

static void rx_filter_apply(void)
{
    for (unsigned j = 0; j < 2; j++) {
        s_rx_filter_saved[j] = rom_chip_i2c_readReg(0x67, 0, 4 + j);
        if (s_rx_filter >= 0) {
            rom_chip_i2c_writeReg(0x67, 0, 4 + j, (s_rx_filter_saved[j] & ~63u) | (unsigned)s_rx_filter);
        }
    }
    s_rx_filter_seen = s_rx_filter >= 0 ? s_rx_filter : (int)(s_rx_filter_saved[0] & 63u);
}

static void rx_filter_restore(void)
{
    if (s_rx_filter >= 0) {
        for (unsigned j = 0; j < 2; j++) {
            rom_chip_i2c_writeReg(0x67, 0, 4 + j, s_rx_filter_saved[j]);
        }
    }
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
    rx_filter_apply();
    ring_capture_run(&c, &r);
    rx_filter_restore();

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

/* Upstream RINGCAP: capture `units` consecutive ring units and leave them in
 * the banks. Same receiver setup as the spectrum slices. */
void sdr_engine_capture(unsigned rate_code, unsigned units, sdr_capture_t *out)
{
    memset(out, 0, sizeof(*out));
    ring_config_t c;
    memset(&c, 0, sizeof(c));
    c.mode = RING_MODE_CAPTURE;
    c.rate = rate_code;
    c.capture_units = units < 1 ? 1 : units > RING_BANKS ? RING_BANKS : units;

    ring_result_t r;
    prepare_rx();
    rx_filter_apply();
    ring_capture_run(&c, &r);
    rx_filter_restore();
    out->status = r.status;
    out->fs_hz = ring_capture_rate_hz(rate_code);
    if (r.status != 0) {
        return;
    }
    /* Same unit walk as upstream ring_send_capture(). */
    for (unsigned u = 0; u < r.units && u < RING_BANKS; u++) {
        const uint32_t *p = ring_capture_bank(r.cap[u].bank);
        const uint32_t first = r.cap[u].first, n = r.cap[u].count;
        uint32_t head = RING_PAIRS - first;
        if (head > n) {
            head = n;
        }
        out->seg[out->nseg] = p + first;
        out->len[out->nseg++] = head;
        if (n > head) {
            out->seg[out->nseg] = p;
            out->len[out->nseg++] = n - head;
        }
    }
}

void *sdr_engine_scratch(size_t *bytes) { return sdr_sink_scratch(bytes); }

/* Upstream gain_reconfigure(): a forced index alone can leave continuous
 * capture on stale RX state, so redo the whole receive setup. */
int sdr_engine_set_gain(int code)
{
    s_gain_manual = code >= 0;
    if (s_gain_manual) {
        s_gain_code = (unsigned)code;
    }
    s_rx_ready = false;
    prepare_rx();
    return s_gain_manual ? (int)s_gain_code : -1;
}

unsigned sdr_engine_gain_max(void) { return gain_max(); }

void sdr_engine_set_rx_filter(int code) { s_rx_filter = code < 0 ? -1 : code > 63 ? 63 : code; }
int sdr_engine_rx_filter_code(void) { return s_rx_filter_seen; }

size_t sdr_engine_read(uint8_t *dst, size_t cap) { return sdr_sink_read(dst, cap); }
size_t sdr_engine_sink_capacity(void) { return sdr_sink_capacity(); }
unsigned sdr_engine_rate_hz(unsigned rate_code) { return ring_capture_rate_hz(rate_code); }
bool sdr_engine_dual_core(void) { return ring_capture_dual_active(); }
unsigned sdr_engine_center_mhz(void) { return s_frequency_mhz; }
