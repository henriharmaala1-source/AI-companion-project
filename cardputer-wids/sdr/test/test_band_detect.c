/*
 * Host tests for band_watch.c (microwave ovens, wide occupancy) and
 * ofdm_detect.c (Wi-Fi vs LTE-like OFDM by cyclic prefix).
 *
 * Frames are built here directly (noise codes plus signals), not with the
 * simulator's renderer: these tests pin the detectors' rules; the simulator
 * (sim/) checks them inside the whole firmware.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "band_watch.h"
#include "lora_detect.h"
#include "lora_synth.h"
#include "ofdm_detect.h"

static unsigned g_fail, g_pass;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (cond) {                                       \
            g_pass++;                                     \
        } else {                                          \
            g_fail++;                                     \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);   \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

/* ------------------------------------------------------------ frames */

#define LO_KHZ    2442000u
#define FS        80000000u
#define BIN_KHZ   312.5
#define FRAME_US  2000u
#define SLICE_US  100000u
#define GAP_US    40000u      /* processing between slices */

static uint32_t g_rng = 12345u;
static double urand(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return (g_rng + 0.5) / 4294967296.0;
}

static bool g_invert;   /* frames delivered mirrored: RF above the LO lands below it */

static unsigned bin_of(double rf_khz)
{
    const int k = (g_invert ? -1 : 1) * (int)lrint((rf_khz - LO_KHZ) / BIN_KHZ);
    return k >= 0 ? (unsigned)k : (unsigned)(256 + k);
}

static void noise_frame(uint8_t *c)
{
    for (unsigned b = 0; b < 256; b++) {
        c[b] = (uint8_t)(50 + (int)(urand() * 8.0));   /* max-hold noise: a few dB of spread */
    }
    c[0] = 90;   /* LO leakage, always there */
}

static void put(uint8_t *c, double rf_khz, double width_khz, unsigned code)
{
    const int n = (int)(width_khz / BIN_KHZ);
    for (int j = -n / 2; j <= n / 2; j++) {
        const unsigned b = bin_of(rf_khz + j * BIN_KHZ);
        c[b] = c[b] > code ? c[b] : (uint8_t)code;
    }
}

typedef void (*scene_fn)(uint8_t *codes, uint64_t t_us, void *ctx);

/* Run `seconds` of slices through band_watch; returns the oven verdict at the end. */
static band_mw_t run_frames(band_watch_t *w, double seconds, scene_fn fn, void *ctx, bool invert)
{
    band_config_t cfg;
    band_default_config(&cfg, LO_KHZ, FS, 66000u);
    cfg.invert = invert;
    g_invert = invert;
    band_init(w, &cfg);
    uint8_t codes[256];
    uint64_t t = 1000000u;
    const uint64_t end = t + (uint64_t)(seconds * 1e6);
    while (t < end) {
        for (uint64_t f = t; f < t + SLICE_US; f += FRAME_US) {
            noise_frame(codes);
            fn(codes, f, ctx);
            band_push_frame(w, codes, 256, 2, f, FRAME_US);
        }
        t += SLICE_US;
        band_slice_done(w, t);
        t += GAP_US;
    }
    band_mw_t m;
    band_microwave(w, t, &m);
    return m;
}

typedef struct {
    double hz, rf_khz, sweep_khz, duty;
} oven_t;

/* On for `duty` of each mains cycle, sweeping across the on-time. */
static void oven(uint8_t *c, uint64_t t_us, void *ctx)
{
    const oven_t *o = ctx;
    const double period = 1e6 / o->hz;
    const double ph = fmod((double)t_us + 7300.0, period) / period;   /* arbitrary mains phase */
    if (ph < o->duty) {
        put(c, o->rf_khz - o->sweep_khz / 2 + o->sweep_khz * ph / o->duty, 1500.0, 110);
    }
}

static void elrs50(uint8_t *c, uint64_t t_us, void *ctx)
{
    (void)ctx;
    const uint64_t pkt = t_us / 20000u;              /* 50 Hz, a hop per packet */
    if (t_us % 20000u < 8000u) {
        const unsigned ch = (unsigned)((pkt * 37u) % 80u);
        put(c, 2400400.0 + 1000.0 * ch, 800.0, 100);
    }
}

static void wifi11(uint8_t *c, uint64_t t_us, void *ctx)
{
    (void)ctx;
    (void)t_us;
    if (urand() < 0.4) {
        put(c, 2462000.0, 18000.0, 100);
    }
}

static void carrier(uint8_t *c, uint64_t t_us, void *ctx)
{
    (void)ctx;
    (void)t_us;
    put(c, 2455000.0, 300.0, 110);
}

/* A Wi-Fi channel 11 access point that sends a frame every 20 ms (a 50 fps
 * camera): mains-like rhythm, but Wi-Fi-wide. */
static void wifi_camera50(uint8_t *c, uint64_t t_us, void *ctx)
{
    (void)ctx;
    if (t_us % 20000u < 6000u) {
        put(c, 2462000.0, 18000.0, 100);
    }
}

/* The simulator's failing case: a busy access point overlapping the oven. */
static void oven_beside_wifi(uint8_t *c, uint64_t t_us, void *ctx)
{
    oven(c, t_us, ctx);
    if (urand() < 0.3) {
        put(c, 2447000.0, 18000.0, 100);   /* channel 8 */
    }
}

/* Oven plus Bluetooth-like narrow hops all over the band. */
static void oven_beside_bt(uint8_t *c, uint64_t t_us, void *ctx)
{
    oven(c, t_us, ctx);
    put(c, 2402000.0 + 1000.0 * (double)((t_us / 625u * 37u) % 79u), 1000.0, 100);
}

static void wide_dji(uint8_t *c, uint64_t t_us, void *ctx)
{
    (void)ctx;
    (void)t_us;
    put(c, 2460500.0, 9000.0, 95);
}

static void test_microwave(void)
{
    static band_watch_t w;
    oven_t o50 = { 50.0, 2458000.0, 5000.0, 0.45 };
    band_mw_t m = run_frames(&w, 3.0, oven, &o50, false);
    CHECK(m.active, "50 Hz oven detected (r %.2f duty %.2f stable %.2f cover %.2f width %.1f, %u/%u)", (double)m.r,
          (double)m.duty, (double)m.stable, (double)m.cover, (double)m.width_mhz, m.passes, m.slices);
    CHECK(m.mains_hz == 50, "mains 50 Hz (got %u)", m.mains_hz);
    CHECK(fabsf(m.rf_mhz - 2458.0f) < 4.0f, "oven at ~2458 MHz (got %.1f)", (double)m.rf_mhz);

    oven_t o60 = { 60.0, 2455000.0, 4000.0, 0.5 };
    m = run_frames(&w, 3.0, oven, &o60, false);
    CHECK(m.active && m.mains_hz == 60, "60 Hz oven detected as 60 Hz (active %d, %u Hz)", m.active, m.mains_hz);

    /* Mirrored spectrum: the oven's raw position flips, its RF must not. */
    m = run_frames(&w, 3.0, oven, &o50, true);
    CHECK(m.active && fabsf(m.rf_mhz - 2458.0f) < 4.0f, "inverted view: oven still at ~2458 MHz (%d, %.1f)", m.active,
          (double)m.rf_mhz);

    m = run_frames(&w, 3.0, oven_beside_wifi, &o50, false);
    CHECK(m.active, "oven beside busy Wi-Fi channel 8 still detected (r %.2f duty %.2f conc %.2f, %u/%u)", (double)m.r,
          (double)m.duty, (double)m.stable, m.passes, m.slices);
    m = run_frames(&w, 3.0, oven_beside_bt, &o50, false);
    CHECK(m.active, "oven beside Bluetooth-like hopping still detected (r %.2f duty %.2f conc %.2f, %u/%u)",
          (double)m.r, (double)m.duty, (double)m.stable, m.passes, m.slices);

    m = run_frames(&w, 5.0, elrs50, NULL, false);
    CHECK(!m.active, "ELRS 50 Hz hopping is not an oven (conc %.2f r %.2f cover %.2f)", (double)m.stable, (double)m.r,
          (double)m.cover);
    m = run_frames(&w, 5.0, wifi11, NULL, false);
    CHECK(!m.active, "busy Wi-Fi channel 11 is not an oven (r %.2f duty %.2f)", (double)m.r, (double)m.duty);
    m = run_frames(&w, 5.0, wifi_camera50, NULL, false);
    CHECK(!m.active, "Wi-Fi sending at 50 Hz is not an oven: wide runs are left out (duty %.2f)", (double)m.duty);
    m = run_frames(&w, 5.0, carrier, NULL, false);
    CHECK(!m.active, "a steady carrier is not an oven (duty %.2f)", (double)m.duty);

    /* Wide occupancy: a 9 MHz continuous link lights its buckets only. */
    (void)run_frames(&w, 3.0, wide_dji, NULL, false);
    const float on = band_wide_duty(&w, 18.5f, 2.0f), off = band_wide_duty(&w, -20.0f, 2.0f);
    CHECK(on > 0.5f && off < 0.01f, "wide duty %.2f at the link, %.3f elsewhere", (double)on, (double)off);
    (void)run_frames(&w, 3.0, oven, &o50, false);
    CHECK(band_wide_duty(&w, 16.0f, 4.0f) < 0.05f, "an oven (1.5 MHz) is not 'wide' (%.3f)",
          (double)band_wide_duty(&w, 16.0f, 4.0f));
}

/* ------------------------------------------------------------ snapshots */

#define SNAP_N (3u * 12288u)
static uint32_t g_words[SNAP_N];
static float g_work[OFDM_WORK_FLOATS];

typedef enum { SIG_NONE, SIG_WIFI, SIG_LTE15, SIG_LTE30, SIG_CW, SIG_GFSK, SIG_LORA, SIG_WIFI_AND_LTE15 } sig_t;

static void make_snapshot(sig_t sig, double f_hz, double snr_db, uint32_t seed, lora_snapshot_t *snap)
{
    synth_t sy;
    synth_init(&sy, SNAP_N, seed);
    const double sigma = 8.0;
    /* Power over the noise in the whole 80 MHz, scaled to the signal's width. */
    const double amp_w = sqrt(2.0 * sigma * sigma * pow(10.0, snr_db / 10.0) * 16.6e6 / 80e6);
    const double amp_l = sqrt(2.0 * sigma * sigma * pow(10.0, snr_db / 10.0) * 9e6 / 80e6);
    const double amp_n = sqrt(2.0 * sigma * sigma * pow(10.0, snr_db / 10.0) * 1e6 / 80e6);
    switch (sig) {
    case SIG_WIFI: synth_ofdm(&sy, f_hz, amp_w); break;
    case SIG_LTE15: synth_ofdm_lte(&sy, f_hz, amp_l, 15e3, 600); break;
    case SIG_LTE30: synth_ofdm_lte(&sy, f_hz, amp_l, 30e3, 300); break;
    case SIG_CW: synth_cw(&sy, f_hz, 200.0); break;
    case SIG_GFSK: synth_gfsk(&sy, f_hz, amp_n * 3.0); break;
    case SIG_LORA: synth_lora(&sy, 5, false, f_hz, amp_n * 3.0, -0.0001, 40); break;
    case SIG_WIFI_AND_LTE15:
        synth_ofdm(&sy, 15e6, amp_w);
        synth_ofdm_lte(&sy, f_hz, amp_l, 15e3, 600);
        break;
    default: break;
    }
    synth_noise(&sy, sigma);
    (void)synth_pack(&sy, g_words);
    synth_free(&sy);
    memset(snap, 0, sizeof(*snap));
    snap->fs_hz = FS;
    snap->nseg = 1;
    snap->seg[0] = (lora_seg_t){ g_words, SNAP_N };
}

static ofdm_result_t scan(sig_t sig, double sig_hz, double slot_hz, double snr_db, uint32_t seed)
{
    lora_snapshot_t snap;
    make_snapshot(sig, sig_hz, snr_db, seed, &snap);
    ofdm_result_t r;
    const bool ok = ofdm_scan(&snap, (float)slot_hz, g_work, OFDM_WORK_FLOATS, &r);
    CHECK(ok, "ofdm_scan accepts the snapshot");
    return r;
}

/* Scores are robust z values (ofdm_detect.c): non-OFDM ~2.3 +- 0.7 per
 * scan, rarely above 5.5; the slot threshold is a running mean of 4.5. */
static void test_ofdm_single(void)
{
    ofdm_result_t r = scan(SIG_WIFI, 5e6, 5e6, 15.0, 1);
    CHECK(r.score[OFDM_WIFI] > 8.0f, "Wi-Fi: 3.2 us prefix scores %.1f", (double)r.score[OFDM_WIFI]);
    CHECK(r.narrow < OFDM_NARROW_MAX, "Wi-Fi is wideband (%.2f)", (double)r.narrow);

    r = scan(SIG_LTE15, -12.5e6, -10e6, 15.0, 2);
    CHECK(r.score[OFDM_LTE15] > 6.0f, "LTE 15 kHz, slot 2.5 MHz off: scores %.1f", (double)r.score[OFDM_LTE15]);
    CHECK(r.score[OFDM_WIFI] < 6.0f && r.score[OFDM_LTE30] < 6.0f, "LTE 15 kHz: not Wi-Fi, not 30 kHz (%.1f %.1f)",
          (double)r.score[OFDM_WIFI], (double)r.score[OFDM_LTE30]);
    CHECK(r.narrow < OFDM_NARROW_MAX, "LTE-like is wideband (%.2f)", (double)r.narrow);

    r = scan(SIG_LTE30, 20e6, 20e6, 15.0, 3);
    CHECK(r.score[OFDM_LTE30] > 6.0f, "LTE 30 kHz: scores %.1f", (double)r.score[OFDM_LTE30]);
    CHECK(r.score[OFDM_LTE15] < 6.0f, "LTE 30 kHz is not 15 kHz (%.1f)", (double)r.score[OFDM_LTE15]);

    r = scan(SIG_LTE15, -12.5e6, 20e6, 15.0, 4);
    CHECK(r.score[OFDM_LTE15] < 6.0f, "LTE seen from a slot 32 MHz away: %.1f", (double)r.score[OFDM_LTE15]);

    r = scan(SIG_WIFI_AND_LTE15, -12.5e6, -12.5e6, 15.0, 5);
    CHECK(r.score[OFDM_LTE15] > 6.0f, "LTE beside Wi-Fi: LTE slot scores %.1f", (double)r.score[OFDM_LTE15]);

    r = scan(SIG_LORA, 0.4e6, 0.0, 25.0, 6);
    CHECK(r.narrow > OFDM_NARROW_MAX, "strong LoRa fills the slot: gated as narrowband (%.2f)", (double)r.narrow);
}

/* Things that are not OFDM must never pile up into an "on" slot. */
static void test_ofdm_negatives(void)
{
    static const struct {
        sig_t sig;
        double f, snr;
        const char *name;
        int allowed;   /* the one kind this signal may turn on, -1 none */
    } cases[] = {
        { SIG_NONE, 0, 15, "noise", -1 },
        { SIG_CW, 1.2e6, 15, "strong carrier", -1 },
        { SIG_GFSK, -0.5e6, 15, "Bluetooth-like", -1 },
        { SIG_LORA, 0.4e6, 15, "LoRa SF5", -1 },
        { SIG_LORA, 0.4e6, 0, "weak LoRa SF5", -1 },
        { SIG_CW, 0.0, 15, "carrier at the slot centre", -1 },
        { SIG_WIFI, 1.0e6, 15, "Wi-Fi (may only be Wi-Fi)", OFDM_WIFI },
        { SIG_LTE15, 1.0e6, 15, "LTE 15 kHz (may only be 15 kHz)", OFDM_LTE15 },
    };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        ofdm_watch_t w;
        ofdm_watch_init(&w);
        float worst[OFDM_KINDS] = { -1, -1, -1 };
        bool any_on = false;
        for (unsigned k = 0; k < 40; k++) {
            const ofdm_result_t r = scan(cases[c].sig, cases[c].f, 0.0, cases[c].snr, 100 + 7 * k + c);
            ofdm_watch_update(&w, 6, &r, 1000000ull * (k + 1));
            for (unsigned f = 0; f < OFDM_KINDS; f++) {
                worst[f] = r.score[f] > worst[f] ? r.score[f] : worst[f];
                any_on |= (int)f != cases[c].allowed && ofdm_watch_on(&w, 6, (ofdm_kind_t)f, 1000000ull * (k + 1));
            }
        }
        CHECK(!any_on, "%s: no other OFDM kind ever on (worst single scores %.1f %.1f %.1f, %u gated narrow)",
              cases[c].name, (double)worst[0], (double)worst[1], (double)worst[2], w.narrow_skips);
    }
}

/* Weak LTE-like: per-scan scores are noisy, the slot mean must still find it. */
static void test_ofdm_weak(void)
{
    ofdm_watch_t w;
    ofdm_watch_init(&w);
    bool on = false;
    unsigned scans = 0;
    for (unsigned k = 0; k < 12 && !on; k++) {
        const ofdm_result_t r = scan(SIG_LTE15, 0.3e6, 0.0, 0.0, 300 + k);
        ofdm_watch_update(&w, 6, &r, 1000000ull * (k + 1));
        on = ofdm_watch_on(&w, 6, OFDM_LTE15, 1000000ull * (k + 1));
        scans++;
    }
    CHECK(on, "LTE 15 kHz at 0 dB in the slot found within %u scans (mean %.1f)", scans,
          (double)w.slot[6].avg[OFDM_LTE15]);
    CHECK(!ofdm_watch_on(&w, 6, OFDM_WIFI, 1000000ull * 12) && !ofdm_watch_on(&w, 6, OFDM_LTE30, 1000000ull * 12),
          "...and never called Wi-Fi or 30 kHz");
}

/* The simulator's false DJI: two wild scans among ordinary ones must not
 * make a verdict (ofdm_detect.c, k_step_max); a steady high score must. */
static void test_outliers(void)
{
    ofdm_watch_t w;
    ofdm_watch_init(&w);
    ofdm_result_t r;
    memset(&r, 0, sizeof(r));
    bool on = false;
    for (unsigned k = 0; k < 20; k++) {
        for (unsigned f = 0; f < OFDM_KINDS; f++) {
            r.score[f] = (k == 8 || k == 9) ? 15.0f : 2.3f;
        }
        ofdm_watch_update(&w, 4, &r, 1000000ull * (k + 1));
        on |= ofdm_watch_on(&w, 4, OFDM_LTE30, 1000000ull * (k + 1));
    }
    CHECK(!on, "two wild scans (z 15) among ordinary ones: no verdict (mean peaked below threshold)");
    ofdm_watch_init(&w);
    unsigned scans = 0;
    for (on = false; scans < 12 && !on; scans++) {
        for (unsigned f = 0; f < OFDM_KINDS; f++) {
            r.score[f] = 9.0f;
        }
        ofdm_watch_update(&w, 4, &r, 1000000ull * (scans + 1));
        on = ofdm_watch_on(&w, 4, OFDM_LTE30, 1000000ull * (scans + 1));
    }
    CHECK(on && scans <= 8, "a steady z of 9 is on after %u scans", scans);
}

static void test_pick(void)
{
    ofdm_watch_t w;
    ofdm_watch_init(&w);
    float duty[OFDM_SLOTS] = { 0 };
    duty[3] = 0.5f;
    duty[9] = 0.3f;
    unsigned slots[2];
    unsigned n = ofdm_watch_pick(&w, duty, 0.05f, 2, slots);
    CHECK(n == 2 && slots[0] == 3 && slots[1] != 9, "a wide slot, then any slot (%u: %u %u)", n, slots[0], slots[1]);
    ofdm_result_t r;
    memset(&r, 0, sizeof(r));
    ofdm_watch_update(&w, 3, &r, 100);
    ofdm_watch_update(&w, 9, &r, 200);   /* two scans: the next pick is a "wide" one again */
    n = ofdm_watch_pick(&w, duty, 0.05f, 1, slots);
    CHECK(n == 1 && slots[0] == 3, "least recently scanned wide slot next (%u)", slots[0]);
    ofdm_watch_update(&w, 3, &r, 300);
    n = ofdm_watch_pick(&w, duty, 0.05f, 1, slots);
    CHECK(n == 1 && slots[0] != 3 && slots[0] != 9, "then a slot regardless of activity (%u)", slots[0]);
    memset(duty, 0, sizeof(duty));
    n = ofdm_watch_pick(&w, duty, 0.05f, 1, slots);
    CHECK(n == 1 && slots[0] != 3 && slots[0] != 9, "no wide slot: an unscanned slot (%u)", slots[0]);
}

int main(void)
{
    test_microwave();
    test_ofdm_single();
    test_ofdm_negatives();
    test_ofdm_weak();
    test_outliers();
    test_pick();
    printf("band_detect: %u passed, %u failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
