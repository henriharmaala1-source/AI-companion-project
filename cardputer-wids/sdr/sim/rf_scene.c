/*
 * rf_scene.c - see rf_scene.h for what is and is not modelled.
 *
 * How one window is rendered:
 *   1. every transmitter lists the transmissions that overlap [t0, t1);
 *   2. each transmission spreads its power over FFT bins through the Hann
 *      window's response (so a 0.8 MHz burst lights ~3 bins, with skirts);
 *   3. each bin gets a max-hold noise sample for `ffts` FFTs, and the code is
 *      2 * 10*log10(power), esp-sdr's 0.5 dB steps.
 *
 * Random choices that belong to a transmission (which channel, how long) come
 * from a hash of its index, never from a running generator, so a packet that
 * straddles two windows looks the same in both.
 */
#include "rf_scene.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------ randomness */

static uint32_t hash32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

static double hash_unit(uint32_t a, uint32_t b) /* [0, 1) */
{
    return hash32(a * 0x9e3779b9u ^ hash32(b)) / 4294967296.0;
}

static uint32_t s_noise_state = 0x1234567u;
static double noise_unit(void) /* (0, 1) */
{
    s_noise_state ^= s_noise_state << 13;
    s_noise_state ^= s_noise_state >> 17;
    s_noise_state ^= s_noise_state << 5;
    return (s_noise_state + 0.5) / 4294967296.0;
}

/* ------------------------------------------------------------ Hann leakage */

/* Cumulative integral of the Hann window's power response |H(x)|^2, x in bins,
 * normalised so a signal much wider than a bin puts exactly 1.0 in each bin. */
#define LEAK_SPAN  8.0
#define LEAK_STEP  0.01
#define LEAK_N     1601  /* 2 * LEAK_SPAN / LEAK_STEP + 1 */

static double s_leak_cum[LEAK_N];
static bool   s_leak_ready;

static double hann_power(double x)
{
    if (fabs(x) < 1e-9) {
        return 1.0;
    }
    if (fabs(fabs(x) - 1.0) < 1e-9) {
        return 0.25; /* limit of the amplitude, 0.5, squared */
    }
    const double sinc = sin(M_PI * x) / (M_PI * x);
    const double h = sinc / (1.0 - x * x);
    return h * h;
}

static void leak_init(void)
{
    double acc = 0.0;
    s_leak_cum[0] = 0.0;
    for (int i = 1; i < LEAK_N; i++) {
        const double x0 = -LEAK_SPAN + (i - 1) * LEAK_STEP;
        acc += 0.5 * (hann_power(x0) + hann_power(x0 + LEAK_STEP)) * LEAK_STEP;
        s_leak_cum[i] = acc;
    }
    for (int i = 0; i < LEAK_N; i++) {
        s_leak_cum[i] /= acc;  /* acc is the equivalent noise bandwidth, ~1.5 bins */
    }
    s_leak_ready = true;
}

static double leak_cum(double x)
{
    if (x <= -LEAK_SPAN) {
        return 0.0;
    }
    if (x >= LEAK_SPAN) {
        return 1.0;
    }
    const double pos = (x + LEAK_SPAN) / LEAK_STEP;
    const int i = (int)pos;
    if (i >= LEAK_N - 1) {
        return 1.0;
    }
    const double f = pos - i;
    return s_leak_cum[i] * (1.0 - f) + s_leak_cum[i + 1] * f;
}

/* ------------------------------------------------------------ spreading */

typedef struct {
    const rf_view_t *v;
    double bin_khz;
    float *lin;  /* signal power per bin, noise mean = 1.0 */
} render_t;

/* ASSUMPTION: the analogue filter is flat to +-30 MHz and then falls about
 * 1.2 dB per MHz. Real numbers come from M0 recordings. */
static double rolloff_db(double offset_khz)
{
    const double mhz = fabs(offset_khz) / 1000.0;
    return mhz <= 30.0 ? 0.0 : -1.2 * (mhz - 30.0);
}

static void add_tx(render_t *r, double f_khz, double bw_khz, double level_db)
{
    const rf_view_t *v = r->v;
    const double off_khz = f_khz - (double)v->center_khz;
    double xc = off_khz / r->bin_khz;
    if (v->inverted) {
        xc = -xc;
    }
    const double half = 0.5 * bw_khz / r->bin_khz;
    const double p = pow(10.0, (level_db + rolloff_db(off_khz)) / 10.0);
    const int half_bins = (int)v->bins / 2;
    int k0 = (int)floor(xc - half - LEAK_SPAN);
    int k1 = (int)ceil(xc + half + LEAK_SPAN);
    if (k0 < -half_bins) {
        k0 = -half_bins;
    }
    if (k1 > half_bins - 1) {
        k1 = half_bins - 1;
    }
    for (int k = k0; k <= k1; k++) {
        const double frac = leak_cum(xc + half - k) - leak_cum(xc - half - k);
        const float pw = (float)(p * frac);
        const unsigned idx = k >= 0 ? (unsigned)k : (unsigned)((int)v->bins + k);
        if (pw > r->lin[idx]) {
            r->lin[idx] = pw; /* max-hold: transmissions in one window are sequential */
        }
    }
}

/* ------------------------------------------------------------ transmitters */

/* Packet k of a train with period `per_us`, length `air_us`, first at t=start:
 * calls back for every packet that overlaps [lo, hi). */
typedef void (*packet_fn)(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k);

static void each_packet(render_t *r, const rf_emitter_t *e, const uint8_t *seq, double start,
                        double per_us, double air_us, double lo, double hi, packet_fn fn)
{
    double first = ceil((lo - start - air_us) / per_us);
    if (first < 0.0) {
        first = 0.0;
    }
    for (uint64_t k = (uint64_t)first; start + (double)k * per_us < hi; k++) {
        const double a = start + (double)k * per_us;
        if (a + air_us > lo) {
            fn(r, e, seq, k);
        }
    }
}

static double jitter_db(uint32_t salt, uint64_t k) /* +-2 dB of fading */
{
    return 4.0 * hash_unit(salt, (uint32_t)k) - 2.0;
}

static void elrs_packet(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k)
{
    const unsigned hop = (unsigned)((k / e->packets_per_hop) % 240u);
    const double f = 2400400.0 + 1000.0 * seq[hop];
    add_tx(r, f, e->bw_khz, e->level_db + jitter_db(e->seed, k));
}

/* BT Classic, audio streaming: per 6-slot cycle (3.75 ms) one DH5 packet
 * (2.87 ms) then a 1-slot reply (0.37 ms), each on its own hop. AFH keeps it
 * off the Wi-Fi channel given in `wifi_channel` (0 = no AFH). */
static unsigned bt_channel(const rf_emitter_t *e, uint64_t hop)
{
    unsigned allowed[79], n = 0;
    const double wifi_mhz = 2407.0 + 5.0 * e->wifi_channel;
    for (unsigned c = 0; c < 79; c++) {
        if (e->wifi_channel == 0 || fabs(2402.0 + c - wifi_mhz) > 11.0) {
            allowed[n++] = c;
        }
    }
    return allowed[hash32((uint32_t)hop ^ 0xb7c1u) % n];
}

static void bt_data(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k)
{
    (void)seq;
    add_tx(r, 2402000.0 + 1000.0 * bt_channel(e, 2 * k), 1000.0, e->level_db + jitter_db(7, k));
}

static void bt_reply(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k)
{
    (void)seq;
    add_tx(r, 2402000.0 + 1000.0 * bt_channel(e, 2 * k + 1), 1000.0,
           e->level_db - 6.0 + jitter_db(8, k));
}

/* BLE: advertising events every 100 ms on the three primary channels, and one
 * connection with a 15 ms interval hopping by channel selection algorithm #1
 * (hop increment 7 over the 37 data channels). */
static void ble_adv(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k)
{
    (void)seq;
    static const double adv_mhz[3] = { 2402.0, 2426.0, 2480.0 };
    add_tx(r, 1000.0 * adv_mhz[k % 3], 1000.0, e->level_db + jitter_db(9, k));
}

static void ble_conn(render_t *r, const rf_emitter_t *e, const uint8_t *seq, uint64_t k)
{
    (void)seq;
    const unsigned idx = (unsigned)((k * 7u) % 37u);
    const double mhz = idx <= 10 ? 2404.0 + 2.0 * idx : 2428.0 + 2.0 * (idx - 11);
    add_tx(r, 1000.0 * mhz, 1000.0, e->level_db + jitter_db(10, k));
}

static void ble_render(render_t *r, const rf_emitter_t *e, double start, double lo, double hi)
{
    /* Three advertising PDUs 0.6 ms apart, every 100 ms: as one train of
     * period 0.6 ms whose packets only exist in the first three slots. */
    const double adv_period = 100000.0;
    for (double ev = floor((lo - start) / adv_period) * adv_period; start + ev < hi; ev += adv_period) {
        if (ev < 0) {
            continue;
        }
        const uint64_t base = (uint64_t)(ev / adv_period) * 3u;
        for (unsigned i = 0; i < 3; i++) {
            const double a = start + ev + 600.0 * i;
            if (a < hi && a + 376.0 > lo) {
                ble_adv(r, e, NULL, base + i);
            }
        }
    }
    each_packet(r, e, NULL, start + 5000.0, 15000.0, 700.0, lo, hi, ble_conn);
}

/* Wi-Fi: beacons every 102.4 ms, plus traffic: each 1 ms slot carries a frame
 * with probability `duty`, 0.2-0.9 ms long. */
static void wifi_render(render_t *r, const rf_emitter_t *e, double start, double lo, double hi)
{
    const double f = 1000.0 * (2407.0 + 5.0 * e->wifi_channel);
    const double first = floor((lo - start) / 102400.0);
    for (double b = first < 0 ? 0 : first; start + b * 102400.0 < hi; b += 1.0) {
        const double a = start + b * 102400.0;
        if (a + 1000.0 > lo) {
            add_tx(r, f, 18000.0, e->level_db);
        }
    }
    double s0 = floor((lo - start) / 1000.0) - 1.0;
    for (double s = s0 < 0 ? 0 : s0; start + s * 1000.0 < hi; s += 1.0) {
        const uint32_t slot = (uint32_t)s;
        if (hash_unit(e->wifi_channel, slot) >= e->duty) {
            continue;
        }
        const double a = start + s * 1000.0 + 300.0 * hash_unit(11, slot);
        const double len = 200.0 + 700.0 * hash_unit(12, slot);
        if (a < hi && a + len > lo) {
            add_tx(r, f, 18000.0, e->level_db + jitter_db(13, slot));
        }
    }
}

static void emitter_render(render_t *r, const rf_emitter_t *e, const uint8_t *seq, double t0, double t1)
{
    const double start = e->on_s * 1e6, end = e->off_s * 1e6;
    const double lo = t0 > start ? t0 : start;
    const double hi = t1 < end ? t1 : end;
    if (lo >= hi) {
        return;
    }
    switch (e->kind) {
    case RF_ELRS:
        each_packet(r, e, seq, start, e->packet_ms * 1000.0, e->airtime_ms * 1000.0, lo, hi, elrs_packet);
        break;
    case RF_BT_CLASSIC:
        each_packet(r, e, seq, start, 3750.0, 2871.0, lo, hi, bt_data);
        each_packet(r, e, seq, start + 3125.0, 3750.0, 366.0, lo, hi, bt_reply);
        break;
    case RF_BLE:
        ble_render(r, e, start, lo, hi);
        break;
    case RF_WIFI:
        wifi_render(r, e, start, lo, hi);
        break;
    }
}

/* ------------------------------------------------------------ public */

/* ExpressLRS FHSS.cpp: each block of 80 starts on the sync channel and is
 * otherwise a permutation of the other 79. The real shuffle is seeded from
 * the binding phrase; any permutation has the properties the detector uses. */
static void elrs_sequence(uint8_t seq[240], uint32_t seed)
{
    for (unsigned b = 0; b < 3; b++) {
        uint8_t *blk = seq + b * 80;
        for (unsigned i = 0; i < 80; i++) {
            blk[i] = (uint8_t)(i == 0 ? 40 : i == 40 ? 0 : i);
        }
        for (unsigned i = 1; i < 80; i++) {
            const unsigned j = 1 + hash32(seed + b * 977u + i) % 79u;
            const uint8_t t = blk[i];
            blk[i] = blk[j];
            blk[j] = t;
        }
    }
}

void rf_scene_init(rf_scene_t *s)
{
    memset(s, 0, sizeof(*s));
    if (!s_leak_ready) {
        leak_init();
    }
}

int rf_scene_add(rf_scene_t *s, const rf_emitter_t *e)
{
    if (s->n >= RF_MAX_EMITTERS) {
        return -1;
    }
    s->em[s->n] = *e;
    if (e->kind == RF_ELRS) {
        elrs_sequence(s->elrs_seq[s->n], e->seed);
    }
    return (int)s->n++;
}

void rf_scene_render(const rf_scene_t *s, const rf_view_t *view, double t0_us, double t1_us,
                     uint8_t *codes)
{
    float lin[2048];
    const unsigned bins = view->bins <= 2048 ? view->bins : 2048;
    memset(lin, 0, sizeof(float) * bins);
    render_t r = { .v = view, .bin_khz = view->fs_hz / 1000.0 / bins, .lin = lin };

    for (unsigned i = 0; i < s->n; i++) {
        emitter_render(&r, &s->em[i], s->elrs_seq[i], t0_us, t1_us);
    }

    const double inv_n = 1.0 / (view->ffts ? view->ffts : 1);
    for (unsigned b = 0; b < bins; b++) {
        /* The largest of `ffts` exponential noise powers (mean 1), drawn
         * directly from its distribution: F(x) = (1 - e^-x)^n. */
        const double noise = -log(1.0 - pow(noise_unit(), inv_n));
        const double code = RF_NOISE_CODE + 20.0 * log10(noise + lin[b]);
        codes[b] = code <= 0.0 ? 0 : code >= 255.0 ? 255 : (uint8_t)lrint(code);
    }
}
