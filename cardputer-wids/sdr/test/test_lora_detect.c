/*
 * Host tests for lora_detect.c on synthetic 80 MS/s I/Q (lora_synth.c).
 *
 * The snapshot is laid out like the real ring: three 12,288-pair units in
 * three 16,384-word banks, the middle unit wrapping around its bank, so the
 * detector reads six segments exactly as it will on the S3.
 *
 * SNR here is LoRa power over noise in the LoRa bandwidth (812.5 kHz).
 * Passing proves the method on the model; the real check is a snapshot from
 * the Cardputer (milestone M0).
 *
 * Built with -Wall -Wextra -Werror under ASan + UBSan.
 */
#include "lora_detect.h"
#include "lora_synth.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define UNIT     12288u
#define UNITS    3u
#define BANK     16384u
#define TOTAL    (UNIT * UNITS)
#define SIGMA    8.0          /* noise per component, ADC units */

static uint32_t s_bank[UNITS][BANK];
static uint32_t s_words[TOTAL];
static float    s_work[LORA_WORK_FLOATS + 1];   /* +1: room to test a misaligned start */
static unsigned s_work_off;                     /* floats to offset `work` by */
static double   s_filter_bw;   /* > 0: pass everything through an analog low-pass model */
static lora_floor_t *s_floor;  /* non-NULL: scans keep a long-term floor, as the firmware does */
static double   s_rf_gain = 1.0; /* receiver gain: scales signal AND analog noise, not ADC noise */

/* Spread s_words over the fake banks; unit 1 wraps. */
static void build_snapshot(lora_snapshot_t *snap)
{
    static const uint32_t first[UNITS] = { 2000, 10000, 0 };
    memset(snap, 0, sizeof(*snap));
    snap->fs_hz = 80000000u;
    for (unsigned u = 0; u < UNITS; u++) {
        const uint32_t *src = s_words + u * UNIT;
        const uint32_t head = BANK - first[u] < UNIT ? BANK - first[u] : UNIT;
        memcpy(&s_bank[u][first[u]], src, head * 4);
        memcpy(&s_bank[u][0], src + head, (UNIT - head) * 4);
        snap->seg[snap->nseg++] = (lora_seg_t){ &s_bank[u][first[u]], head };
        if (UNIT > head) {
            snap->seg[snap->nseg++] = (lora_seg_t){ &s_bank[u][0], UNIT - head };
        }
    }
}

static double amp_for_snr(double snr_db)
{
    return SIGMA * sqrt(2.0 * 812500.0 / SYNTH_FS * pow(10.0, snr_db / 10.0));
}

typedef enum { X_NONE, X_CW, X_GFSK, X_OFDM, X_ALL } extra_t;

static void add_extra(synth_t *s, extra_t x, double amp)
{
    if (x == X_CW || x == X_ALL) {
        synth_cw(s, 7.3e6, amp);
    }
    if (x == X_GFSK || x == X_ALL) {
        synth_gfsk(s, -12e6, amp);
    }
    if (x == X_OFDM || x == X_ALL) {
        synth_ofdm(s, 20e6, amp);
    }
}

/* One snapshot: optional LoRa (sf 0 = none) plus extras plus noise. */
static lora_result_t run(unsigned sf, bool inv, double snr_db, double ch_hz, double start_s, extra_t x,
                         double extra_amp, double gain, uint32_t seed, uint32_t *clipped)
{
    synth_t s;
    synth_init(&s, TOTAL, seed);
    if (sf) {
        synth_lora(&s, sf, inv, ch_hz, amp_for_snr(snr_db) * gain * s_rf_gain, start_s, 60);
    }
    add_extra(&s, x, extra_amp * gain * s_rf_gain);
    synth_noise(&s, SIGMA * s_rf_gain);
    if (s_filter_bw > 0.0) {
        /* ADC noise 25 dB under the analog noise, as esp-sdr measured
         * (docs/rx-controls.md): out-of-filter bins sit that much lower. */
        synth_analog_filter(&s, s_filter_bw, SIGMA * pow(10.0, -25.0 / 20.0));
    }
    const uint32_t c = synth_pack(&s, s_words);
    if (clipped) {
        *clipped = c;
    }
    synth_free(&s);
    lora_snapshot_t snap;
    build_snapshot(&snap);
    lora_result_t r;
    assert(lora_scan(&snap, s_work + s_work_off, LORA_WORK_FLOATS, s_floor, &r));
    return r;
}

static void show(const char *what, const lora_result_t *r)
{
    printf("  %-34s %s sf %u slope %+d  f %+7.2f MHz  cand %4.1f dB |", what,
           r->detected ? "LORA" : " -- ", r->sf, r->slope, r->freq_hz / 1e6, r->cand_db);
    for (unsigned k = 0; k <= LORA_SF_MAX - LORA_SF_MIN; k++) {
        const lora_sf_metric_t *m = &r->sf_metric[k];
        printf(" SF%u %5.1f/%4.1f/%3.0f%%", LORA_SF_MIN + k, m->score, m->asym, 100.0f * m->hit_ratio);
    }
    printf("\n");
}

/* ------------------------------------------------------------ unit tests */

static void test_primitives(void)
{
    float i, q;
    lora_unpack(0x000001ffu | (0x200u << 10), &i, &q);   /* +511, -512 */
    assert(i == 511.0f && q == -512.0f);
    lora_unpack(0x3ffu | (0x001u << 10) | (0xffu << 20), &i, &q);
    assert(i == -1.0f && q == 1.0f);

    /* FFT against a direct DFT */
    float x[2 * 64], ref[2 * 64];
    for (unsigned n = 0; n < 64; n++) {
        x[2 * n] = sinf(0.3f * n) + 0.1f * n;
        x[2 * n + 1] = cosf(1.7f * n);
    }
    for (unsigned k = 0; k < 64; k++) {
        double re = 0, im = 0;
        for (unsigned n = 0; n < 64; n++) {
            const double a = -2.0 * 3.14159265358979 * k * n / 64.0;
            re += x[2 * n] * cos(a) - x[2 * n + 1] * sin(a);
            im += x[2 * n] * sin(a) + x[2 * n + 1] * cos(a);
        }
        ref[2 * k] = (float)re;
        ref[2 * k + 1] = (float)im;
    }
    lora_fft(x, 64);
    for (unsigned k = 0; k < 128; k++) {
        assert(fabsf(x[k] - ref[k]) < 1e-3f * (1.0f + fabsf(ref[k])));
    }
    printf("  unpack (signed 10-bit), FFT vs DFT OK\n");
}

int main(int argc, char **argv)
{
    const bool verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("primitives\n");
    test_primitives();

    printf("LoRa, every ExpressLRS SF, both slopes, 15 dB in-band SNR\n");
    const double ch = 2447.4e6 - 2442e6;   /* ELRS channel 47 */
    for (unsigned sf = LORA_SF_MIN; sf <= LORA_SF_MAX; sf++) {
        for (int inv = 0; inv < 2; inv++) {
            char what[64];
            snprintf(what, sizeof(what), "SF%u %s", sf, inv ? "inverted IQ" : "normal IQ");
            const lora_result_t r = run(sf, inv, 15.0, ch, -0.2e-3 * sf, X_NONE, 0, 1.0, 100 + sf * 2 + inv, NULL);
            show(what, &r);
            assert(r.detected && r.sf == sf);
            assert(fabsf(r.freq_hz - (float)ch) < 300e3f);   /* x.4 or x.6, see lora_detect.h */
        }
    }

    printf("sensitivity sweep (SF6, ELRS 250 Hz; weak = at least the weak tier)\n");
    for (double snr = 15.0; snr >= -7.5; snr -= 2.5) {
        unsigned hits = 0, weak = 0;
        lora_result_t r;
        for (unsigned trial = 0; trial < 20; trial++) {
            r = run(6, trial & 1, snr, ch, -1e-3, X_NONE, 0, 1.0, 500 + trial + (uint32_t)(snr * 10 + 100) * 32, NULL);
            hits += r.detected && r.sf == 6;
            weak += r.weak && r.sf == 6;
        }
        printf("  SNR %+5.1f dB in 812.5 kHz: detected %2u/20 weak %2u/20   last: ", snr, hits, weak);
        show("", &r);
        /* The 1 dB candidate gate: before it, 0 dB found 2 in 10. */
        assert(snr < 0.0 || hits >= 19);
    }

    printf("signal strength: measured SNR vs truth (expect truth - 1.7 dB: 1.2 MHz noise band vs 812.5 kHz)\n");
    for (double snr = 25.0; snr >= 5.0; snr -= 10.0) {
        const lora_result_t r = run(6, false, snr, ch, -1e-3, X_NONE, 0, 1.0, 700 + (uint32_t)snr, NULL);
        const double expect = snr - 1.7;
        printf("  true %4.1f dB -> measured SNR %5.1f dB (expect %5.1f), level %6.1f dBFS, agc %u\n", snr,
               r.snr_db, expect, r.level_dbfs, r.agc);
        assert(r.detected && fabs(r.snr_db - expect) < 2.0);
    }

    /* At 80 MS/s most bins lie outside the receiver's analog filter, about
     * 25 dB below the in-band noise (esp-sdr docs/rx-controls.md). The SNR
     * must be measured against the noise next to the signal, not against
     * the median of the whole spectrum. */
    printf("signal strength behind a 30 MHz analog filter (ADC noise 25 dB lower)\n");
    s_filter_bw = 30e6;
    for (double snr = 25.0; snr >= 5.0; snr -= 10.0) {
        const lora_result_t r = run(6, false, snr, ch, -1e-3, X_NONE, 0, 1.0, 750 + (uint32_t)snr, NULL);
        const double expect = snr - 1.7;
        printf("  true %4.1f dB -> measured SNR %5.1f dB (expect %5.1f), detected %s\n", snr, r.snr_db, expect,
               r.detected ? "yes" : "no");
        assert(r.detected && fabs(r.snr_db - expect) < 2.0);
    }
    /* The filter's edges are steps 25 dB tall that a single snapshot takes
     * for signals; they filled every candidate slot (BUGLOG). The firmware
     * keeps a long-term floor (lora_floor_t), which removes them. */
    {
        /* ELRS hops: a different grid channel, inside the filter, each time. */
#define HOP(t) (2400.4e6 + (double)(32u + (t) * 7u % 20u) * 1e6 - 2442e6)
        unsigned alone = 0, with_floor = 0, fa = 0, weak = 0;
        for (unsigned trial = 0; trial < 20; trial++) {
            const lora_result_t r = run(6, trial & 1, 0.0, HOP(trial), -1e-3, X_NONE, 0, 1.0, 7700 + trial, NULL);
            alone += r.detected && r.sf == 6;
        }
        static lora_floor_t fl;
        memset(&fl, 0, sizeof(fl));
        s_floor = &fl;
        for (unsigned trial = 0; trial < 20; trial++) {   /* settle: no LoRa yet */
            (void)run(0, false, 0, 0, 0, X_NONE, 0, 1.0, 7800 + trial, NULL);
        }
        for (unsigned trial = 0; trial < 20; trial++) {
            const lora_result_t r = run(6, trial & 1, 0.0, HOP(trial), -1e-3, X_NONE, 0, 1.0, 7700 + trial, NULL);
            with_floor += r.detected && r.sf == 6;
        }
        for (unsigned trial = 0; trial < 60; trial++) {   /* not LoRa, through the same filter */
            const extra_t x = trial % 3 == 0 ? X_NONE : trial % 3 == 1 ? X_GFSK : X_OFDM;
            const lora_result_t r = run(0, false, 0, 0, 0, x, 150, 1.0, 7900 + trial, NULL);
            fa += r.detected;
            weak += r.weak;
        }
        /* The AGC drops the gain 12 dB (a new gain index): everything before
         * the ADC is 12 dB lower, below the learned floor. The floor must
         * not blind the detector while it is relearned at the new gain. */
        unsigned after_agc = 0;
        synth_gain_index = 30;
        s_rf_gain = 0.25;
        for (unsigned trial = 0; trial < 20; trial++) {
            const lora_result_t r = run(6, trial & 1, 0.0, HOP(trial + 3), -1e-3, X_NONE, 0, 1.0, 8100 + trial, NULL);
            after_agc += r.detected && r.sf == 6;
        }
        s_rf_gain = 1.0;
        synth_gain_index = 40;
        s_floor = NULL;
        printf("  SNR 0 dB behind the filter: detected %u/20 judged alone, %u/20 with the floor "
               "(no filter: 20/20); not LoRa: %u/60 false, %u/60 weak; after a 12 dB gain drop: %u/20\n",
               alone, with_floor, fa, weak, after_agc);
        assert(with_floor >= 19 && fa == 0 && after_agc >= 17);
#undef HOP
    }
    s_filter_bw = 0.0;

    /* The heap only promises 4-byte alignment; the S3 FFT needs more. A
     * misaligned scratch must give the same answer (BUGLOG). */
    printf("work buffer misaligned by 4 bytes\n");
    {
        s_work_off = 1;
        const lora_result_t r = run(6, true, 15.0, ch, -1e-3, X_NONE, 0, 1.0, 9100, NULL);
        s_work_off = 0;
        show("SF6, work + 4 bytes", &r);
        assert(r.detected && r.sf == 6);
    }

    printf("overload: LoRa driven 4x past full scale (clips), like a TX 2 m away\n");
    {
        uint32_t clipped = 0;
        const lora_result_t r = run(6, false, 60.0, ch, -1e-3, X_NONE, 0, 1.0, 900, &clipped);
        show("SF6 clipped", &r);
        printf("  clipped samples: %u of %u\n", clipped, TOTAL * 2);
        assert(clipped > TOTAL / 2);
        assert(r.detected && r.sf == 6);
    }

    printf("not LoRa: must never be reported (30 seeds each)\n");
    static const struct { const char *name; extra_t x; double amp; } k_neg[] = {
        { "noise only", X_NONE, 0 },
        { "strong carrier", X_CW, 200 },
        { "Bluetooth-like GFSK", X_GFSK, 120 },
        { "Wi-Fi-like OFDM", X_OFDM, 200 },
        { "all three", X_ALL, 120 },
        { "all three, overloaded", X_ALL, 600 },
    };
    unsigned false_alarms = 0, weak_total = 0, runs = 0;
    for (unsigned k = 0; k < sizeof(k_neg) / sizeof(k_neg[0]); k++) {
        unsigned fa = 0, weak = 0;
        lora_result_t r;
        for (unsigned trial = 0; trial < 30; trial++) {
            r = run(0, false, 0, 0, 0, k_neg[k].x, k_neg[k].amp, 1.0, 2000 + k * 100 + trial, NULL);
            fa += r.detected;
            weak += r.weak;
            runs++;
            if (verbose && r.weak) {
                show(k_neg[k].name, &r);
            }
        }
        printf("  %-24s false alarms %u/30 weak %u/30   last: ", k_neg[k].name, fa, weak);
        show("", &r);
        false_alarms += fa;
        weak_total += weak;
    }
    assert(false_alarms == 0);
    /* Weak snapshots happen by chance (~1 in 300 in a longer host run) and
     * are only trusted when 3 agree (chirp_watch.c); many more would mean the
     * weak floor is too low for that rule. */
    printf("  weak by chance: %u of %u snapshots\n", weak_total, runs);
    assert(weak_total * 50 <= runs);

    printf("LoRa among interferers\n");
    {
        const lora_result_t r = run(6, true, 15.0, ch, -1e-3, X_ALL, 120, 1.0, 3000, NULL);
        show("SF6 + carrier + GFSK + OFDM", &r);
        assert(r.detected && r.sf == 6);
    }

    printf("timing (host, informational)\n");
    {
        const clock_t t0 = clock();
        for (unsigned k = 0; k < 5; k++) {
            (void)run(6, false, 15.0, ch, -1e-3, X_NONE, 0, 1.0, 4000 + k, NULL);
        }
        printf("  %.1f ms per snapshot including synthesis\n", 1000.0 * (double)(clock() - t0) / CLOCKS_PER_SEC / 5);
    }

    printf("ALL LORA DETECTOR TESTS PASSED\n");
    return 0;
}
