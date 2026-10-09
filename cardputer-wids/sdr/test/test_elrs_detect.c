/*
 * Host tests for spc1.c and elrs_detect.c.
 *
 * The signal generator is a MODEL, not a recording. It reproduces what the
 * ExpressLRS source says about the link (grid, hop blocks, packet timing) and
 * crude versions of Bluetooth and Wi-Fi. Passing here proves the logic does
 * what it claims on that model; it does not prove the Cardputer can see a real
 * transmitter. That is milestone M0, and real recordings replace this
 * generator at M2.
 *
 * Built with -Wall -Wextra -Werror and run under ASan + UBSan.
 */
#include "elrs_detect.h"
#include "spc1.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------ model */

#define FS_HZ     80000000u
#define BINS      256u
#define CENTER_KHZ 2442000u
#define FRAME_US  1843u          /* 12 ring units of 12288 pairs at 80 MS/s */
#define NOISE     60u            /* noise code (0.5 dB steps) */

static uint32_t rng_state = 12345u;
static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

typedef enum { SRC_NONE, SRC_ELRS, SRC_BLE, SRC_WIFI, SRC_CARRIER } source_t;

typedef struct {
    source_t kind;
    /* ELRS */
    float dwell_ms, packet_ms, airtime_ms;
    uint8_t seq[240];
    unsigned hop;
    double hop_start_ms;
    /* BLE-like */
    double next_hop_ms;
    unsigned ble_ch;
} model_t;

/* FHSS.cpp: blocks of 80, each starting on the sync channel and otherwise a
 * permutation of the remaining 79 channels. */
static void make_elrs_sequence(uint8_t seq[240])
{
    for (unsigned b = 0; b < 3; b++) {
        uint8_t *blk = seq + b * 80;
        for (unsigned i = 0; i < 80; i++) {
            blk[i] = (uint8_t)(i == 0 ? ELRS_SYNC_CHANNEL : i == ELRS_SYNC_CHANNEL ? 0 : i);
        }
        for (unsigned i = 1; i < 80; i++) {
            unsigned r = 1 + rnd() % 79;
            uint8_t t = blk[i];
            blk[i] = blk[r];
            blk[r] = t;
        }
    }
}

static void add_burst(uint8_t *codes, double f_khz, double width_khz, unsigned level)
{
    const double bin_khz = FS_HZ / 1000.0 / BINS;
    for (unsigned i = 0; i < BINS; i++) {
        int k = i < BINS / 2 ? (int)i : (int)i - (int)BINS;
        double bf = CENTER_KHZ + k * bin_khz;
        double dist = fabs(bf - f_khz);
        unsigned add = 0;
        if (dist <= width_khz / 2.0) {
            add = level;
        } else if (dist <= width_khz / 2.0 + bin_khz) {
            add = level / 3;      /* window leakage into the neighbour */
        }
        if (codes[i] + add > 255u) {
            codes[i] = 255u;
        } else if (codes[i] < NOISE + add) {
            codes[i] = (uint8_t)(NOISE + add);
        }
    }
}

/* Fill one frame covering [t_ms, t_ms + frame). */
static void render(model_t *m, double t_ms, uint8_t *codes, unsigned level)
{
    for (unsigned i = 0; i < BINS; i++) {
        codes[i] = (uint8_t)(NOISE + rnd() % 7);   /* max-hold noise jitter */
    }
    const double frame_ms = FRAME_US / 1000.0;
    switch (m->kind) {
    case SRC_ELRS: {
        while (t_ms >= m->hop_start_ms + m->dwell_ms) {
            m->hop_start_ms += m->dwell_ms;
            m->hop = (m->hop + 1) % 240;
        }
        /* Packets inside this hop that overlap the frame. */
        double into = t_ms - m->hop_start_ms;
        double pk = floor(into / m->packet_ms) * m->packet_ms;
        bool on = (into - pk) < m->airtime_ms ||
                  (pk + m->packet_ms < m->dwell_ms && pk + m->packet_ms < into + frame_ms);
        if (on) {
            add_burst(codes, ELRS_BASE_KHZ + m->seq[m->hop] * ELRS_STEP_KHZ, 800.0, level);
        }
        break;
    }
    case SRC_BLE:
        /* Even-MHz channels 2404..2478, hop every 7.5 ms, ~1 ms bursts. */
        if (t_ms >= m->next_hop_ms) {
            m->next_hop_ms += 7.5;
            m->ble_ch = rnd() % 38;
        }
        if (fmod(t_ms, 7.5) < 1.2) {
            add_burst(codes, 2404000.0 + 2000.0 * m->ble_ch, 1000.0, level);
        }
        break;
    case SRC_WIFI:
        if (rnd() % 10 < 3) {
            static const double ch[3] = { 2412000.0, 2437000.0, 2462000.0 };
            add_burst(codes, ch[rnd() % 3], 18000.0, level);
        }
        break;
    case SRC_CARRIER:
        add_burst(codes, 2430400.0, 600.0, level);
        break;
    default:
        break;
    }
}

static elrs_state_t run(model_t *m, double seconds, unsigned level, elrs_status_t *st,
                        double *first_likely_s, elrs_detector_t *d, double *t_ms_io)
{
    uint8_t codes[BINS];
    elrs_state_t state = d->state;
    const unsigned frames = (unsigned)(seconds * 1e6 / FRAME_US);
    for (unsigned f = 0; f < frames; f++) {
        render(m, *t_ms_io, codes, level);
        elrs_push_frame(d, codes, BINS, 2, (uint64_t)(*t_ms_io * 1000.0), FRAME_US);
        *t_ms_io += FRAME_US / 1000.0;
        if (f % 60 == 59) {                 /* evaluate about every 110 ms, like a slice */
            elrs_evaluate(d, (uint64_t)(*t_ms_io * 1000.0), st);
            state = st->state;
            if (first_likely_s && *first_likely_s < 0 && state == ELRS_LIKELY) {
                *first_likely_s = *t_ms_io / 1000.0;
            }
        }
    }
    return state;
}

static void new_detector(elrs_detector_t *d)
{
    elrs_config_t cfg;
    elrs_default_config(&cfg, CENTER_KHZ, FS_HZ);
    elrs_init(d, &cfg);
}

static elrs_detector_t det; /* ~10 KB: keep it off the test's stack */

static void check_elrs(const char *name, float dwell, float packet, float air)
{
    model_t m = { .kind = SRC_ELRS, .dwell_ms = dwell, .packet_ms = packet, .airtime_ms = air };
    make_elrs_sequence(m.seq);
    new_detector(&det);
    elrs_status_t st;
    double t = 0, first = -1;
    elrs_state_t s = run(&m, 20.0, 40, &st, &first, &det, &t);
    printf("  %-22s -> %-11s first %.1fs  seen %u/%u  on-grid %.2f  cv %.2f  dwell~%.0fms  score %.2f\n",
           name, elrs_state_name(s), first, st.channels_seen, st.channels_visible,
           st.on_grid_ratio, st.uniformity_cv, st.dwell_ms, st.score);
    assert(s == ELRS_LIKELY);
    assert(first > 0 && first <= 12.0);
}

static void check_not_elrs(const char *name, source_t kind, elrs_state_t max_allowed)
{
    model_t m = { .kind = kind, .next_hop_ms = 0 };
    new_detector(&det);
    elrs_status_t st;
    double t = 0, first = -1;
    elrs_state_t worst = ELRS_QUIET;
    for (int i = 0; i < 10; i++) {          /* 30 s in 3 s chunks, track the worst state */
        elrs_state_t s = run(&m, 3.0, 40, &st, &first, &det, &t);
        if (s > worst) {
            worst = s;
        }
    }
    printf("  %-22s -> worst %-11s seen %u  on-grid %.2f  distinct %u  wide/s %.1f\n",
           name, elrs_state_name(worst), st.channels_seen, st.on_grid_ratio,
           st.distinct_narrow, st.wide_per_s);
    assert(worst <= max_allowed);
}

/* ------------------------------------------------------------ decoder tests */

static unsigned got_frames;
static uint32_t got_seq;
static void on_frame(const spc1_frame_t *f, void *ctx)
{
    (void)ctx;
    got_frames++;
    got_seq = f->sequence;
    assert(f->bins == 256 && f->codes[0] == 0 && f->codes[255] == 255);
}

static size_t make_frame(uint8_t *out, uint32_t seq)
{
    memset(out, 0, 28);
    memcpy(out, "SPC1", 4);
    out[4] = (uint8_t)seq;
    out[26] = 8;                  /* 256 bins */
    out[27] = 2;
    for (unsigned i = 0; i < 256; i++) {
        out[28 + i] = (uint8_t)i;
    }
    uint32_t crc = spc1_crc32(0, out, 28 + 256);
    memcpy(out + 28 + 256, &crc, 4);   /* host is little-endian */
    return 28 + 256 + 4;
}

static void decoder_tests(void)
{
    static spc1_decoder_t dec;
    uint8_t fr[400], stream[2000];

    /* zlib check value */
    assert(spc1_crc32(0, (const uint8_t *)"123456789", 9) == 0xCBF43926u);

    /* whole frame, then byte-by-byte */
    size_t n = make_frame(fr, 7);
    spc1_init(&dec, on_frame, NULL);
    got_frames = 0;
    spc1_feed(&dec, fr, n);
    assert(got_frames == 1 && got_seq == 7);
    for (size_t i = 0; i < n; i++) {
        spc1_feed(&dec, fr + i, 1);
    }
    assert(got_frames == 2);

    /* garbage, a fake magic with a corrupt CRC, then two good frames */
    size_t len = 0;
    memcpy(stream, "noise SPC", 9);
    len = 9;
    size_t bad = make_frame(stream + len, 1);
    stream[len + 100] ^= 0x55;         /* break the CRC */
    len += bad;
    len += make_frame(stream + len, 2);
    len += make_frame(stream + len, 3);
    spc1_init(&dec, on_frame, NULL);
    got_frames = 0;
    for (size_t i = 0; i < len; i += 37) {   /* odd chunking */
        spc1_feed(&dec, stream + i, len - i < 37 ? len - i : 37);
    }
    assert(got_frames == 2 && got_seq == 3 && dec.crc_bad == 1);

    /* header claiming 2^30 bins must be treated as garbage, not a huge frame */
    n = make_frame(fr, 9);
    fr[26] = 30;
    spc1_init(&dec, on_frame, NULL);
    got_frames = 0;
    spc1_feed(&dec, fr, n);
    n = make_frame(fr, 10);
    spc1_feed(&dec, fr, n);
    assert(got_frames == 1 && got_seq == 10);

    /* truncated frame: nothing delivered, nothing read past the input */
    n = make_frame(fr, 11);
    spc1_init(&dec, on_frame, NULL);
    got_frames = 0;
    spc1_feed(&dec, fr, n - 5);
    assert(got_frames == 0);
    printf("  spc1 decoder: crc, chunking, resync, bad header, truncation OK\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("decoder\n");
    decoder_tests();

    printf("ELRS model (rates from ExpressLRS common.cpp)\n");
    check_elrs("FLRC 1000 Hz", 2.0f, 1.0f, 0.25f);
    check_elrs("LoRa 500 Hz", 8.0f, 2.0f, 1.1f);
    check_elrs("LoRa 250 Hz", 16.0f, 4.0f, 2.2f);
    check_elrs("LoRa 50 Hz", 40.0f, 20.0f, 8.0f);

    printf("not ELRS\n");
    check_not_elrs("noise only", SRC_NONE, ELRS_QUIET);
    check_not_elrs("Wi-Fi 20 MHz bursts", SRC_WIFI, ELRS_QUIET);
    check_not_elrs("fixed narrow carrier", SRC_CARRIER, ELRS_QUIET);
    check_not_elrs("BLE-like hopping", SRC_BLE, ELRS_HOPPER);

    /* Hold-down: after the link stops, the verdict stays for hold_s, then drops. */
    {
        model_t m = { .kind = SRC_ELRS, .dwell_ms = 8, .packet_ms = 2, .airtime_ms = 1.1f };
        make_elrs_sequence(m.seq);
        new_detector(&det);
        elrs_status_t st;
        double t = 0;
        assert(run(&m, 15.0, 40, &st, NULL, &det, &t) == ELRS_LIKELY);
        m.kind = SRC_NONE;
        assert(run(&m, 5.0, 40, &st, NULL, &det, &t) == ELRS_LIKELY);  /* held */
        assert(run(&m, 30.0, 40, &st, NULL, &det, &t) == ELRS_QUIET);  /* released */
        printf("  hysteresis: held after link stops, released later OK\n");
    }

    /* Weak link: 6 dB above noise is under the 10 dB threshold -> must not alarm. */
    {
        model_t m = { .kind = SRC_ELRS, .dwell_ms = 8, .packet_ms = 2, .airtime_ms = 1.1f };
        make_elrs_sequence(m.seq);
        new_detector(&det);
        elrs_status_t st;
        double t = 0;
        assert(run(&m, 20.0, 12, &st, NULL, &det, &t) == ELRS_QUIET);
        printf("  below-threshold link stays QUIET (honest miss, not a false alarm) OK\n");
    }

    printf("ALL ELRS DETECTOR TESTS PASSED\n");
    return 0;
}
