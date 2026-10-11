/*
 * sim_engine.c - sdr_engine.h on the host.
 *
 * esp-sdr's S3 capture loop reduced to its timing and output rules, feeding
 * the firmware's REAL sdr_sink.c. Each rule below cites the upstream line it
 * copies (third_party/esp-sdr/main/common/ring_capture.c @ e74f2a4), because
 * the whole point is to behave like upstream, not like what we assumed:
 *
 *   - the ring hands over a unit every RING_THRESHOLD = 12288 pairs
 *     (ring_capture.h), about 154 us at 80 MS/s;
 *   - finished FFTs accumulate into the open frame, and the frame is closed
 *     "as soon as the previous output has drained" (:626, :1191-1192), i.e. when
 *     the 16 KiB output queue is empty. There is no frame timer, and
 *     units_per_frame is ignored on the S3 (docs/spectrum.md);
 *   - txq_pump() moves at most 64 bytes per call into ring_write() (:260);
 *   - a frame that does not fit the queue is dropped and counted (:591);
 *   - the open frame is emitted at the end of the run (:1287), then the queue
 *     is drained with a 500 ms deadline (:1764);
 *   - the next run starts by resetting the queue (:1340), discarding anything
 *     still in it.
 *
 * What it does not model: FFT work being abandoned when core 1 falls behind,
 * the exact poll cadence, and the cost of a recalibration in prepare_rx().
 */
#include "sim_engine.h"

#include <math.h>
#include <string.h>

#include "lora_synth.h"
#include "sdr_engine.h"
#include "sdr_sink.h"
#include "sim.h"

#define UNIT_PAIRS        12288u   /* RING_THRESHOLD */
#define TXQ_SIZE          16384u   /* ring_capture.c, S3 */
#define PUMP_BYTES        64u      /* txq_pump(): one 64-byte packet per call */
#define DRAIN_DEADLINE_US 500000   /* end of ring_capture_run() */
#define HEADER_LEN        28u

/* ASSUMPTIONS, not taken from upstream; the results are not sensitive to them
 * as long as the pump runs much faster than a unit arrives. */
#define PUMP_PERIOD_US    5.0      /* how often the poll loop calls txq_pump() */
#define RUN_SETUP_US      300      /* prepare_rx() + run setup before unit 0 */
#define PROCESS_US        2000     /* decode + detect between slices */
#define CHIRP_SCAN_US     30000    /* lora_scan() on the S3: unknown until measured (scan_ms in the log) */
#define CAP_UNIT          12288u   /* pairs per ring unit, as upstream */
#define CAP_NOISE_SIGMA   8.0      /* ADC units per component */

static struct {
    const rf_scene_t *scene;
    bool     inverted;
    unsigned center_mhz;
    bool     charge_processing;
    /* upstream's output queue */
    uint8_t  txq[TXQ_SIZE];
    uint32_t head, tail;
    /* the open frame */
    uint8_t  acc[2048];
    unsigned units, ffts;
    uint64_t index, pairs;
    bool     dropped;
    uint32_t frames, drops;     /* this run */
    uint8_t  out[HEADER_LEN + 2048u + 4u];
    sim_engine_stats_t stats;
} E;

/* ------------------------------------------------------------ helpers */

static uint32_t crc32_zlib(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (unsigned k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static void put_le(uint8_t *p, uint64_t v, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        p[i] = (uint8_t)(v >> (8u * i));
    }
}

static void advance_to(double t_us)
{
    const int64_t target = (int64_t)llround(t_us);
    if (target > sim_now_us()) {
        sim_advance_us(target - sim_now_us());
    }
}

static unsigned log2u(unsigned v)
{
    unsigned l = 0;
    while ((1u << l) < v) {
        l++;
    }
    return l;
}

/* ------------------------------------------------------------ upstream rules */

/* txq_pump(): at most one 64-byte packet into ring_write() per call. */
static void pump(void)
{
    const uint32_t used = E.head - E.tail;
    if (used == 0) {
        return;
    }
    const uint32_t off = E.tail % TXQ_SIZE;
    uint32_t n = TXQ_SIZE - off;
    if (n > used) {
        n = used;
    }
    if (n > PUMP_BYTES) {
        n = PUMP_BYTES;
    }
    const int written = sdr_sink_write(E.txq + off, n);
    if (written > 0) {
        E.tail += (uint32_t)written;
    }
}

static bool txq_push(const uint8_t *p, uint32_t n)
{
    if (TXQ_SIZE - (E.head - E.tail) < n) {
        return false;
    }
    for (uint32_t i = 0; i < n; i++) {
        E.txq[(E.head + i) % TXQ_SIZE] = p[i];
    }
    E.head += n;
    return true;
}

/* frame_close() + emit_chunk(): header, codes, CRC, then into the queue. */
static void close_frame(unsigned bins, unsigned fs_hz)
{
    uint8_t *o = E.out;
    memcpy(o, "SPC1", 4);
    put_le(o + 4, E.frames + E.drops, 4);
    put_le(o + 8, E.index, 8);
    put_le(o + 16, E.pairs, 4);
    put_le(o + 20, E.ffts > 65535u ? 65535u : E.ffts, 2);
    o[22] = (uint8_t)(1u | (E.dropped ? 4u : 0u));   /* max-hold */
    o[23] = 0;                                         /* gain metadata */
    put_le(o + 24, E.drops > 65535u ? 65535u : E.drops, 2);
    o[26] = (uint8_t)log2u(bins);
    o[27] = 2;
    memcpy(o + HEADER_LEN, E.acc, bins);
    const uint32_t len = HEADER_LEN + bins;
    put_le(o + len, crc32_zlib(0, o, len), 4);

    const double frame_us = (double)E.pairs * 1e6 / fs_hz;
    if (txq_push(o, len + 4)) {
        E.frames++;
        E.dropped = false;
        E.stats.frames++;
        E.stats.frame_us_sum += frame_us;
        if (frame_us > E.stats.frame_us_max) {
            E.stats.frame_us_max = frame_us;
        }
        if (frame_us > 10000.0) {
            E.stats.frames_over_10ms++;
        }
    } else {
        E.drops++;
        E.dropped = true;
        E.stats.drops++;
    }
    E.units = E.ffts = 0;
    E.pairs = 0;
}

static void merge_unit(const uint8_t *codes, unsigned bins, unsigned ffts, uint64_t index)
{
    if (E.units == 0) {
        E.index = index;
        memcpy(E.acc, codes, bins);
    } else {
        for (unsigned b = 0; b < bins; b++) {
            if (codes[b] > E.acc[b]) {
                E.acc[b] = codes[b];    /* max-hold across units */
            }
        }
    }
    E.units++;
    E.ffts += ffts;
    E.pairs += UNIT_PAIRS;
}

/* ------------------------------------------------------------ sdr_engine.h */

void sim_engine_attach(const rf_scene_t *scene, bool inverted)
{
    E.scene = scene;
    E.inverted = inverted;
}

const sim_engine_stats_t *sim_engine_stats(void) { return &E.stats; }

esp_err_t sdr_engine_init(unsigned center_mhz)
{
    if (E.scene == NULL || !sdr_sink_init(SDR_SINK_BYTES)) {
        return ESP_FAIL;
    }
    E.center_mhz = center_mhz;
    return ESP_OK;
}

void sdr_engine_run_slice(const sdr_spec_cfg_t *cfg, uint32_t duration_ms, sdr_slice_result_t *out)
{
    memset(out, 0, sizeof(*out));
    const unsigned fs = sdr_engine_rate_hz(cfg->rate_code);
    const unsigned bins = cfg->nfft <= 2048u ? cfg->nfft : 2048u;
    const rf_view_t view = {
        .center_khz = E.center_mhz * 1000u, .fs_hz = fs, .bins = bins,
        .ffts = UNIT_PAIRS / bins, .inverted = E.inverted,
    };

    /* :1340 - a new run starts with an empty queue, whatever was left. */
    E.stats.txq_discarded += E.head - E.tail;
    E.head = E.tail = 0;
    E.units = 0;
    E.frames = E.drops = 0;
    E.dropped = false;

    sim_advance_us(RUN_SETUP_US);
    sdr_sink_pace(SDR_FRAME_BYTES(bins), cfg->frame_us);   /* as s3_rx.c does */
    const double t_start = (double)sim_now_us();
    const double unit_us = UNIT_PAIRS * 1e6 / fs;
    const double end_us = t_start + duration_ms * 1000.0;
    uint8_t codes[2048];
    uint64_t index = 0;

    for (double t0 = t_start; t0 < end_us; t0 += unit_us) {
        const double t1 = t0 + unit_us;
        /* The poll loop pumps the queue while the next unit fills. */
        for (double tp = t0 + PUMP_PERIOD_US; tp <= t1; tp += PUMP_PERIOD_US) {
            advance_to(tp);
            pump();
        }
        advance_to(t1);
        rf_scene_render(E.scene, &view, t0, t1, codes);
        merge_unit(codes, bins, view.ffts, index);
        index += UNIT_PAIRS;
        /* :626 / :1191 - emit as soon as the previous output has drained. */
        if (E.head == E.tail) {
            close_frame(bins, fs);
        }
    }
    const double elapsed = (double)sim_now_us() - t_start;

    /* :1287 - the open frame goes out at the end of the run regardless. */
    if (E.units > 0) {
        close_frame(bins, fs);
    }
    /* :1764 - drain with a deadline; the "host" may have stopped reading. */
    const double deadline = (double)sim_now_us() + DRAIN_DEADLINE_US;
    while (E.head != E.tail && (double)sim_now_us() < deadline) {
        advance_to((double)sim_now_us() + PUMP_PERIOD_US);
        pump();
    }
    if (E.head != E.tail) {
        E.stats.drain_timeouts++;
    }

    E.stats.slices++;
    E.stats.capture_us += elapsed;
    E.charge_processing = true;

    out->frames = E.frames;
    out->drops = E.drops;
    out->ffts = (uint32_t)(index / bins);
    out->pairs = index;
    out->elapsed_us = (uint64_t)elapsed;
    out->sink_used = sdr_sink_used();
}

size_t sdr_engine_read(uint8_t *dst, size_t cap)
{
    const size_t n = sdr_sink_read(dst, cap);
    if (n == 0 && E.charge_processing) {
        sim_advance_us(PROCESS_US);   /* the CPU time decode + detect take */
        E.charge_processing = false;
    }
    return n;
}

/* ------------------------------------------------------------ raw capture */

/* Scene level (dB over noise per 312.5 kHz FFT bin, rf_scene.h) to a
 * synthesis amplitude for a signal spread over bw_hz. */
static double amp_for(float level_db, double bw_hz)
{
    const double noise_per_bin = 2.0 * CAP_NOISE_SIGMA * CAP_NOISE_SIGMA * (312500.0 / SYNTH_FS);
    const double bins = bw_hz / 312500.0;
    return sqrt(pow(10.0, level_db / 10.0) * noise_per_bin * (bins > 1.0 ? bins : 1.0));
}

static uint32_t mix32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    return x;
}

/* ELRS packets overlapping [t0, t0 + dur): LoRa at their hop channel, or
 * GFSK-like for FLRC rates (no chirp). */
static void synth_elrs(synth_t *sy, const rf_scene_t *sc, unsigned idx, double t0, double dur, double center)
{
    const rf_emitter_t *e = &sc->em[idx];
    const double start = e->on_s * 1e6, end = e->off_s * 1e6;
    const double per = e->packet_ms * 1000.0, air = e->airtime_ms * 1000.0;
    double k0 = floor((t0 - start - air) / per);
    for (double k = k0 < 0 ? 0 : k0; start + k * per < t0 + dur; k += 1.0) {
        const double ps = start + k * per;
        if (ps + air <= t0 || ps >= end) {
            continue;
        }
        const unsigned ch = sc->elrs_seq[idx][((uint64_t)k / e->packets_per_hop) % 240u];
        const double f = 2400.4e6 + ch * 1e6 - center;
        const float level = rf_level_at(e, ps);
        if (e->sf == 0) {
            synth_gfsk(sy, f, amp_for(level, 600e3));
            return;
        }
        const double sym_us = (double)(1u << e->sf) / 812500.0 * 1e6;
        int payload = (int)(air / sym_us - 16.25);
        synth_lora(sy, e->sf, e->invert_iq, f, amp_for(level, 812500.0), (ps - t0) / 1e6,
                   (unsigned)(payload < 4 ? 4 : payload));
    }
}

void sdr_engine_capture(unsigned rate_code, unsigned units, sdr_capture_t *out)
{
    static uint32_t words[3 * CAP_UNIT];
    static uint32_t seed = 77;
    memset(out, 0, sizeof(*out));
    units = units < 1 ? 1 : units > 3 ? 3 : units;
    const uint32_t n = units * CAP_UNIT;
    const unsigned fs = sdr_engine_rate_hz(rate_code);
    const double center = E.center_mhz * 1e6;

    sim_advance_us(RUN_SETUP_US);
    const double t0 = (double)sim_now_us(), dur = n * 1e6 / fs;
    synth_t sy;
    synth_init(&sy, n, seed++);
    for (unsigned i = 0; i < E.scene->n; i++) {
        const rf_emitter_t *e = &E.scene->em[i];
        if (t0 + dur <= e->on_s * 1e6 || t0 >= e->off_s * 1e6) {
            continue;
        }
        const uint32_t h = mix32((uint32_t)(t0 / 100.0) ^ (i * 0x9e37u));
        switch (e->kind) {
        case RF_ELRS:
            synth_elrs(&sy, E.scene, i, t0, dur, center);
            break;
        case RF_BT_CLASSIC:   /* busy most of the time: one hop in the window */
            synth_gfsk(&sy, (2402.0 + h % 79u) * 1e6 - center, amp_for(e->level_db, 1e6));
            break;
        case RF_WIFI:
            if ((h % 1000u) < (uint32_t)(e->duty * 1000.0f)) {
                synth_ofdm(&sy, (2407.0 + 5.0 * e->wifi_channel) * 1e6 - center, amp_for(e->level_db, 16.6e6));
            }
            break;
        case RF_BLE:          /* sparse 0.4-1 ms events: mostly absent from 0.46 ms */
            if (h % 10u == 0) {
                synth_gfsk(&sy, (2404.0 + 2.0 * (h % 37u)) * 1e6 - center, amp_for(e->level_db, 1e6));
            }
            break;
        }
    }
    synth_noise(&sy, CAP_NOISE_SIGMA);
    if (E.inverted) {
        /* A mirrored spectrum is conjugated I/Q: frequencies flip sign and
         * chirps change direction. The raw path sees it the same way. */
        for (uint32_t k = 0; k < n; k++) {
            sy.q[k] = -sy.q[k];
        }
    }
    (void)synth_pack(&sy, words);
    synth_free(&sy);
    sim_advance_us((int64_t)dur + CHIRP_SCAN_US);

    out->fs_hz = fs;
    out->nseg = 1;
    out->seg[0] = words;
    out->len[0] = n;
}

void *sdr_engine_scratch(size_t *bytes) { return sdr_sink_scratch(bytes); }

/* Gain is not modelled: the scene's levels are what the detector sees. */
int sdr_engine_set_gain(int code) { return code < 0 ? -1 : code > 82 ? 82 : code; }
unsigned sdr_engine_gain_max(void) { return 82; }

size_t sdr_engine_sink_capacity(void) { return sdr_sink_capacity(); }

/* ring_capture_rate_hz(), S3 branch. */
unsigned sdr_engine_rate_hz(unsigned rate_code)
{
    return rate_code == 6 ? 16000000u : rate_code == 1 ? 40000000u : 80000000u;
}

bool sdr_engine_dual_core(void) { return true; }
unsigned sdr_engine_center_mhz(void) { return E.center_mhz; }
