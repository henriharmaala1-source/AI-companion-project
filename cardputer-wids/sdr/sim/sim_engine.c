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

size_t sdr_engine_sink_capacity(void) { return sdr_sink_capacity(); }

/* ring_capture_rate_hz(), S3 branch. */
unsigned sdr_engine_rate_hz(unsigned rate_code)
{
    return rate_code == 6 ? 16000000u : rate_code == 1 ? 40000000u : 80000000u;
}

bool sdr_engine_dual_core(void) { return true; }
unsigned sdr_engine_center_mhz(void) { return E.center_mhz; }
