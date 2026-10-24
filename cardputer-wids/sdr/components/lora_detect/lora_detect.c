/* lora_detect.c - see lora_detect.h for what is detected and why. */
#include "lora_detect.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include "dsps_fft2r.h"
#endif

/* Candidate selection on the coarse spectrum. Only ELRS grid positions are
 * candidates: 2400.4 + k MHz is x.4 MHz from an integer-MHz LO, or x.6 MHz
 * if the hardware mirrors the spectrum (unconfirmed, plan M0), so both. The
 * baseband is 1.25 MHz wide, so a burst 200 kHz off either still dechirps. */
/* A LoRa signal at 0 dB SNR raises its channel only ~3 dB, so a 3 dB gate
 * lost everything below about +1 dB. 48 averaged blocks keep the estimate's
 * spread to a few tenths of a dB, so 1 dB is still clear of noise. Host
 * sweep: 50% detection moved from about +1 to about -3.5 dB SNR (SF6), no
 * false alarm added. A false candidate costs time, never a false alarm:
 * the dechirp decides. */
#define CAND_MIN_DB      1.0f   /* channel over its neighbours (narrowband excess) */
#define CAND_FLOOR_DB    1.0f   /* channel over its long-term floor (or the median bin) */
/* lora_floor_t tracking, per snapshot: fall a quarter of the way to a lower
 * reading, rise 0.05 dB. A hopping ELRS link is on a given channel in ~1 %
 * of snapshots and never lifts it; a parked carrier is absorbed at ~0.35 dB/s
 * (7 snapshots/s). */
#define FLOOR_FALL       0.25f
#define FLOOR_RISE       1.0116f
/* The floor is only comparable at the gain it was learned at; the hardware
 * AGC can move between snapshots. A snapshot at another gain index is judged
 * the single-snapshot way and leaves the floor alone. If the new gain lasts
 * this many snapshots (~1 s), the floor is relearned there. */
#define FLOOR_RELEARN    8u
#define COARSE_BLOCKS    48u    /* FFT blocks averaged, spread over the snapshot */
#define EDGE_HZ          34e6f  /* ignore beyond: analogue roll-off */
#define GRID_FRAC_HZ     400e3f /* ELRS grid offset from an integer MHz */
#define CHAN_HALF_HZ     350e3f /* channel power window: +-350 kHz */
#define NEIGH_IN_HZ      900e3f /* neighbour windows: 0.9..1.6 MHz away */
#define NEIGH_OUT_HZ     1600e3f
#define MERGE_HZ         1000e3f /* one signal lights several grid slots: keep one */
/* Per-window tests. A window passes when its concentration is high AND the
 * opposite slope's is much lower: noise and non-chirps fail the second. */
#define SCORE_MIN_FRAC   0.25f  /* of the window length, i.e. of a perfect chirp */
#define SCORE_WEAK_FRAC  0.18f  /* the weak tier (lora_detect.h): ~1.5 dB more */
#define SCORE_MIN_ABS    12.0f  /* noise alone scores ~7-9 */
#define ASYM_MIN         2.5f
/* Snapshot verdict for one SF and slope. */
#define HIT_RATIO_MIN    0.3f

#define TWO_PI 6.28318530717958647692f

/* ------------------------------------------------------------ samples */

void lora_unpack(uint32_t word, float *i, float *q)
{
    int32_t a = (int32_t)(word & 0x3ffu);
    int32_t b = (int32_t)((word >> 10) & 0x3ffu);
    a -= (a & 0x200) << 1;   /* sign-extend 10 bits */
    b -= (b & 0x200) << 1;
    *i = (float)a;
    *q = (float)b;
}

uint32_t lora_snapshot_len(const lora_snapshot_t *s)
{
    uint32_t n = 0;
    for (unsigned k = 0; k < s->nseg && k < LORA_MAX_SEGS; k++) {
        n += s->seg[k].n;
    }
    return n;
}

/* Sequential reader over the segments. */
typedef struct {
    const lora_snapshot_t *s;
    unsigned seg;
    uint32_t off;
} reader_t;

static void reader_seek(reader_t *r, const lora_snapshot_t *s, uint32_t pos)
{
    r->s = s;
    r->seg = 0;
    while (r->seg < s->nseg && pos >= s->seg[r->seg].n) {
        pos -= s->seg[r->seg].n;
        r->seg++;
    }
    r->off = pos;
}

static bool reader_next(reader_t *r, float *i, float *q)
{
    while (r->seg < r->s->nseg && r->off >= r->s->seg[r->seg].n) {
        r->seg++;
        r->off = 0;
    }
    if (r->seg >= r->s->nseg) {
        return false;
    }
    lora_unpack(r->s->seg[r->seg].p[r->off++], i, q);
    return true;
}

/* ------------------------------------------------------------ FFT */

void lora_fft(float *x, unsigned n)
{
    /* bit reversal */
    for (unsigned i = 1, j = 0; i < n; i++) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float t = x[2 * i];
            x[2 * i] = x[2 * j];
            x[2 * j] = t;
            t = x[2 * i + 1];
            x[2 * i + 1] = x[2 * j + 1];
            x[2 * j + 1] = t;
        }
    }
    /* butterflies; twiddles by recurrence, exact enough for n <= 512 */
    for (unsigned len = 2; len <= n; len <<= 1) {
        const float ang = -TWO_PI / (float)len;
        const float wr = cosf(ang), wi = sinf(ang);
        for (unsigned i = 0; i < n; i += len) {
            float cr = 1.0f, ci = 0.0f;
            for (unsigned k = 0; k < len / 2; k++) {
                float *a = x + 2 * (i + k);
                float *b = x + 2 * (i + k + len / 2);
                const float tr = b[0] * cr - b[1] * ci;
                const float ti = b[0] * ci + b[1] * cr;
                b[0] = a[0] - tr;
                b[1] = a[1] - ti;
                a[0] += tr;
                a[1] += ti;
                const float nr = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = nr;
            }
        }
    }
}

/* The FFT the scan uses. On the S3: Espressif's optimised esp-dsp routine,
 * checked on the host against lora_fft() (same e^-j convention, largest
 * difference 1.6e-5). esp-dsp's init keeps a global pointer to the twiddle
 * table, so the table is static (1 KB), never scratch. Elsewhere: lora_fft. */
#if defined(ESP_PLATFORM)
static float s_twiddle[LORA_FFT_MAX] __attribute__((aligned(16)));
static bool  s_dsp_ready;

static void fft(float *x, unsigned n)
{
    /* The S3 routine loads the data in 64-bit pairs (ee.ldf.64.ip). The heap
     * only promises 4-byte alignment (ESP-IDF TLSF, ALIGN_SIZE_LOG2 = 2),
     * and a misaligned buffer gave garbage spectra on hardware (BUGLOG).
     * lora_scan() aligns its buffers; this guard keeps any other caller
     * correct. */
    if ((uintptr_t)x & 15u) {
        lora_fft(x, n);
        return;
    }
    if (!s_dsp_ready) {
        s_dsp_ready = dsps_fft2r_init_fc32(s_twiddle, LORA_FFT_MAX) == ESP_OK;
    }
    if (s_dsp_ready && dsps_fft2r_fc32(x, (int)n) == ESP_OK && dsps_bit_rev_fc32(x, (int)n) == ESP_OK) {
        return;
    }
    lora_fft(x, n);   /* never expected; keeps the scan correct regardless */
}
#else
static void fft(float *x, unsigned n) { lora_fft(x, n); }
#endif

/* ------------------------------------------------------------ candidates */

typedef struct {
    float freq_hz;
    float db;      /* over the quieter neighbour */
    float noise;   /* noise power per full-rate sample next to this channel */
} cand_t;

static int cmp_float(const void *a, const void *b)
{
    const float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

/* Mean coarse power over [lo, hi) Hz (signed offsets from the LO). */
static float band_power(const float *power, unsigned n, float bin_hz, float lo, float hi)
{
    int k0 = (int)ceilf(lo / bin_hz), k1 = (int)floorf(hi / bin_hz);
    float sum = 0.0f;
    unsigned cnt = 0;
    for (int k = k0; k <= k1; k++) {
        if (k >= -1 && k <= 1) {
            continue;   /* DC / LO residue */
        }
        if (k < -(int)n / 2 || k >= (int)n / 2) {
            continue;
        }
        sum += power[(unsigned)((k + (int)n) % (int)n)];
        cnt++;
    }
    return cnt ? sum / (float)cnt : 0.0f;
}

static void keep_best(cand_t *out, unsigned *found, cand_t c)
{
    /* Same signal as one already kept: keep whichever stands out more. */
    for (unsigned k = 0; k < *found; k++) {
        if (fabsf(out[k].freq_hz - c.freq_hz) < MERGE_HZ) {
            if (c.db <= out[k].db) {
                return;
            }
            for (unsigned j = k; j + 1 < *found; j++) {
                out[j] = out[j + 1];
            }
            (*found)--;
            break;
        }
    }
    unsigned pos = *found < LORA_MAX_CAND ? *found : LORA_MAX_CAND;
    while (pos > 0 && out[pos - 1].db < c.db) {
        if (pos < LORA_MAX_CAND) {
            out[pos] = out[pos - 1];
        }
        pos--;
    }
    if (pos < LORA_MAX_CAND) {
        out[pos] = c;
        if (*found < LORA_MAX_CAND) {
            (*found)++;
        }
    }
}

/* Average power spectrum over the snapshot, then rank the ELRS grid
 * positions by how far each stands out from its neighbours: a narrow LoRa
 * burst does, a 20 MHz Wi-Fi plateau does not. */
static unsigned find_candidates(const lora_snapshot_t *s, float *power, float *buf, lora_floor_t *fl,
                                uint8_t agc, cand_t *out, float *clip_frac)
{
    const unsigned n = LORA_COARSE_N;
    const uint32_t total = lora_snapshot_len(s);
    const uint32_t blocks = total / n < COARSE_BLOCKS ? total / n : COARSE_BLOCKS;
    if (blocks == 0) {
        return 0;
    }
    /* Hann window by a rotating phasor: no table (static RAM is scarce, it
     * must end below the RF ring) and no cosf() per sample (slow on the S3). */
    const float wr = cosf(TWO_PI / (float)n), wi = sinf(TWO_PI / (float)n);
    memset(power, 0, n * sizeof(float));
    uint32_t clipped = 0;
    for (uint32_t b = 0; b < blocks; b++) {
        reader_t r;
        reader_seek(&r, s, (uint32_t)((uint64_t)b * (total - n) / (blocks > 1 ? blocks - 1 : 1)));
        float cr = 1.0f, ci = 0.0f;   /* cos/sin of 2*pi*k/n */
        for (unsigned k = 0; k < n; k++) {
            float i = 0.0f, q = 0.0f;
            (void)reader_next(&r, &i, &q);
            clipped += i >= 511.0f || i <= -512.0f || q >= 511.0f || q <= -512.0f;
            const float w = 0.5f - 0.5f * cr;
            buf[2 * k] = i * w;
            buf[2 * k + 1] = q * w;
            const float nr = cr * wr - ci * wi;
            ci = cr * wi + ci * wr;
            cr = nr;
        }
        fft(buf, n);
        for (unsigned k = 0; k < n; k++) {
            power[k] += buf[2 * k] * buf[2 * k] + buf[2 * k + 1] * buf[2 * k + 1];
        }
    }

    *clip_frac = (float)clipped / ((float)blocks * (float)n);

    /* median of the power, via a sorted copy in the FFT buffer */
    memcpy(buf, power, n * sizeof(float));
    qsort(buf, n, sizeof(float), cmp_float);
    const float median = buf[n / 2] > 0.0f ? buf[n / 2] : 1e-9f;
    /* Coarse bin power -> noise power per sample: E|X|^2 = sigma^2 * sum(w^2)
     * for white noise, and Hann's sum(w^2) = 3n/8. */
    const float per_sample = 1.0f / ((float)blocks * 0.375f * (float)n);

    /* Use (and update) the floor only at the gain it was learned at. */
    if (fl && fl->ready && agc != fl->agc) {
        if (++fl->other_agc >= FLOOR_RELEARN) {
            fl->ready = false;   /* the gain moved for good: relearn below */
        } else {
            fl = NULL;           /* this snapshot only */
        }
    } else if (fl) {
        fl->other_agc = 0;
    }

    unsigned found = 0;
    const float bin_hz = (float)s->fs_hz / (float)n;
    for (int mhz = -(int)(EDGE_HZ / 1e6f); mhz <= (int)(EDGE_HZ / 1e6f); mhz++) {
        for (int side = -1; side <= 1; side += 2) {
            const float f = (float)mhz * 1e6f + (float)side * GRID_FRAC_HZ;
            if (fabsf(f) > EDGE_HZ) {
                continue;
            }
            const float pc = band_power(power, n, bin_hz, f - CHAN_HALF_HZ, f + CHAN_HALF_HZ);
            const float pl = band_power(power, n, bin_hz, f - NEIGH_OUT_HZ, f - NEIGH_IN_HZ);
            const float pr = band_power(power, n, bin_hz, f + NEIGH_IN_HZ, f + NEIGH_OUT_HZ);
            const float pn = fminf(pl, pr) > 0.0f ? fminf(pl, pr) : median;
            /* Over the long-term floor of the same bins when there is one:
             * that removes filter edges (lora_floor_t). Else the median. */
            const float ref = fl && fl->ready ? band_power(fl->bin, n, bin_hz, f - CHAN_HALF_HZ, f + CHAN_HALF_HZ)
                                              : median;
            if (pc <= 0.0f || ref <= 0.0f || 10.0f * log10f(pc / ref) < CAND_FLOOR_DB) {
                continue;
            }
            const float excess = 10.0f * log10f(pc / pn);
            if (excess >= CAND_MIN_DB) {
                /* Noise for the SNR: the quieter neighbour, NOT the median:
                 * at 80 MS/s most bins lie outside the analog filter (esp-sdr
                 * docs/rx-controls.md), and the median made SNRs read ~23 dB
                 * high (host test). */
                keep_best(out, &found, (cand_t){ f, excess, pn * per_sample });
            }
        }
    }
    if (fl) {
        for (unsigned k = 0; k < n; k++) {
            if (!fl->ready) {
                fl->bin[k] = power[k];
            } else if (power[k] < fl->bin[k]) {
                fl->bin[k] += FLOOR_FALL * (power[k] - fl->bin[k]);
            } else {
                fl->bin[k] *= FLOOR_RISE;
            }
        }
        if (!fl->ready) {
            fl->agc = agc;
            fl->other_agc = 0;
        }
        fl->ready = true;
    }
    return found;
}

/* ------------------------------------------------------------ baseband */

/* Mix the candidate to 0 Hz, then decimate 80 -> 10 -> 1.25 MS/s:
 *   stage 1: triangular 15-tap (two cascaded 8-sample boxcars), double null
 *            at 10 MHz, so nothing near 10 MHz folds into the band;
 *   stage 2: 96-tap Hamming windowed-sinc, cut-off 0.6 MHz: passes LoRa's
 *            +-406 kHz, rejects a neighbour 1-2 MHz away by ~50 dB.
 * A plain 64-sample integrate-and-dump let a carrier 1.9 MHz away through
 * at -13 dB, enough to bury the chirp (found by the host tests). */
#define S1_DECIM 8u
#define S1_TAPS  15u
#define S2_DECIM 8u
#define S2_TAPS  LORA_FIR_TAPS

/* Taps live in the caller's scratch, not in static RAM. 96 sinf/cosf per
 * scan: negligible. */
static void design_taps(float *s_taps)
{
    const float fc = 0.6e6f / 10e6f;   /* cycles per stage-1 sample */
    float sum = 0.0f;
    for (unsigned n = 0; n < S2_TAPS; n++) {
        const float x = (float)n - (float)(S2_TAPS - 1) / 2.0f;
        const float sinc = x == 0.0f ? 2.0f * fc : sinf(TWO_PI * fc * x) / (3.14159265f * x);
        const float hamming = 0.54f - 0.46f * cosf(TWO_PI * (float)n / (float)(S2_TAPS - 1));
        s_taps[n] = sinc * hamming;
        sum += s_taps[n];
    }
    for (unsigned n = 0; n < S2_TAPS; n++) {
        s_taps[n] /= sum;
    }
}

static unsigned to_baseband(const lora_snapshot_t *s, float freq_hz, const float *s_taps, float *z,
                            unsigned max_out)
{
    reader_t r;
    reader_seek(&r, s, 0);
    const float step = -TWO_PI * freq_hz / (float)s->fs_hz;
    const float sr = cosf(step), si = sinf(step);
    float pr = 1.0f, pi = 0.0f;                 /* rotating phasor */
    float d1[2 * S1_TAPS] = { 0 }, d2[2 * S2_TAPS] = { 0 };
    unsigned h1 = 0, h2 = 0, n1 = 0, n2 = 0, fill1 = 0, fill2 = 0, m = 0;
    float i, q;
    while (m < max_out && reader_next(&r, &i, &q)) {
        /* mix */
        d1[2 * h1] = i * pr - q * pi;
        d1[2 * h1 + 1] = i * pi + q * pr;
        h1 = (h1 + 1) % S1_TAPS;
        fill1 += fill1 < S1_TAPS;
        const float nr = pr * sr - pi * si;
        pi = pr * si + pi * sr;
        pr = nr;
        if (++n1 < S1_DECIM) {
            continue;
        }
        n1 = 0;
        /* keep |phasor| = 1 against rounding drift */
        const float mag = sqrtf(pr * pr + pi * pi);
        pr /= mag;
        pi /= mag;
        if (fill1 < S1_TAPS) {
            continue;
        }
        /* stage 1: weights 1..8..1 over the last 15 samples */
        float ar = 0.0f, ai = 0.0f;
        for (unsigned k = 0; k < S1_TAPS; k++) {
            const unsigned idx = (h1 + k) % S1_TAPS;            /* oldest first */
            const float w = (float)(k < 8 ? k + 1 : S1_TAPS - k);
            ar += w * d1[2 * idx];
            ai += w * d1[2 * idx + 1];
        }
        d2[2 * h2] = ar / 64.0f;
        d2[2 * h2 + 1] = ai / 64.0f;
        h2 = (h2 + 1) % S2_TAPS;
        fill2 += fill2 < S2_TAPS;
        if (++n2 < S2_DECIM) {
            continue;
        }
        n2 = 0;
        if (fill2 < S2_TAPS) {
            continue;
        }
        /* stage 2 */
        float br = 0.0f, bi = 0.0f;
        for (unsigned k = 0; k < S2_TAPS; k++) {
            const unsigned idx = (h2 + k) % S2_TAPS;
            br += s_taps[k] * d2[2 * idx];
            bi += s_taps[k] * d2[2 * idx + 1];
        }
        z[2 * m] = br;
        z[2 * m + 1] = bi;
        m++;
    }
    return m;
}

unsigned lora_baseband(const lora_snapshot_t *s, float freq_hz, float *z, unsigned max)
{
    if (s == NULL || z == NULL || s->fs_hz != 80000000u) {
        return 0;
    }
    float taps[S2_TAPS];
    design_taps(taps);
    return to_baseband(s, freq_hz, taps, z, max);
}

/* ------------------------------------------------------------ dechirp */

/* conj of an up-chirp at the LoRa rate for this SF: exp(-j*pi*k*n^2). */
static void make_ref(float *ref, unsigned len, unsigned sf, float rate)
{
    /* cycles/sample^2. The phase reaches ~1,700 cycles at SF5 over 512
     * samples, beyond float's precision, so it is formed in double and
     * reduced mod 1 before the trig. Once per SF per candidate: cheap. */
    const double k = (double)LORA_BW_HZ * LORA_BW_HZ / (double)(1u << sf) / ((double)rate * rate);
    for (unsigned n = 0; n < len; n++) {
        double cyc = 0.5 * k * (double)n * (double)n;
        cyc -= floor(cyc);
        ref[2 * n] = cosf(TWO_PI * (float)cyc);
        ref[2 * n + 1] = -sinf(TWO_PI * (float)cyc);
    }
}

/* Concentration of one window after multiplying by ref (slope +1) or its
 * conjugate (slope -1): energy in the peak bin and its two neighbours over
 * the mean bin energy. A perfect chirp scores about the window length. */
static float window_score(const float *z, const float *ref, unsigned len, unsigned nfft, int slope,
                          float *buf)
{
    for (unsigned n = 0; n < len; n++) {
        const float rr = ref[2 * n], ri = slope > 0 ? ref[2 * n + 1] : -ref[2 * n + 1];
        buf[2 * n] = z[2 * n] * rr - z[2 * n + 1] * ri;
        buf[2 * n + 1] = z[2 * n] * ri + z[2 * n + 1] * rr;
    }
    memset(buf + 2 * len, 0, (nfft - len) * 2 * sizeof(float));
    fft(buf, nfft);
    float total = 0.0f, peak = 0.0f;
    unsigned at = 0;
    for (unsigned k = 0; k < nfft; k++) {
        const float p = buf[2 * k] * buf[2 * k] + buf[2 * k + 1] * buf[2 * k + 1];
        buf[2 * k] = p;   /* keep power for the neighbour sum */
        total += p;
        if (p > peak) {
            peak = p;
            at = k;
        }
    }
    if (total <= 0.0f) {
        return 0.0f;
    }
    const float near = peak + buf[2 * ((at + 1) % nfft)] + buf[2 * ((at + nfft - 1) % nfft)];
    return near / (total / (float)nfft);
}

static unsigned next_pow2(unsigned v)
{
    unsigned p = 1;
    while (p < v) {
        p <<= 1;
    }
    return p;
}

/* Test one SF on the baseband samples, both slopes, sliding by half a symbol. */
static lora_sf_metric_t test_sf(const float *z, unsigned m, unsigned sf, float rate, float *ref, float *buf)
{
    lora_sf_metric_t best = { 0 };
    const float sym = (float)(1u << sf) * rate / LORA_BW_HZ;   /* samples per symbol */
    unsigned len = (unsigned)sym;
    if (len > m) {
        len = m;           /* SF8 can be longer than a short snapshot */
    }
    if (len < 16) {
        return best;
    }
    const unsigned nfft = next_pow2(len) <= LORA_FFT_MAX ? next_pow2(len) : LORA_FFT_MAX;
    if (len > nfft) {
        len = nfft;
    }
    make_ref(ref, len, sf, rate);

    const unsigned hop = len / 2 > 0 ? len / 2 : 1;
    unsigned windows = 0, hits_up = 0, hits_down = 0, weak_up = 0, weak_down = 0;
    float best_up = 0.0f, best_down = 0.0f, asym_up = 0.0f, asym_down = 0.0f;
    for (unsigned s = 0; s + len <= m; s += hop) {
        const float up = window_score(z + 2 * s, ref, len, nfft, +1, buf);
        const float down = window_score(z + 2 * s, ref, len, nfft, -1, buf);
        const float floor_score = fmaxf(SCORE_MIN_ABS, SCORE_MIN_FRAC * (float)len);
        const float floor_weak = fmaxf(SCORE_MIN_ABS, SCORE_WEAK_FRAC * (float)len);
        windows++;
        if (up >= floor_weak && up >= ASYM_MIN * down) {
            weak_up++;
            hits_up += up >= floor_score;
        }
        if (down >= floor_weak && down >= ASYM_MIN * up) {
            weak_down++;
            hits_down += down >= floor_score;
        }
        if (up > best_up) {
            best_up = up;
            asym_up = up / fmaxf(down, 1e-6f);
        }
        if (down > best_down) {
            best_down = down;
            asym_down = down / fmaxf(up, 1e-6f);
        }
    }
    if (windows == 0) {
        return best;
    }
    const bool up_wins = hits_up != hits_down ? hits_up > hits_down
                       : weak_up != weak_down ? weak_up > weak_down
                       : best_up >= best_down;
    best.slope = up_wins ? +1 : -1;
    best.score = up_wins ? best_up : best_down;
    best.asym = up_wins ? asym_up : asym_down;
    best.hit_ratio = (float)(up_wins ? hits_up : hits_down) / (float)windows;
    best.hit_ratio_weak = (float)(up_wins ? weak_up : weak_down) / (float)windows;
    return best;
}

static bool sf_passes(const lora_sf_metric_t *m, bool weak)
{
    return (weak ? m->hit_ratio_weak : m->hit_ratio) >= HIT_RATIO_MIN && m->asym >= ASYM_MIN;
}

/* Strength of a detected signal: the loudest window's mean power (the
 * packet may cover only part of the snapshot) against the noise expected
 * in the decimator's ~1.2 MHz bandwidth, from the neighbour bands. */
static void measure(const float *z, unsigned m, unsigned sf, float noise_per_sample, lora_result_t *out)
{
    unsigned len = (unsigned)((float)(1u << sf) * (float)LORA_DECIM_RATE / LORA_BW_HZ);
    len = len > m ? m : len;
    float best = 0.0f;
    for (unsigned s = 0; len > 0 && s + len <= m; s += len) {
        float p = 0.0f;
        for (unsigned n = s; n < s + len; n++) {
            p += z[2 * n] * z[2 * n] + z[2 * n + 1] * z[2 * n + 1];
        }
        best = fmaxf(best, p / (float)len);
    }
    const float full_scale = 512.0f * 512.0f;   /* 10-bit complex full scale */
    const float noise = noise_per_sample * (1.2e6f / 80e6f);
    out->level_dbfs = 10.0f * log10f(fmaxf(best, 1e-6f) / full_scale);
    out->snr_db = 10.0f * log10f(fmaxf(best - noise, 1e-6f) / fmaxf(noise, 1e-9f));
}

/* ------------------------------------------------------------ scan */

bool lora_scan(const lora_snapshot_t *s, float *work, size_t work_floats, lora_floor_t *floor,
               lora_result_t *out)
{
    /* The decimator is designed for 80 MS/s (64 x 1.25 MS/s), the rate the
     * firmware uses. */
    if (s == NULL || work == NULL || out == NULL || work_floats < LORA_WORK_FLOATS || s->fs_hz != 80000000u ||
        s->nseg > LORA_MAX_SEGS) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    /* 16-byte alignment for every buffer below: all their sizes are
     * multiples of 4 floats, so aligning the start aligns them all. */
    float *aligned = (float *)(((uintptr_t)work + 15u) & ~(uintptr_t)15u);
    float *power = aligned;
    float *buf = power + LORA_COARSE_N;          /* 2 * LORA_FFT_MAX */
    float *ref = buf + 2 * LORA_FFT_MAX;         /* 2 * LORA_FFT_MAX */
    float *z = ref + 2 * LORA_FFT_MAX;           /* 2 * LORA_MAX_BASEBAND */
    float *taps = z + 2 * LORA_MAX_BASEBAND;     /* LORA_FIR_TAPS */
    design_taps(taps);

    cand_t cand[LORA_MAX_CAND];
    out->agc = s->nseg > 0 && s->seg[0].n > 0 ? (uint8_t)(s->seg[0].p[0] >> 20) : 0;
    out->candidates = find_candidates(s, power, buf, floor, out->agc, cand, &out->clip_frac);

    const float rate = (float)LORA_DECIM_RATE;
    float best_rank = -1.0f;

    for (unsigned c = 0; c < out->candidates; c++) {
        const unsigned m = to_baseband(s, cand[c].freq_hz, taps, z, LORA_MAX_BASEBAND);
        lora_sf_metric_t metric[LORA_SF_MAX - LORA_SF_MIN + 1u];
        int pick = -1, pick_weak = -1;
        float pick_rank = -1.0f, weak_rank = -1.0f;
        for (unsigned sf = LORA_SF_MIN; sf <= LORA_SF_MAX; sf++) {
            lora_sf_metric_t *mt = &metric[sf - LORA_SF_MIN];
            *mt = test_sf(z, m, sf, rate, ref, buf);
            /* Rank by how close to a perfect chirp the best window came,
             * relative to that SF's window length: the right SF wins. */
            const float len = fminf((float)(1u << sf) * rate / LORA_BW_HZ, (float)m);
            const float rank = mt->score / fmaxf(len, 1.0f);
            if (sf_passes(mt, false) && rank > pick_rank) {
                pick_rank = rank;
                pick = (int)sf;
            }
            if (sf_passes(mt, true) && rank > weak_rank) {
                weak_rank = rank;
                pick_weak = (int)sf;
            }
        }
        /* Report the best candidate: a full pass beats a weak one beats
         * none; with none, the strongest (so the log shows what was measured). */
        const int use = pick >= 0 ? pick : pick_weak;
        const float cand_rank = pick >= 0 ? 2.0f + pick_rank
                              : pick_weak >= 0 ? 1.0f + weak_rank
                              : (c == 0 ? 0.0f : -1.0f);
        if (cand_rank > best_rank) {
            best_rank = cand_rank;
            out->detected = pick >= 0;
            out->weak = use >= 0;
            out->sf = use >= 0 ? (uint8_t)use : 0;
            out->slope = use >= 0 ? metric[use - (int)LORA_SF_MIN].slope : 0;
            out->freq_hz = cand[c].freq_hz;
            out->cand_db = cand[c].db;
            memcpy(out->sf_metric, metric, sizeof(metric));
            if (use >= 0) {
                measure(z, m, out->sf, cand[c].noise, out);   /* now: the next candidate reuses z */
            }
        }
        if (out->detected) {
            break;   /* presence is the question: no need to test the rest */
        }
    }
    return true;
}
