/* lora_synth.c - see lora_synth.h. Double precision throughout: the test
 * signal must be cleaner than anything the detector is asked to find. */
#include "lora_synth.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define LORA_BW 812500.0

static uint32_t rnd(synth_t *s)
{
    s->rng ^= s->rng << 13;
    s->rng ^= s->rng >> 17;
    s->rng ^= s->rng << 5;
    return s->rng;
}

static double unit(synth_t *s) { return (rnd(s) + 0.5) / 4294967296.0; }

static double gauss(synth_t *s)
{
    return sqrt(-2.0 * log(unit(s))) * cos(2.0 * PI * unit(s));
}

void synth_init(synth_t *s, uint32_t n, uint32_t seed)
{
    s->n = n;
    s->i = calloc(n, sizeof(double));
    s->q = calloc(n, sizeof(double));
    s->rng = seed ? seed : 1u;
}

void synth_free(synth_t *s)
{
    free(s->i);
    free(s->q);
    s->i = s->q = NULL;
}

/* Instantaneous frequency of LoRa at time t (s) into the packet, or NAN
 * outside it. Symbols: preamble up-chirps (value 0), two sync-word symbols,
 * 2.25 down-chirps, then data. */
static double lora_freq(unsigned sf, double t, unsigned preamble, const unsigned *data, unsigned ndata)
{
    const double T = (double)(1u << sf) / LORA_BW;
    const unsigned M = 1u << sf;
    if (t < 0) {
        return NAN;
    }
    double sym = t / T;
    unsigned k = (unsigned)sym;
    double frac = sym - k;
    unsigned value;
    if (k < preamble) {
        value = 0;
    } else if (k < preamble + 2) {
        value = (k == preamble ? 8u : 16u) % M;   /* sync word: any non-zero values */
    } else if (sym < preamble + 4.25) {
        return LORA_BW / 2.0 - LORA_BW * (frac);   /* down-chirp */
    } else {
        const double ds = sym - (preamble + 4.25);
        k = (unsigned)ds;
        frac = ds - k;
        if (k >= ndata) {
            return NAN;
        }
        value = data[k];
    }
    double pos = frac + (double)value / M;
    pos -= floor(pos);
    return -LORA_BW / 2.0 + LORA_BW * pos;
}

void synth_lora(synth_t *s, unsigned sf, bool invert_iq, double f_hz, double amp, double start_s,
                unsigned payload_symbols)
{
    unsigned data[256];
    const unsigned M = 1u << sf;
    if (payload_symbols > 256) {
        payload_symbols = 256;
    }
    for (unsigned k = 0; k < payload_symbols; k++) {
        data[k] = rnd(s) % M;
    }
    double phase = 2.0 * PI * unit(s);
    for (uint32_t n = 0; n < s->n; n++) {
        const double t = (double)n / SYNTH_FS - start_s;
        const double f = lora_freq(sf, t, 12, data, payload_symbols);
        if (isnan(f)) {
            continue;
        }
        phase += 2.0 * PI * (invert_iq ? -f : f) / SYNTH_FS;
        const double p = phase + 2.0 * PI * f_hz * (double)n / SYNTH_FS;
        s->i[n] += amp * cos(p);
        s->q[n] += amp * sin(p);
    }
}

void synth_gfsk(synth_t *s, double f_hz, double amp)
{
    const unsigned spb = (unsigned)(SYNTH_FS / 1e6);   /* samples per bit */
    double phase = 0.0, dev = 0.0, target = 0.0;
    for (uint32_t n = 0; n < s->n; n++) {
        if (n % spb == 0) {
            target = (rnd(s) & 1u) ? 160e3 : -160e3;
        }
        dev += (target - dev) * 0.05;                  /* Gaussian-ish smoothing */
        phase += 2.0 * PI * (f_hz + dev) / SYNTH_FS;
        s->i[n] += amp * cos(phase);
        s->q[n] += amp * sin(phase);
    }
}

void synth_ofdm(synth_t *s, double f_hz, double amp)
{
    const unsigned sym = (unsigned)(SYNTH_FS * 4e-6);
    double ai[52], aq[52];
    for (uint32_t n = 0; n < s->n; n++) {
        if (n % sym == 0) {
            for (unsigned c = 0; c < 52; c++) {
                ai[c] = (rnd(s) & 1u) ? 1.0 : -1.0;
                aq[c] = (rnd(s) & 1u) ? 1.0 : -1.0;
            }
        }
        double re = 0.0, im = 0.0;
        for (unsigned c = 0; c < 52; c++) {
            const int k = (int)c - 26 + (c >= 26 ? 1 : 0);  /* skip DC subcarrier */
            const double p = 2.0 * PI * (f_hz + k * 312500.0) * (double)n / SYNTH_FS;
            re += ai[c] * cos(p) - aq[c] * sin(p);
            im += ai[c] * sin(p) + aq[c] * cos(p);
        }
        s->i[n] += amp * re / sqrt(104.0);
        s->q[n] += amp * im / sqrt(104.0);
    }
}

void synth_cw(synth_t *s, double f_hz, double amp)
{
    for (uint32_t n = 0; n < s->n; n++) {
        const double p = 2.0 * PI * f_hz * (double)n / SYNTH_FS;
        s->i[n] += amp * cos(p);
        s->q[n] += amp * sin(p);
    }
}

void synth_noise(synth_t *s, double sigma)
{
    for (uint32_t n = 0; n < s->n; n++) {
        s->i[n] += sigma * gauss(s);
        s->q[n] += sigma * gauss(s);
    }
}

/* The receiver's analog low-pass, applied to everything added so far, then
 * the ADC's own white noise. A 127-tap Hamming windowed-sinc: a model of a
 * smooth roll-off, not of the S3's actual curve. */
void synth_analog_filter(synth_t *s, double bw_hz, double adc_sigma)
{
    enum { TAPS = 127 };
    double h[TAPS], sum = 0.0;
    const double fc = 0.5 * bw_hz / SYNTH_FS;   /* cycles per sample, one-sided */
    for (int k = 0; k < TAPS; k++) {
        const double x = k - (TAPS - 1) / 2.0;
        const double sinc = x == 0.0 ? 2.0 * fc : sin(2.0 * M_PI * fc * x) / (M_PI * x);
        h[k] = sinc * (0.54 - 0.46 * cos(2.0 * M_PI * k / (TAPS - 1)));
        sum += h[k];
    }
    double *fi = calloc(s->n, sizeof(double)), *fq = calloc(s->n, sizeof(double));
    if (fi == NULL || fq == NULL) {
        free(fi);
        free(fq);
        return;
    }
    for (uint32_t n = 0; n < s->n; n++) {
        double ai = 0.0, aq = 0.0;
        for (int k = 0; k < TAPS; k++) {
            const int64_t m = (int64_t)n - k + (TAPS - 1) / 2;
            if (m >= 0 && m < (int64_t)s->n) {
                ai += h[k] * s->i[m];
                aq += h[k] * s->q[m];
            }
        }
        fi[n] = ai / sum + adc_sigma * gauss(s);
        fq[n] = aq / sum + adc_sigma * gauss(s);
    }
    memcpy(s->i, fi, s->n * sizeof(double));
    memcpy(s->q, fq, s->n * sizeof(double));
    free(fi);
    free(fq);
}

uint8_t synth_gain_index = 40;

static int32_t quant(double v, uint32_t *clipped)
{
    long x = lround(v);
    if (x > 511) {
        x = 511;
        (*clipped)++;
    } else if (x < -512) {
        x = -512;
        (*clipped)++;
    }
    return (int32_t)x;
}

uint32_t synth_pack(const synth_t *s, uint32_t *words)
{
    uint32_t clipped = 0;
    for (uint32_t n = 0; n < s->n; n++) {
        const uint32_t i = (uint32_t)quant(s->i[n], &clipped) & 0x3ffu;
        const uint32_t q = (uint32_t)quant(s->q[n], &clipped) & 0x3ffu;
        words[n] = i | (q << 10) | ((uint32_t)synth_gain_index << 20);   /* gain metadata */
    }
    return clipped;
}
