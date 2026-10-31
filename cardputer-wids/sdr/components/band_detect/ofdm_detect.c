/* ofdm_detect.c - see ofdm_detect.h. */
#include "ofdm_detect.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265f

/* One delay per family: Tu at 10 MS/s, rounded (333.3 -> 333 and 666.7 ->
 * 667 lose little: the copy is then 33 ns off, a few percent of a 9 MHz
 * signal's coherence). Correlation products are folded by their position
 * in the symbol period: the prefix pairs all land in the same few timing
 * bins and add up, everything else spreads over all bins.
 *   Wi-Fi: 4.0 us symbols (0.8 us prefix): 40 samples, 20 bins, prefix ~4 bins.
 *   LTE-like: 500 us holds 7 (15 kHz) or 14 (30 kHz) symbols with a ~4.7 /
 *   2.3 us prefix; folding at the mean period (71.4 / 35.7 us) puts the
 *   prefix in ~2 of 32 bins. The slightly longer first prefix of each
 *   0.5 ms slot shifts timing by under one bin per snapshot. */
typedef struct {
    uint16_t lag;
    float    period;   /* symbol period, 10 MS/s samples */
    uint8_t  bins, win;
} family_t;

static const family_t k_family[OFDM_KINDS] = {
    [OFDM_WIFI] = { 32, 40.0f, 20, 4 },
    [OFDM_LTE30] = { 333, 5000.0f / 14.0f, 32, 3 },
    [OFDM_LTE15] = { 667, 5000.0f / 7.0f, 32, 3 },
};
#define MAX_FOLD 32u

/* Thresholds on the running mean of a slot's scores (robust z, see
 * fold_score). Host statistics, test/test_band_detect.c and the probe in
 * BUGLOG: noise and non-OFDM signals average ~2.3 (sd 0.7 per scan, ~0.3 for
 * the running mean); LTE-like OFDM averages ~9 at +3 dB in the slot, ~4 at
 * -3 dB; Wi-Fi ~8 at -3 dB. */
/* LTE-like is set higher: ELRS hopping through a slot without filling it
 * slips past the narrowband gate, and for some spreading factors its
 * delayed product is a tone that lines up with the LTE symbol period (SF6:
 * 688 kHz x 71.4 us = 49.1 cycles) and folds coherently. In the simulator a
 * few such scans lifted a slot's mean from ~2.3 to 5.0. */
static const float k_on[OFDM_KINDS] = { 4.5f, 5.5f, 5.5f };
#define AVG_WEIGHT  0.3f
/* For the LTE-like kinds one scan may pull the mean up by at most
 * AVG_WEIGHT x k_step_max: a single wild scan (in the simulator, a Wi-Fi
 * burst plus part of an ELRS chirp scored ~15 and lifted a slot from 2.3 to
 * 6.1 at once) can no longer make a verdict. A DJI-style downlink is
 * continuous, keeps scoring high, and still crosses the threshold within
 * MIN_SCANS. Wi-Fi is bursty (most snapshots catch no packet, the rest score
 * ~20): capped, its mean settled near 3.3, under its threshold, so Wi-Fi is
 * not capped. A new slot starts at AVG_START, near the noise mean. */
static const float k_step_max[OFDM_KINDS] = { 1e9f, 3.0f, 3.0f };
#define AVG_START    3.0f
#define MIN_SCANS   6u
#define HOLD_US     8000000ull     /* a kind stays on this long after its last strong mean */
#define STALE_US    20000000ull    /* a slot not scanned for this long starts afresh */

const char *ofdm_kind_name(ofdm_kind_t k)
{
    switch (k) {
    case OFDM_WIFI: return "wifi";
    case OFDM_LTE30: return "lte30k";
    case OFDM_LTE15: return "lte15k";
    default: return "?";
    }
}

typedef struct {
    float    re[MAX_FOLD], im[MAX_FOLD];
    uint32_t n[MAX_FOLD];
    float    pos;          /* position of the current sample in the symbol period */
} fold_t;

/* Lives in the caller's scratch after the delay line: 4-byte members only. */
typedef struct {
    fold_t   f[OFDM_KINDS];
    float    power;        /* sum of |y|^2 */
    float    r1_re, r1_im; /* lag-1 correlation: how narrowband the slot is */
    uint32_t m;            /* decimated samples so far */
} corr_t;

_Static_assert(sizeof(corr_t) % sizeof(float) == 0 && sizeof(corr_t) <= OFDM_CORR_FLOATS * sizeof(float),
               "OFDM_CORR_FLOATS too small");

static void corr_push(corr_t *c, float *ring, float yr, float yi)
{
    for (unsigned k = 0; k < OFDM_KINDS; k++) {
        const family_t *fam = &k_family[k];
        fold_t *f = &c->f[k];
        if (c->m >= fam->lag) {
            const float *z = ring + 2u * ((c->m - fam->lag) % OFDM_RING);
            unsigned b = (unsigned)(f->pos * (float)fam->bins / fam->period);
            b = b < fam->bins ? b : fam->bins - 1u;
            f->re[b] += yr * z[0] + yi * z[1];   /* y * conj(z) */
            f->im[b] += yi * z[0] - yr * z[1];
            f->n[b]++;
        }
        f->pos += 1.0f;
        if (f->pos >= fam->period) {
            f->pos -= fam->period;
        }
    }
    if (c->m > 0) {
        const float *z = ring + 2u * ((c->m - 1u) % OFDM_RING);
        c->r1_re += yr * z[0] + yi * z[1];
        c->r1_im += yi * z[0] - yr * z[1];
    }
    float *slot = ring + 2u * (c->m % OFDM_RING);
    slot[0] = yr;
    slot[1] = yi;
    c->power += yr * yr + yi * yi;
    c->m++;
}

static void sort_f(float *v, unsigned n)
{
    for (unsigned i = 1; i < n; i++) {
        const float x = v[i];
        unsigned j = i;
        for (; j > 0 && v[j - 1] > x; j--) {
            v[j] = v[j - 1];
        }
        v[j] = x;
    }
}

/* How far the best timing window stands out from the others, in robust
 * standard deviations: (best - median) / (1.4826 x median absolute
 * deviation), each window as |sum| / (mean power x pairs). The spread is
 * measured per snapshot because it depends on what fills the slot: a 1 MHz
 * Bluetooth or LoRa signal, oversampled ten times at 10 MS/s, makes these
 * sums fluctuate far more than white noise does (host probe: raw
 * best-minus-median 0.22 for GFSK, 0.055 for noise, 0.10 for LTE at -3 dB).
 * A carrier or any non-OFDM signal has no preferred timing; a prefix makes
 * one window stand out. `contrast` gets best - median for the logs. */
static float fold_score(const corr_t *c, unsigned k, uint16_t *best_bin, float *contrast)
{
    const family_t *fam = &k_family[k];
    const fold_t *f = &c->f[k];
    const float mean_p = c->m ? c->power / (float)c->m : 0.0f;
    float rho[MAX_FOLD], dev[MAX_FOLD], best = 0.0f;
    *best_bin = 0;
    for (unsigned b = 0; b < fam->bins; b++) {
        float re = 0.0f, im = 0.0f;
        uint32_t n = 0;
        for (unsigned j = 0; j < fam->win; j++) {
            const unsigned i = (b + j) % fam->bins;
            re += f->re[i];
            im += f->im[i];
            n += f->n[i];
        }
        rho[b] = n && mean_p > 0.0f ? sqrtf(re * re + im * im) / (mean_p * (float)n) : 0.0f;
        if (rho[b] > best) {
            best = rho[b];
            *best_bin = (uint16_t)b;
        }
    }
    sort_f(rho, fam->bins);
    const float med = rho[fam->bins / 2];
    for (unsigned b = 0; b < fam->bins; b++) {
        dev[b] = fabsf(rho[b] - med);
    }
    sort_f(dev, fam->bins);
    *contrast = best - med;
    const float sd = 1.4826f * dev[fam->bins / 2];
    return sd > 1e-6f ? (best - med) / sd : 0.0f;
}

bool ofdm_scan(const lora_snapshot_t *s, float off_hz, float *work, size_t work_floats, ofdm_result_t *out)
{
    if (s == NULL || work == NULL || out == NULL || work_floats < OFDM_WORK_FLOATS || s->fs_hz != 80000000u) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    float *cs = work, *sn = work + 256;
    float *ring = work + 512;
    corr_t *c = (corr_t *)(void *)(ring + 2u * OFDM_RING);   /* ~1 KB: in the scratch, not on the stack */
    memset(c, 0, sizeof(*c));

    /* Mix by a whole number of 312.5 kHz bins: the phase then repeats every
     * 256 samples and comes from a table (scaled by 1024 for the integer
     * filter below). */
    const int k = (int)lrintf(off_hz / ((float)s->fs_hz / 256.0f));
    const uint32_t ku = (uint32_t)k & 255u;
    for (unsigned j = 0; j < 256; j++) {
        cs[j] = 1024.0f * cosf(2.0f * PI_F * (float)j / 256.0f);
        sn[j] = 1024.0f * sinf(2.0f * PI_F * (float)j / 256.0f);
    }

    /* 80 -> 10 MS/s by a third-order CIC (three running sums, decimate by
     * 8, three differences). A plain sum of 8 samples has sidelobes only
     * 13 dB down, and a strong access point 15-30 MHz away then showed its
     * prefix in every slot (simulator); three stages put them ~39 dB down.
     * Integer arithmetic that wraps modulo 2^32 is exact for a CIC as long
     * as the output fits: |mixed| <= 512 * 1024 * sqrt(2), gain 8^3 = 512,
     * so |output| < 3.8e8 < 2^31. */
    uint32_t ir1 = 0, ir2 = 0, ir3 = 0, ii1 = 0, ii2 = 0, ii3 = 0;   /* integrators */
    uint32_t dr1 = 0, dr2 = 0, dr3 = 0, di1 = 0, di2 = 0, di3 = 0;   /* comb delays */
    uint32_t n = 0;
    unsigned in_block = 0, outputs = 0;
    for (unsigned g = 0; g < s->nseg && g < LORA_MAX_SEGS; g++) {
        const lora_seg_t *seg = &s->seg[g];
        for (uint32_t w = 0; seg->p != NULL && w < seg->n; w++, n++) {
            float i, q;
            lora_unpack(seg->p[w], &i, &q);
            const uint32_t t = (ku * n) & 255u;
            /* (i + jq) * e^(-j theta) */
            ir1 += (uint32_t)(int32_t)(i * cs[t] + q * sn[t]);
            ii1 += (uint32_t)(int32_t)(q * cs[t] - i * sn[t]);
            ir2 += ir1;
            ii2 += ii1;
            ir3 += ir2;
            ii3 += ii2;
            if (++in_block < OFDM_DECIM) {
                continue;
            }
            in_block = 0;
            const uint32_t yr1 = ir3 - dr1, yi1 = ii3 - di1;
            dr1 = ir3;
            di1 = ii3;
            const uint32_t yr2 = yr1 - dr2, yi2 = yi1 - di2;
            dr2 = yr1;
            di2 = yi1;
            const uint32_t yr3 = yr2 - dr3, yi3 = yi2 - di3;
            dr3 = yr2;
            di3 = yi2;
            if (++outputs > 3u) {   /* the first three still hold the start-up transient */
                corr_push(c, ring, (float)(int32_t)yr3 * (1.0f / 524288.0f), (float)(int32_t)yi3 * (1.0f / 524288.0f));
            }
        }
    }
    out->samples = c->m;
    out->level_db = c->m ? 10.0f * log10f(c->power / (float)c->m + 1e-12f) : -120.0f;
    out->narrow = c->power > 0.0f ? sqrtf(c->r1_re * c->r1_re + c->r1_im * c->r1_im) / c->power : 0.0f;
    for (unsigned f = 0; f < OFDM_KINDS; f++) {
        out->score[f] = fold_score(c, f, &out->timing[f], &out->contrast[f]);
    }
    return true;
}

/* ------------------------------------------------------------ evidence */

void ofdm_watch_init(ofdm_watch_t *w) { memset(w, 0, sizeof(*w)); }

/* Least recently scanned slot, optionally only among wide ones. */
static int oldest(const ofdm_watch_t *w, const float duty[OFDM_SLOTS], float min_duty, const bool *taken)
{
    int best = -1;
    for (unsigned s = 0; s < OFDM_SLOTS; s++) {
        if (taken[s] || (duty && duty[s] < min_duty)) {
            continue;
        }
        if (best < 0 || w->slot[s].last_scan_us < w->slot[best].last_scan_us) {
            best = (int)s;
        }
    }
    return best;
}

unsigned ofdm_watch_pick(const ofdm_watch_t *w, const float duty[OFDM_SLOTS], float min_duty, unsigned max,
                         unsigned *slots)
{
    /* Every other pick ignores activity: busy Wi-Fi is always "wide", and
     * slots under the spectrum threshold (a weak link) must still be seen. */
    bool taken[OFDM_SLOTS];
    memset(taken, 0, sizeof(taken));
    unsigned n = 0;
    for (unsigned k = 0; n < max && k < OFDM_SLOTS; k++) {
        const bool any = ((w->picks + k) & 1u) != 0;
        int s = oldest(w, any ? NULL : duty, min_duty, taken);
        s = s < 0 ? oldest(w, NULL, min_duty, taken) : s;
        if (s < 0) {
            break;
        }
        taken[s] = true;
        slots[n++] = (unsigned)s;
    }
    return n;
}

void ofdm_watch_update(ofdm_watch_t *w, unsigned slot, const ofdm_result_t *r, uint64_t now_us)
{
    if (slot >= OFDM_SLOTS || r == NULL) {
        return;
    }
    ofdm_slot_t *s = &w->slot[slot];
    w->picks++;
    if (r->narrow > OFDM_NARROW_MAX) {
        /* A narrowband signal dominates the slot: its scores mean nothing
         * (see ofdm_detect.h). Visited, but no evidence either way. */
        s->last_scan_us = now_us;
        w->narrow_skips++;
        return;
    }
    if (s->scans && now_us - s->last_scan_us > STALE_US) {
        memset(s->avg, 0, sizeof(s->avg));
        s->scans = 0;
    }
    for (unsigned k = 0; k < OFDM_KINDS; k++) {
        const float base = s->scans ? s->avg[k] : AVG_START;
        const float x = r->score[k] < base + k_step_max[k] ? r->score[k] : base + k_step_max[k];
        s->avg[k] = base + AVG_WEIGHT * (x - base);
        if (s->scans + 1u >= MIN_SCANS && s->avg[k] >= k_on[k]) {
            s->on_until_us[k] = now_us + HOLD_US;
        }
    }
    s->scans += s->scans < 0xffffu;
    s->last_scan_us = now_us;
    w->scans_total++;
}

float ofdm_watch_threshold(ofdm_kind_t k) { return (unsigned)k < OFDM_KINDS ? k_on[k] : 0.0f; }

bool ofdm_watch_on(const ofdm_watch_t *w, unsigned slot, ofdm_kind_t k, uint64_t now_us)
{
    return slot < OFDM_SLOTS && (unsigned)k < OFDM_KINDS && now_us < w->slot[slot].on_until_us[k];
}
