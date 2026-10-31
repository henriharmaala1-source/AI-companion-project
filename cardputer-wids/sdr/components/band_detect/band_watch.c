/* band_watch.c - see band_watch.h. */
#include "band_watch.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI_F 3.14159265f

/* The LO's own leakage sits in the bins around 0 Hz in every frame: never
 * a signal, and it lies inside the oven range (LO 2442 MHz). */
#define DC_BINS          1
/* A run of occupied bins may contain this many quiet bins (Wi-Fi's empty
 * DC subcarrier, a dip in a max-hold) and still count as one signal. */
#define RUN_GAP_BINS     2
#define WIDE_MIN_KHZ     6000.0f    /* narrower than any Wi-Fi or DJI channel */
#define DUTY_TAU_FRAMES  1500.0f    /* ~3 s of frames */

/* Oven tests, per slice (see band_watch.h for why each exists). */
#define MW_MIN_FRAMES    25u
#define MW_MIN_CYCLES    3u
#define MW_R_MIN         0.45f
#define MW_DUTY_LO       0.2f
#define MW_DUTY_HI       0.8f
#define MW_CONC_MIN      0.5f
#define MW_COVER_MIN     0.8f
/* Verdict: enough passing slices in a window, held a while after. */
#define MW_WINDOW_MS     3000u
#define MW_MIN_PASSES    4u
#define MW_HOLD_US       3000000ull

void band_default_config(band_config_t *c, uint32_t center_khz, uint32_t fs_hz, uint32_t usable_khz)
{
    memset(c, 0, sizeof(*c));
    c->center_khz = center_khz;
    c->fs_hz = fs_hz;
    c->usable_khz = usable_khz;
    c->on_db = 8.0f;
    c->mw_lo_khz = 2440000u;   /* ovens peak at ~2.45-2.47 GHz; sweep and spread around it */
    c->mw_hi_khz = c->mw_lo_khz + 1000u * BAND_MW_BUCKETS;
}

void band_init(band_watch_t *w, const band_config_t *c)
{
    memset(w, 0, sizeof(*w));
    w->cfg = *c;
    w->bin_khz = (float)c->fs_hz / (float)BAND_BINS / 1000.0f;
    int half = (int)((float)c->usable_khz / 2.0f / w->bin_khz);
    w->half_bins = half > (int)BAND_BINS / 2 - 1 ? (int)BAND_BINS / 2 - 1 : half;
}

/* Signed bin k (-127..127) to its index in natural FFT order. */
static unsigned idx(int k) { return k >= 0 ? (unsigned)k : (unsigned)((int)BAND_BINS + k); }

float band_rf_mhz(const band_watch_t *w, float off_mhz)
{
    return (float)w->cfg.center_khz / 1000.0f + (w->cfg.invert ? -off_mhz : off_mhz);
}

/* The frame's quiet level: the 25th percentile of the usable bins. Robust
 * while up to three quarters of the band is busy, and it needs no learning,
 * so a signal that never stops (a video link) cannot teach itself away. */
static unsigned quiet_code(const band_watch_t *w, const uint8_t *codes)
{
    uint16_t hist[256];
    memset(hist, 0, sizeof(hist));
    unsigned n = 0;
    for (int k = -w->half_bins; k <= w->half_bins; k++) {
        if (abs(k) <= DC_BINS) {
            continue;
        }
        hist[codes[idx(k)]]++;
        n++;
    }
    const unsigned want = n / 4;
    unsigned acc = 0;
    for (unsigned c = 0; c < 256; c++) {
        acc += hist[c];
        if (acc > want) {
            return c;
        }
    }
    return 255;
}

static void mark_wide(float *cover, float lo_khz, float hi_khz)
{
    int j0 = (int)floorf(lo_khz / 1000.0f) + (int)BAND_BUCKETS / 2;
    int j1 = (int)floorf(hi_khz / 1000.0f) + (int)BAND_BUCKETS / 2;
    j0 = j0 < 0 ? 0 : j0;
    j1 = j1 > (int)BAND_BUCKETS - 1 ? (int)BAND_BUCKETS - 1 : j1;
    for (int j = j0; j <= j1; j++) {
        cover[j] = 1.0f;
    }
}

/* Runs of occupied bins across the usable band. Wide ones mark occupancy
 * buckets and their bins (in_wide[k + BAND_BINS / 2]). */
static void find_wide(const band_watch_t *w, const uint8_t *codes, unsigned thr, float *cover, bool *in_wide)
{
    int start = 0, last = 0;
    bool open = false;
    for (int k = -w->half_bins; k <= w->half_bins + RUN_GAP_BINS + 1; k++) {
        const bool occ = k <= w->half_bins && codes[idx(k)] >= thr;
        if (occ) {
            if (!open) {
                start = k;
                open = true;
            }
            last = k;
        } else if (open && k - last > RUN_GAP_BINS) {
            const float lo = ((float)start - 0.5f) * w->bin_khz, hi = ((float)last + 0.5f) * w->bin_khz;
            if (hi - lo >= WIDE_MIN_KHZ) {
                mark_wide(cover, lo, hi);
                for (int j = start; j <= last; j++) {
                    in_wide[j + (int)BAND_BINS / 2] = true;
                }
            }
            open = false;
        }
    }
}

/* Narrow activity in the oven range, as bits of 1 MHz RF buckets. */
static uint64_t oven_range_bits(const band_watch_t *w, const uint8_t *codes, unsigned thr, const bool *in_wide)
{
    uint64_t occ = 0;
    for (int k = -w->half_bins; k <= w->half_bins; k++) {
        if (abs(k) <= DC_BINS || in_wide[k + (int)BAND_BINS / 2] || codes[idx(k)] < thr) {
            continue;
        }
        const float off = (float)k * w->bin_khz;
        const float rf = (float)w->cfg.center_khz + (w->cfg.invert ? -off : off);
        if (rf < (float)w->cfg.mw_lo_khz || rf >= (float)w->cfg.mw_hi_khz) {
            continue;
        }
        const unsigned j = (unsigned)((rf - (float)w->cfg.mw_lo_khz) / 1000.0f);
        occ |= j < BAND_MW_BUCKETS ? 1ull << j : 0u;
    }
    return occ;
}

void band_push_frame(band_watch_t *w, const uint8_t *codes, unsigned bins, uint8_t db_step,
                     uint64_t t_us, uint32_t dur_us)
{
    if (codes == NULL || bins != BAND_BINS || db_step == 0) {
        return;
    }
    w->frames++;
    const unsigned q = quiet_code(w, codes);
    const unsigned add = (unsigned)lrintf(w->cfg.on_db * (float)db_step);
    const unsigned thr = q + add > 255u ? 255u : q + add;

    float cover[BAND_BUCKETS];
    bool in_wide[BAND_BINS];
    memset(cover, 0, sizeof(cover));
    memset(in_wide, 0, sizeof(in_wide));
    find_wide(w, codes, thr, cover, in_wide);
    for (unsigned j = 0; j < BAND_BUCKETS; j++) {
        w->duty[j] += (cover[j] - w->duty[j]) / DUTY_TAU_FRAMES;
    }

    if (w->nfr >= BAND_SLICE_FRAMES) {
        return;
    }
    const uint64_t t_mid = t_us + dur_us / 2u;
    if (w->nfr == 0) {
        w->slice_t0_us = t_mid;
    }
    band_frame_t *f = &w->fr[w->nfr++];
    f->t_us = (uint32_t)(t_mid - w->slice_t0_us);
    f->occ = oven_range_bits(w, codes, thr, in_wide);
}

/* ------------------------------------------------------------ the oven rhythm */

static bool is_on(const band_frame_t *f, uint64_t mask) { return (f->occ & mask) != 0; }

static unsigned popcount64(uint64_t v)
{
    unsigned k = 0;
    for (; v; v &= v - 1) {
        k++;
    }
    return k;
}

/* |first Fourier coefficient at `hz`| / mean |deviation| of the on/off
 * series: 2/pi for a 50 % square wave locked to hz, ~1/(2 sqrt(N d(1-d)))
 * for random bursts. Frame times, not indices: frames vary in length. */
static float rhythm(const band_frame_t *fr, unsigned n, uint64_t mask, float duty, float hz)
{
    float re = 0.0f, im = 0.0f, den = 0.0f;
    const float period_us = 1e6f / hz;
    for (unsigned i = 0; i < n; i++) {
        const float d = (is_on(&fr[i], mask) ? 1.0f : 0.0f) - duty;
        const float ph = 2.0f * PI_F * fmodf((float)fr[i].t_us, period_us) / period_us;
        re += d * cosf(ph);
        im -= d * sinf(ph);
        den += fabsf(d);
    }
    return den > 0.0f ? sqrtf(re * re + im * im) / den : 0.0f;
}

/* Share of whole mains cycles in the slice with at least one on-frame. */
static float cycle_cover(const band_frame_t *fr, unsigned n, uint64_t mask, float hz)
{
    const float period_us = 1e6f / hz;
    const unsigned cycles = (unsigned)((float)fr[n - 1].t_us / period_us);
    if (cycles < MW_MIN_CYCLES || cycles > 64u) {
        return 0.0f;
    }
    uint64_t seen = 0;
    for (unsigned i = 0; i < n; i++) {
        const unsigned c = (unsigned)((float)fr[i].t_us / period_us);
        if (is_on(&fr[i], mask) && c < cycles) {
            seen |= 1ull << c;
        }
    }
    return (float)popcount64(seen) / (float)cycles;
}

/* The BAND_MW_WINDOW buckets holding most of the slice's activity: returns
 * the first bucket, and the activity inside / in the whole range. */
static unsigned busiest_window(const band_watch_t *w, unsigned *inside, unsigned *total)
{
    unsigned hits[BAND_MW_BUCKETS];
    memset(hits, 0, sizeof(hits));
    *total = 0;
    for (unsigned i = 0; i < w->nfr; i++) {
        for (unsigned j = 0; j < BAND_MW_BUCKETS; j++) {
            hits[j] += (unsigned)((w->fr[i].occ >> j) & 1u);
        }
    }
    unsigned best = 0, best_j = 0;
    for (unsigned j = 0; j < BAND_MW_BUCKETS; j++) {
        *total += hits[j];
    }
    for (unsigned j = 0; j + BAND_MW_WINDOW <= BAND_MW_BUCKETS; j++) {
        unsigned sum = 0;
        for (unsigned d = 0; d < BAND_MW_WINDOW; d++) {
            sum += hits[j + d];
        }
        if (sum > best) {
            best = sum;
            best_j = j;
        }
    }
    *inside = best;
    return best_j;
}

/* Is this slice an oven? Fills w->last with the measurements either way. */
static bool slice_is_oven(band_watch_t *w, uint8_t *mains, float *rf_mhz)
{
    band_mw_t *m = &w->last;
    const unsigned n = w->nfr;
    m->frames = n;
    m->r = m->duty = m->stable = m->cover = m->width_mhz = 0.0f;
    if (n < MW_MIN_FRAMES) {
        return false;
    }
    unsigned inside = 0, total = 0;
    const unsigned j0 = busiest_window(w, &inside, &total);
    const uint64_t mask = ((1ull << BAND_MW_WINDOW) - 1u) << j0;
    unsigned on = 0, occupied = 0;
    for (unsigned i = 0; i < n; i++) {
        if (is_on(&w->fr[i], mask)) {
            on++;
            occupied += popcount64(w->fr[i].occ & mask);
        }
    }
    m->duty = (float)on / (float)n;
    if (on < 3 || total == 0) {
        return false;
    }
    const float r50 = rhythm(w->fr, n, mask, m->duty, 50.0f), r60 = rhythm(w->fr, n, mask, m->duty, 60.0f);
    const float hz = r60 > r50 ? 60.0f : 50.0f;
    m->r = r60 > r50 ? r60 : r50;
    m->stable = (float)inside / (float)total;
    m->width_mhz = (float)occupied / (float)on;
    m->cover = cycle_cover(w->fr, n, mask, hz);
    *mains = (uint8_t)hz;
    *rf_mhz = (float)w->cfg.mw_lo_khz / 1000.0f + (float)j0 + (float)BAND_MW_WINDOW / 2.0f;
    return m->duty >= MW_DUTY_LO && m->duty <= MW_DUTY_HI && m->r >= MW_R_MIN && m->stable >= MW_CONC_MIN &&
           m->cover >= MW_COVER_MIN;
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

void band_slice_done(band_watch_t *w, uint64_t now_us)
{
    uint8_t mains = 0;
    float rf = 0.0f;
    const bool analysed = w->nfr >= MW_MIN_FRAMES;
    const bool pass = slice_is_oven(w, &mains, &rf);
    w->nfr = 0;
    if (!analysed) {
        return;
    }
    const uint32_t t_ms = (uint32_t)(now_us / 1000u);
    w->hist[w->head] = (band_slice_t){ t_ms, pass, mains, w->last.r, rf };
    w->head = (w->head + 1) % BAND_MW_HISTORY;
    w->count += w->count < BAND_MW_HISTORY;

    unsigned passes = 0, slices = 0;
    uint8_t mains_last = 0;
    float where[BAND_MW_HISTORY];
    for (unsigned k = 0; k < w->count; k++) {
        const band_slice_t *s = &w->hist[(w->head + BAND_MW_HISTORY - 1 - k) % BAND_MW_HISTORY];
        if (t_ms - s->t_ms > MW_WINDOW_MS) {
            break;
        }
        slices++;
        if (s->pass) {
            mains_last = passes == 0 ? s->mains_hz : mains_last;   /* newest passing slice */
            where[passes++] = s->rf_mhz;
        }
    }
    w->last.passes = passes;
    w->last.slices = slices;
    if (passes >= MW_MIN_PASSES && 2u * passes >= slices) {
        sort_f(where, passes);
        w->mw_rf_mhz = where[passes / 2];
        w->mw_mains = mains_last;
        w->mw_until_us = now_us + MW_HOLD_US;
    }
}

void band_microwave(const band_watch_t *w, uint64_t now_us, band_mw_t *out)
{
    *out = w->last;
    out->active = now_us < w->mw_until_us;
    out->rf_mhz = w->mw_rf_mhz;
    out->mains_hz = w->mw_mains;
}

float band_wide_duty(const band_watch_t *w, float off_mhz, float half_mhz)
{
    int j0 = (int)floorf(off_mhz - half_mhz) + (int)BAND_BUCKETS / 2;
    int j1 = (int)floorf(off_mhz + half_mhz) + (int)BAND_BUCKETS / 2;
    j0 = j0 < 0 ? 0 : j0;
    j1 = j1 > (int)BAND_BUCKETS - 1 ? (int)BAND_BUCKETS - 1 : j1;
    float best = 0.0f;
    for (int j = j0; j <= j1; j++) {
        best = w->duty[j] > best ? w->duty[j] : best;
    }
    return best;
}
