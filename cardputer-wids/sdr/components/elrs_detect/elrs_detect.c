/* elrs_detect.c - see elrs_detect.h for what is detected and why. */
#include "elrs_detect.h"

#include <math.h>
#include <string.h>

/* A hit on the same channel within this gap belongs to the same hop. Long
 * enough to bridge the packet spacing inside one dwell (up to 20 ms at the
 * 50 Hz rate), far shorter than the time before ELRS returns to a channel
 * (one 80-hop block, >= 160 ms). */
#define VISIT_GAP_US 25000u

/* Time constant for the "bursts right now" rate. The 10 s window answers
 * "what has the band looked like"; this answers "is it still going", so a
 * verdict can be released a few seconds after the link stops. */
#define RECENT_TAU_S 1.0f

/* ELRS-LIKELY needs ELRS-grid bursts NOW, not just in the 10 s statistics;
 * otherwise the verdict outlives the link by the whole window. Even 50 Hz
 * ELRS gives ~40 on-grid bursts/s over the visible channels. */
#define MIN_ON_NOW_PER_S 3.0f

/* Bluetooth sends hundreds of bursts per second. If even a few percent of
 * their centres were misjudged by 0.2+ MHz they would land on the ELRS grid,
 * so on-grid hits must also be a real share of the integer-MHz traffic.
 * Simulator (sim/): 50 Hz ELRS beside full-band Bluetooth audio scores 0.26;
 * Bluetooth alone scores 0.000 once clipped runs are ignored (0.165 before).
 * A starting value for M2, not a truth. */
#define MIN_ON_VS_INTEGER 0.08f

/* Upper edges of the dwell buckets (ms) and the value each bucket reports.
 * Centres are the discrete dwell times the ELRS rate table produces. */
static const float k_dwell_edge_ms[ELRS_DWELL_BUCKETS - 1] = { 3, 6, 10, 14, 21, 33 };
static const float k_dwell_centre_ms[ELRS_DWELL_BUCKETS] = { 2, 4, 8, 12, 16, 27, 40 };

const char *elrs_state_name(elrs_state_t s)
{
    switch (s) {
    case ELRS_LIKELY: return "ELRS-LIKELY";
    case ELRS_HOPPER: return "HOPPER";
    default:          return "QUIET";
    }
}

void elrs_default_config(elrs_config_t *cfg, uint32_t center_khz, uint32_t sample_rate_hz)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->center_khz = center_khz;
    cfg->sample_rate_hz = sample_rate_hz;
    cfg->usable_khz = 66000;      /* S3 analogue bandwidth tops out at 69 MHz */
    cfg->invert = false;
    cfg->threshold_db = 10;
    cfg->grid_tol_khz = 200;      /* x.4 grid vs Bluetooth's integer MHz: 400 kHz apart */
    cfg->int_tol_khz = 150;       /* leaves a 50 kHz gap to the ELRS window */
    cfg->narrow_min_khz = 250;
    cfg->narrow_max_khz = 1600;
    cfg->wide_min_khz = 4000;
    cfg->window_s = 10.0f;
    cfg->hold_s = 10.0f;
}

void elrs_init(elrs_detector_t *d, const elrs_config_t *cfg)
{
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    d->state = ELRS_QUIET;
    d->pending = ELRS_QUIET;
}

/* ------------------------------------------------------------ geometry */

static float bin_khz(const elrs_detector_t *d)
{
    return (float)d->cfg.sample_rate_hz / (float)d->bins / 1000.0f;
}

/* Signed offset from the LO, in bins, of natural-order index i. */
static int signed_bin(unsigned i, unsigned bins)
{
    return i < bins / 2 ? (int)i : (int)i - (int)bins;
}

static float offset_khz(const elrs_detector_t *d, float k)
{
    float off = k * bin_khz(d);
    return d->cfg.invert ? -off : off;
}

/* Bins next to the LO carry the DC-correction residue; the edges are outside
 * the analogue filter. Neither is evidence of anything. */
static bool bin_usable(const elrs_detector_t *d, int k)
{
    if (k >= -1 && k <= 1) {
        return false;
    }
    return fabsf(offset_khz(d, (float)k)) <= (float)d->cfg.usable_khz / 2.0f;
}

static void compute_visible(elrs_detector_t *d)
{
    const float half = (float)d->cfg.usable_khz / 2.0f - (float)d->cfg.narrow_max_khz / 2.0f;
    const float dc_guard = 2.0f * bin_khz(d);
    d->n_visible = 0;
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        const float rel = (float)(ELRS_BASE_KHZ + ch * ELRS_STEP_KHZ) - (float)d->cfg.center_khz;
        d->visible[ch] = fabsf(rel) <= half && fabsf(rel) > dc_guard;
        d->n_visible += d->visible[ch];
    }
}

/* ------------------------------------------------------------ per frame */

static void decay_all(elrs_detector_t *d, float f, float f_recent)
{
    d->recent_narrow *= f_recent;
    d->recent_on *= f_recent;
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        d->visits[ch] *= f;
        d->spread[ch] *= f;
    }
    for (unsigned b = 0; b < ELRS_DWELL_BUCKETS; b++) {
        d->dwell_hist[b] *= f;
    }
    d->on_bursts *= f;
    d->int_bursts *= f;
    d->off_bursts *= f;
    d->wide_runs *= f;
}

static void close_visit(elrs_detector_t *d, unsigned ch)
{
    const float ms = (float)(d->visit_last_us[ch] - d->visit_start_us[ch]) / 1000.0f;
    unsigned b = 0;
    while (b < ELRS_DWELL_BUCKETS - 1 && ms >= k_dwell_edge_ms[b]) {
        b++;
    }
    d->dwell_hist[b] += 1.0f;
    d->in_visit[ch] = false;
}

/* Classify one run of above-floor bins [k0, k1] (signed bin numbers).
 * `clipped`: the run touches a bin we ignore (band edge, DC guard), so part of
 * it is unseen and its centre is biased. A clipped wide run is still wide; a
 * clipped narrow one is not graded at all. Without this, Bluetooth bursts cut
 * off at 2441-2442 MHz and at the band edges landed on the ELRS grid (found
 * with the simulator, BUGLOG.md). */
static void classify_run(elrs_detector_t *d, const uint8_t *codes, int k0, int k1, bool clipped,
                         bool hit_now[ELRS_CHANNELS])
{
    const float width = (float)(k1 - k0 + 1) * bin_khz(d);
    if (width >= (float)d->cfg.wide_min_khz) {
        d->wide_runs += 1.0f;
        return;
    }
    if (clipped || width < (float)d->cfg.narrow_min_khz || width > (float)d->cfg.narrow_max_khz) {
        return;
    }

    /* Power-weighted centre gives sub-bin accuracy, which the grid test needs:
     * at 312 kHz bins the x.4 vs integer-MHz distinction is about one bin. */
    float wsum = 0.0f, ksum = 0.0f;
    for (int k = k0; k <= k1; k++) {
        const unsigned i = (unsigned)((k + (int)d->bins) % (int)d->bins);
        float w = (float)codes[i] * 16.0f - (float)d->floor_q4[i];
        if (w < 1.0f) {
            w = 1.0f;
        }
        wsum += w;
        ksum += w * (float)k;
    }
    const float f_khz = (float)d->cfg.center_khz + offset_khz(d, ksum / wsum);
    const float rel = f_khz - (float)ELRS_BASE_KHZ;
    const long ch = lroundf(rel / (float)ELRS_STEP_KHZ);
    const float off = rel - (float)ch * (float)ELRS_STEP_KHZ;

    d->recent_narrow += 1.0f;
    if (ch >= 0 && ch < (long)ELRS_CHANNELS) {
        d->spread[ch] += 1.0f;
    }
    /* Distance to the nearest integer MHz: where Bluetooth, BLE and 802.15.4
     * channels are centred. */
    const float int_frac = fmodf(f_khz, 1000.0f);
    const float int_dist = int_frac < 500.0f ? int_frac : 1000.0f - int_frac;

    if (ch >= 0 && ch < (long)ELRS_CHANNELS && fabsf(off) <= (float)d->cfg.grid_tol_khz) {
        d->on_bursts += 1.0f;
        d->recent_on += 1.0f;
        hit_now[ch] = true;
    } else if (int_dist <= (float)d->cfg.int_tol_khz) {
        d->int_bursts += 1.0f;
    } else {
        d->off_bursts += 1.0f;
    }
}

void elrs_push_frame(elrs_detector_t *d, const uint8_t *codes, unsigned bins,
                     uint8_t db_step, uint64_t t_us, uint32_t dur_us)
{
    if (codes == NULL || bins < 16 || bins > ELRS_MAX_BINS || (bins & (bins - 1)) != 0 ||
        db_step == 0) {
        return;
    }
    if (bins != d->bins) {
        d->bins = bins;
        d->floor_ready = false;
        compute_visible(d);
    }
    d->db_step = db_step;

    if (d->frame_no > 0 && t_us > d->last_t_us) {
        const float dt_s = (float)(t_us - d->last_t_us) / 1e6f;
        decay_all(d, expf(-dt_s / d->cfg.window_s), expf(-dt_s / RECENT_TAU_S));
    }
    d->last_t_us = t_us;

    if (!d->floor_ready) {
        for (unsigned i = 0; i < bins; i++) {
            d->floor_q4[i] = (uint16_t)(codes[i] * 16u);
        }
        d->floor_ready = true;
    }

    /* Pass 1: find runs against the floor as it was BEFORE this frame, so a
     * burst cannot raise its own threshold. Walk in frequency order. */
    const uint16_t thr_q4 = (uint16_t)(d->cfg.threshold_db * db_step * 16u);
    bool hit_now[ELRS_CHANNELS] = { false };
    const int half = (int)bins / 2;
    int run_start = 0;
    bool in_run = false, start_clipped = false, prev_usable = false;
    uint8_t peak = 0;
    for (int k = -half; k <= half; k++) {
        const bool usable = k < half && bin_usable(d, k);
        bool above = false;
        if (usable) {
            const unsigned i = (unsigned)((k + (int)bins) % (int)bins);
            above = (uint32_t)codes[i] * 16u > (uint32_t)d->floor_q4[i] + thr_q4;
            if (codes[i] > peak) {
                peak = codes[i];
            }
        }
        if (above && !in_run) {
            run_start = k;
            in_run = true;
            start_clipped = !prev_usable;
        } else if (!above && in_run) {
            /* Ended on an ignored bin rather than on a quiet one: clipped. */
            classify_run(d, codes, run_start, k - 1, start_clipped || !usable, hit_now);
            in_run = false;
        }
        prev_usable = usable;
    }
    d->last_peak_code = peak;

    /* Pass 2: floor tracker, a cheap low-percentile follower. It drops quickly
     * towards quiet bins and creeps up by 1/16 code per frame, so steady
     * background (a busy Wi-Fi channel) is absorbed while ELRS, which visits a
     * channel once per 80 hops, never is. */
    for (unsigned i = 0; i < bins; i++) {
        const uint16_t c = (uint16_t)(codes[i] * 16u);
        if (c < d->floor_q4[i]) {
            d->floor_q4[i] -= (uint16_t)((d->floor_q4[i] - c + 7u) / 8u);
        } else if (c > d->floor_q4[i]) {
            d->floor_q4[i]++;
        }
    }

    /* Visits: one per hop, however many packets land in it. */
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        if (hit_now[ch]) {
            if (!d->in_visit[ch] || t_us - d->visit_last_us[ch] > VISIT_GAP_US) {
                if (d->in_visit[ch]) {
                    close_visit(d, ch);
                }
                d->in_visit[ch] = true;
                d->visit_start_us[ch] = t_us;
                d->visits[ch] += 1.0f;
            }
            d->visit_last_us[ch] = t_us + dur_us;
            d->last_hit_frame[ch] = d->frame_no;
            d->last_seen_us = t_us;
        } else if (d->in_visit[ch] && t_us > d->visit_last_us[ch] + VISIT_GAP_US) {
            close_visit(d, ch);
        }
    }
    d->frame_no++;
}

/* ------------------------------------------------------------ verdict */

static float clamp01(float x) { return x < 0.0f ? 0.0f : x > 1.0f ? 1.0f : x; }

bool elrs_evaluate(elrs_detector_t *d, uint64_t now_us, elrs_status_t *out)
{
    elrs_status_t s;
    memset(&s, 0, sizeof(s));

    /* Coverage and flatness over the channels we can actually see. Zeros
     * count: a real ELRS link fills every visible channel within a block. */
    float sum = 0.0f, sumsq = 0.0f, vmax = 0.0f;
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        if (!d->visible[ch]) {
            continue;
        }
        const float v = d->visits[ch];
        sum += v;
        sumsq += v * v;
        if (v > vmax) {
            vmax = v;
        }
        if (v >= 0.5f) {
            s.channels_seen++;
        }
    }
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        s.distinct_narrow += d->spread[ch] >= 0.5f;
    }
    s.channels_visible = d->n_visible;
    if (d->n_visible > 0 && sum > 0.0f) {
        const float mean = sum / (float)d->n_visible;
        const float var = sumsq / (float)d->n_visible - mean * mean;
        s.uniformity_cv = sqrtf(var > 0.0f ? var : 0.0f) / mean;
    } else {
        s.uniformity_cv = 99.0f;
    }
    for (unsigned ch = 0; ch < ELRS_CHANNELS; ch++) {
        s.activity[ch] = vmax > 0.0f ? (uint8_t)(255.0f * d->visits[ch] / vmax) : 0;
    }

    /* Integer-MHz bursts are neither for nor against ELRS: they are another
     * system, identified as such. */
    const float graded = d->on_bursts + d->off_bursts;
    const float narrow = graded + d->int_bursts;
    s.on_grid_ratio = graded >= 1.0f ? d->on_bursts / graded : 0.0f;
    s.int_grid_ratio = narrow >= 1.0f ? d->int_bursts / narrow : 0.0f;
    s.on_now_per_s = d->recent_on / RECENT_TAU_S;
    s.narrow_per_s = d->recent_narrow / RECENT_TAU_S;
    s.wide_per_s = d->wide_runs / d->cfg.window_s;

    /* Weighted median of the dwell histogram. */
    float total = 0.0f;
    for (unsigned b = 0; b < ELRS_DWELL_BUCKETS; b++) {
        total += d->dwell_hist[b];
    }
    if (total >= 1.0f) {
        float acc = 0.0f;
        for (unsigned b = 0; b < ELRS_DWELL_BUCKETS; b++) {
            acc += d->dwell_hist[b];
            if (acc >= total / 2.0f) {
                s.dwell_ms = k_dwell_centre_ms[b];
                break;
            }
        }
    }

    /* Candidate verdict. Thresholds are starting points, to be tuned on real
     * recordings (milestone M2), not truths. */
    const unsigned need_seen = d->n_visible * 2u / 5u > 8u ? d->n_visible * 2u / 5u : 8u;
    /* Hopping = narrow bursts arriving steadily AND spread over many
     * frequencies. A fixed narrowband carrier fails the spread test. */
    const bool hopper = s.narrow_per_s >= 3.0f && s.distinct_narrow >= 8u;
    const bool likely = hopper && s.on_now_per_s >= MIN_ON_NOW_PER_S && s.on_grid_ratio >= 0.7f &&
                        d->on_bursts >= MIN_ON_VS_INTEGER * d->int_bursts &&
                        s.channels_seen >= need_seen && s.uniformity_cv <= 1.0f;
    const elrs_state_t cand = likely ? ELRS_LIKELY : hopper ? ELRS_HOPPER : ELRS_QUIET;

    if (hopper && d->n_visible > 0) {
        s.score = clamp01((float)s.channels_seen / (0.6f * (float)d->n_visible)) *
                  clamp01((s.on_grid_ratio - 0.4f) / 0.5f) *
                  clamp01(1.5f - s.uniformity_cv);
    }

    /* Hysteresis: two agreeing evaluations to go up, a hold time to come down,
     * so the screen does not flicker and one stray burst never alarms. */
    bool changed = false;
    const uint64_t hold_us = (uint64_t)(d->cfg.hold_s * 1e6f);
    if (cand >= d->state) {
        d->support_until_us = now_us + hold_us;
    }
    if (cand > d->state) {
        if (d->pending == cand) {
            d->pending_count++;
        } else {
            d->pending = cand;
            d->pending_count = 1;
        }
        if (d->pending_count >= 2) {
            d->state = cand;
            d->pending_count = 0;
            changed = true;
        }
    } else {
        d->pending_count = 0;
        if (cand < d->state && now_us > d->support_until_us) {
            d->state = cand;
            changed = true;
        }
    }

    if (d->state == ELRS_LIKELY) {
        d->last_likely_us = now_us;
    }
    s.state = d->state;
    s.last_seen_us = d->last_seen_us;
    s.last_likely_us = d->last_likely_us;
    s.frames = d->frame_no;
    s.peak_db = (int16_t)(d->db_step ? d->last_peak_code / d->db_step : 0);
    if (d->bins > 0 && d->db_step > 0) {
        uint32_t fsum = 0, fn = 0;
        for (unsigned i = 0; i < d->bins; i++) {
            if (bin_usable(d, signed_bin(i, d->bins))) {
                fsum += d->floor_q4[i];
                fn++;
            }
        }
        s.floor_db = (int16_t)(fn ? fsum / fn / 16u / d->db_step : 0);
    }
    if (out != NULL) {
        *out = s;
    }
    return changed;
}
