/* targets.c - see targets.h. */
#include "targets.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Wide-activity runs: contiguous 1 MHz buckets (band_watch.h) at least this
 * busy. A run is one signal: OFDM verdicts of the slots inside it name it,
 * its duty-weighted centre places it. */
#define RUN_DUTY_MIN       0.05f
/* "Wide signal" needs steady wide activity and enough scans to have looked. */
#define WIDE_DUTY_MIN      0.2f
#define WIDE_MIN_SCANS     6u
/* Any slot of the run with a mean this high (any kind) explains it too: a
 * bursty access point's verdict lapses now and then, and the run is then
 * still Wi-Fi, not an unknown wide signal (simulator). */
#define WIDE_EXPLAINED_AVG 3.5f
#define NEAR_MW_MHZ        10.0f
#define NEAR_LORA_MHZ      3.0f
#define MAX_RUNS           8u
/* A DJI-like slot with no wide activity of its own (a link under the
 * spectrum threshold) needs a clearly higher mean than the slot threshold:
 * nothing in the spectrum backs it up. */
#define LONE_SLOT_MARGIN   1.5f

const char *target_kind_name(target_kind_t k)
{
    switch (k) {
    case TGT_LORA_FIXED: return "lora_fixed";
    case TGT_DJI: return "dji_like";
    case TGT_HOPPER: return "hopper";
    case TGT_WIDE: return "wide";
    case TGT_MICROWAVE: return "microwave";
    case TGT_WIFI: return "wifi";
    default: return "?";
    }
}

typedef struct {
    ui_target_t *out;
    unsigned n, max;
} list_t;

static ui_target_t *add(list_t *l, target_kind_t kind, ui_level_t level, float mhz)
{
    if (l->n >= l->max) {
        return NULL;
    }
    ui_target_t *t = &l->out[l->n++];
    memset(t, 0, sizeof(*t));
    t->kind = (uint8_t)kind;
    t->level = (uint8_t)level;
    t->mhz = mhz;
    return t;
}

typedef struct {
    float lo_mhz, hi_mhz;   /* raw offsets covered */
    float centre_mhz;       /* duty-weighted centre, raw offset */
    float peak_duty;
} run_t;

/* Contiguous buckets with wide duty >= min, as runs. */
static unsigned wide_runs(const band_watch_t *b, float min, run_t *out, unsigned max)
{
    unsigned n = 0;
    for (unsigned j = 0; j < BAND_BUCKETS && n < max;) {
        if (b->duty[j] < min) {
            j++;
            continue;
        }
        float wsum = 0.0f, csum = 0.0f, peak = 0.0f;
        const unsigned j0 = j;
        for (; j < BAND_BUCKETS && b->duty[j] >= min; j++) {
            const float centre = (float)j - (float)BAND_BUCKETS / 2.0f + 0.5f;
            wsum += b->duty[j];
            csum += b->duty[j] * centre;
            peak = b->duty[j] > peak ? b->duty[j] : peak;
        }
        out[n++] = (run_t){ (float)j0 - (float)BAND_BUCKETS / 2.0f, (float)j - (float)BAND_BUCKETS / 2.0f,
                            csum / wsum, peak };
    }
    return n;
}

static bool slot_in_run(unsigned s, const run_t *r)
{
    const float off = ofdm_slot_off_mhz(s);
    return off >= r->lo_mhz - 1.0f && off <= r->hi_mhz + 1.0f;
}

static bool kind_on(const target_inputs_t *in, unsigned s, ofdm_kind_t k) { return ofdm_watch_on(in->ofdm, s, k, in->now_us); }

static bool lte_on(const target_inputs_t *in, unsigned s)
{
    return kind_on(in, s, OFDM_LTE15) || kind_on(in, s, OFDM_LTE30);
}

static bool any_on(const target_inputs_t *in, unsigned s)
{
    return lte_on(in, s) || kind_on(in, s, OFDM_WIFI);
}

static float lte_avg(const target_inputs_t *in, unsigned s)
{
    const ofdm_slot_t *sl = &in->ofdm->slot[s];
    return sl->avg[OFDM_LTE15] > sl->avg[OFDM_LTE30] ? sl->avg[OFDM_LTE15] : sl->avg[OFDM_LTE30];
}

static float slot_rf(const target_inputs_t *in, unsigned s) { return band_rf_mhz(in->band, ofdm_slot_off_mhz(s)); }

static float lora_rf(const target_inputs_t *in) { return band_rf_mhz(in->band, in->chirp->freq_off_mhz); }

static void dji_text(const target_inputs_t *in, ui_target_t *t, unsigned slot)
{
    const ofdm_slot_t *sl = &in->ofdm->slot[slot];
    snprintf(t->text, sizeof(t->text), "DJI-like OFDM link %.0f MHz (%s)", (double)t->mhz,
             sl->avg[OFDM_LTE15] >= sl->avg[OFDM_LTE30] ? "15k" : "30k");
}

static void add_lora_fixed(const target_inputs_t *in, list_t *l)
{
    if (!in->chirp->active || !in->chirp->fixed_channel) {
        return;
    }
    ui_target_t *t = add(l, TGT_LORA_FIXED, UI_LVL_WARN, lora_rf(in));
    if (t) {
        snprintf(t->text, sizeof(t->text), "LoRa 1 channel %.0f MHz, not ELRS", (double)t->mhz);
    }
}

/* One DJI-like target per wide run with an LTE-like slot in it. With none,
 * the strongest LTE-like slot alone: a link under the spectrum threshold.
 * A slot outside every run is otherwise a copy that leaked from a strong
 * signal elsewhere (the CIC filter's sidelobes, ofdm_detect.c). */
static void add_dji(const target_inputs_t *in, list_t *l, const run_t *runs, unsigned nruns)
{
    unsigned found = 0;
    for (unsigned r = 0; r < nruns; r++) {
        int best = -1;
        for (unsigned s = 0; s < OFDM_SLOTS; s++) {
            if (lte_on(in, s) && slot_in_run(s, &runs[r]) && (best < 0 || lte_avg(in, s) > lte_avg(in, (unsigned)best))) {
                best = (int)s;
            }
        }
        ui_target_t *t = best >= 0 ? add(l, TGT_DJI, UI_LVL_WARN, band_rf_mhz(in->band, runs[r].centre_mhz)) : NULL;
        if (t) {
            dji_text(in, t, (unsigned)best);
            found++;
        }
    }
    if (found) {
        return;
    }
    int best = -1;
    const float lone = ofdm_watch_threshold(OFDM_LTE15) + LONE_SLOT_MARGIN;
    for (unsigned s = 0; s < OFDM_SLOTS; s++) {
        if (lte_on(in, s) && lte_avg(in, s) >= lone && (best < 0 || lte_avg(in, s) > lte_avg(in, (unsigned)best))) {
            best = (int)s;
        }
    }
    ui_target_t *t = best >= 0 ? add(l, TGT_DJI, UI_LVL_WARN, slot_rf(in, (unsigned)best)) : NULL;
    if (t) {
        dji_text(in, t, (unsigned)best);
    }
}

static void add_hopper(const target_inputs_t *in, list_t *l)
{
    if (in->elrs_shown || in->elrs->state != ELRS_HOPPER) {
        return;   /* while ELRS is shown, the grid's HOPPER is most likely that link */
    }
    ui_target_t *t = add(l, TGT_HOPPER, UI_LVL_NOTE, 0.0f);
    if (t) {
        /* Bluetooth (and BLE) sit on whole MHz, ELRS on x.4 (elrs_detect.h). */
        snprintf(t->text, sizeof(t->text), "%s",
                 in->elrs->int_grid_ratio >= 0.6f ? "Bluetooth-like hopping" : "other hopper (not ELRS grid)");
    }
}

/* Busy wide runs that no OFDM verdict explains, looked at enough, and not
 * the oven. */
static void add_wide(const target_inputs_t *in, list_t *l, const run_t *runs, unsigned nruns)
{
    for (unsigned r = 0; r < nruns; r++) {
        if (runs[r].peak_duty < WIDE_DUTY_MIN) {
            continue;
        }
        bool explained = false, looked = false;
        for (unsigned s = 0; s < OFDM_SLOTS; s++) {
            if (slot_in_run(s, &runs[r])) {
                explained |= any_on(in, s);
                for (unsigned k = 0; k < OFDM_KINDS; k++) {
                    explained |= in->ofdm->slot[s].avg[k] >= WIDE_EXPLAINED_AVG;
                }
                looked |= in->ofdm->slot[s].scans >= WIDE_MIN_SCANS;
            }
        }
        const float rf = band_rf_mhz(in->band, runs[r].centre_mhz);
        if (explained || !looked || (in->mw->active && fabsf(rf - in->mw->rf_mhz) < NEAR_MW_MHZ)) {
            continue;
        }
        ui_target_t *t = add(l, TGT_WIDE, UI_LVL_NOTE, rf);
        if (t) {
            snprintf(t->text, sizeof(t->text), "wide signal %.0f MHz (not Wi-Fi/DJI)", (double)rf);
        }
    }
}

static void add_microwave(const target_inputs_t *in, list_t *l)
{
    if (!in->mw->active) {
        return;
    }
    /* A non-hopping transmitter keyed at the mains rate passes the oven
     * tests too (band_watch.h); if that is fixed LoRa, it is not an oven. */
    if (in->chirp->active && in->chirp->fixed_channel && fabsf(lora_rf(in) - in->mw->rf_mhz) < NEAR_LORA_MHZ) {
        return;
    }
    ui_target_t *t = add(l, TGT_MICROWAVE, UI_LVL_INFO, in->mw->rf_mhz);
    if (t) {
        snprintf(t->text, sizeof(t->text), "microwave oven %.0f MHz (%u Hz)", (double)t->mhz, in->mw->mains_hz);
    }
}

/* One Wi-Fi line per wide run with a Wi-Fi slot in it, named by the run's
 * centre: channel = (MHz - 2407) / 5. */
static void add_wifi(const target_inputs_t *in, list_t *l, const run_t *runs, unsigned nruns)
{
    for (unsigned r = 0; r < nruns; r++) {
        bool on = false;
        for (unsigned s = 0; s < OFDM_SLOTS; s++) {
            on |= kind_on(in, s, OFDM_WIFI) && slot_in_run(s, &runs[r]);
        }
        if (!on) {
            continue;
        }
        const float mhz = band_rf_mhz(in->band, runs[r].centre_mhz);
        long ch = lroundf((mhz - 2407.0f) / 5.0f);
        ch = ch < 1 ? 1 : ch > 13 ? 13 : ch;
        ui_target_t *t = add(l, TGT_WIFI, UI_LVL_DIM, mhz);
        if (t) {
            snprintf(t->text, sizeof(t->text), "Wi-Fi ch %ld (%.0f MHz)", ch, (double)mhz);
        }
    }
}

unsigned targets_build(const target_inputs_t *in, ui_target_t *out, unsigned max)
{
    list_t l = { out, 0, max };
    run_t runs[MAX_RUNS];
    const unsigned nruns = wide_runs(in->band, RUN_DUTY_MIN, runs, MAX_RUNS);
    add_lora_fixed(in, &l);
    add_dji(in, &l, runs, nruns);
    add_hopper(in, &l);
    add_wide(in, &l, runs, nruns);
    add_microwave(in, &l);
    add_wifi(in, &l, runs, nruns);
    return l.n;
}
