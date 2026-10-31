/* chirp_watch.c - see chirp_watch.h. */
#include "chirp_watch.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "lora_detect.h"
#include "sdr_engine.h"
#include "wids_log.h"

#define CAPTURE_UNITS   3u
/* Snapshot rate: after each step, wait COST_FACTOR times what the step cost
 * (capture + scan), so chirp work takes at most 1/(COST_FACTOR+1) of the
 * time and the rest stays with the spectrum slices. A fast scan therefore
 * runs every pass, a slow one every few: more snapshots, more evidence,
 * without a fixed guess of the S3's scan time. */
#define COST_FACTOR     2u
#define WINDOW_US       5000000ull  /* evidence window */
#define MIN_HITS        2u          /* full-pass snapshots in the window for a verdict */
/* Weak snapshots (lora_detect.h) count only when this many in the window,
 * full or weak, agree on SF and slope. A real link keeps one SF and one
 * slope; chance weak hits (~1 in 300 host snapshots with interferers) land
 * on any of 8 SF/slope pairs, so three that agree within 5 s is rare. */
#define MIN_WEAK_AGREE  3u
#define HOLD_US         10000000ull /* verdict held after the last confirming hit */
/* ELRS hops over 80 channels, so its hits in 5 s land on many of them. Hits
 * that all sit on one channel mean LoRa that does not hop (a mesh node, an
 * IoT sensor, maybe ELRS binding): not an ELRS control link. The test only
 * applies once there are enough hits to expect hopping to show: at the edge
 * of range, with 2-4 hits, the verdict is left alone (no loss of range). */
#define FIXED_MIN_HITS  5u
/* Snapshots are at least one 100 ms slice apart, so 5 s holds at most ~50.
 * If it ever held more, the oldest would drop out of the count: fewer hits
 * counted, never more. */
#define HISTORY         56u
#define DUMP_SAMPLES_PER_LINE 48u   /* 192 bytes -> 384 hex chars per log line */
#define DUMP_RAW_WORDS  384u
#define SNR_SMOOTH      0.3f        /* weight of each new hit in the smoothed SNR */

typedef struct {
    uint32_t t_ms;                  /* ms since boot: wraps after 49 days */
    bool     hit;                   /* passed the full tests */
    bool     weak;                  /* passed at least the weak tier */
    uint8_t  sf;                    /* when weak */
    int8_t   slope;
    int8_t   chan;                  /* when weak: floor(MHz offset from the LO), x.4 and x.6 alike */
} snap_t;

static struct {
    unsigned rate_code;
    uint64_t next_us;                /* earliest next snapshot (COST_FACTOR) */
    snap_t   hist[HISTORY];
    unsigned head, count;
    uint64_t active_until_us, last_hit_us;
    uint8_t  last_sf;
    lora_floor_t floor;              /* long-term noise floor (lora_detect.h) */
    lora_result_t last;              /* most recent scan */
    lora_result_t last_hit;          /* most recent positive scan */
    uint32_t snaps_total, hits_total, weak_total, cap_errors, scan_us_max, scan_us_last, step_us_last;
    bool     by_weak;                /* the verdict rests on agreeing weak hits only */
    unsigned work_mod16;             /* scratch address mod 16, logged: alignment (BUGLOG) */
    float    snr_smooth, snr_ref;         /* smoothed SNR; its value when the detection began */
    bool     have_ref;
    bool     dump_requested;
} s;

const char *chirp_elrs_rate(uint8_t sf)
{
    switch (sf) {
    case 5: return "500/333 Hz";
    case 6: return "250 Hz";
    case 7: return "150/100 Hz";
    case 8: return "50 Hz";
    default: return "?";
    }
}

void chirp_watch_init(unsigned rate_code)
{
    memset(&s, 0, sizeof(s));
    s.rate_code = rate_code;
}

void chirp_watch_request_dump(void) { s.dump_requested = true; }
void chirp_watch_reset_floor(void) { memset(&s.floor, 0, sizeof(s.floor)); }

/* ------------------------------------------------------------ dump */

static void dump_hex_lines(const char *ev, const uint8_t *bytes, size_t n, const char *extra)
{
    char hex[2 * DUMP_SAMPLES_PER_LINE * 4 + 1];
    const size_t per_line = DUMP_SAMPLES_PER_LINE * 4;
    const unsigned parts = (unsigned)((n + per_line - 1) / per_line);
    for (unsigned p = 0; p < parts; p++) {
        const size_t len = n - p * per_line < per_line ? n - p * per_line : per_line;
        wids_hex_encode(bytes + p * per_line, len, hex, sizeof(hex));
        wids_log_event(ev, "\"part\":%u,\"of\":%u,%s,\"hex\":\"%s\"", p, parts, extra, hex);
    }
}

/* Baseband of the reported candidate as int16 I,Q little-endian, then the
 * first raw ring words, so the dechirp can be redone offline on real data. */
static void dump(const lora_snapshot_t *snap, float *work, const lora_result_t *res)
{
    const unsigned m = lora_baseband(snap, res->freq_hz, work, LORA_MAX_BASEBAND);
    uint8_t *bytes = (uint8_t *)(work + 2 * LORA_MAX_BASEBAND);   /* after z in the scratch */
    for (unsigned k = 0; k < 2 * m; k++) {
        float v = roundf(work[k]);
        v = v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : v;
        const int16_t x = (int16_t)v;
        bytes[2 * k] = (uint8_t)(x & 0xff);
        bytes[2 * k + 1] = (uint8_t)((uint16_t)x >> 8);
    }
    char extra[128];
    snprintf(extra, sizeof(extra),
             "\"freq_hz\":%.0f,\"rate_hz\":%u,\"samples\":%u,\"format\":\"int16le_iq\",\"sf\":%u,\"detected\":%s",
             (double)res->freq_hz, LORA_DECIM_RATE, m, res->sf, res->detected ? "true" : "false");
    dump_hex_lines("bb", bytes, (size_t)m * 4, extra);

    uint32_t n = snap->seg[0].n < DUMP_RAW_WORDS ? snap->seg[0].n : DUMP_RAW_WORDS;
    snprintf(extra, sizeof(extra), "\"words\":%" PRIu32 ",\"fs\":%" PRIu32 ",\"format\":\"u32le_iq10\"", n, snap->fs_hz);
    dump_hex_lines("raw", (const uint8_t *)snap->seg[0].p, (size_t)n * 4, extra);
}

/* ------------------------------------------------------------ step */

/* Full-pass snapshots in the window; optionally also the snapshots (full
 * or weak) that agree with `sf`/`slope`. */
/* How many separate channel groups the hits of one SF in the window sit on.
 * Offsets are floored to whole MHz, so a signal reported at x.4 or x.6 stays
 * in one bucket; buckets within 1 MHz of each other count as one. The chirp
 * direction is ignored: it says nothing about the channel, and a stray hit
 * with the other direction made the fixed-channel verdict flap (simulator). */
static unsigned window_clusters(uint64_t now_us, uint8_t sf, unsigned *hits)
{
    int8_t seen[HISTORY];
    unsigned n = 0, h = 0;
    for (unsigned k = 0; k < s.count; k++) {
        const snap_t *e = &s.hist[(s.head + HISTORY - 1 - k) % HISTORY];
        if ((uint32_t)(now_us / 1000u) - e->t_ms > WINDOW_US / 1000u) {
            break;
        }
        if (e->weak && e->sf == sf) {
            seen[n++] = e->chan;
            h++;
        }
    }
    /* insertion sort, then count the gaps larger than one bucket */
    for (unsigned i = 1; i < n; i++) {
        const int8_t v = seen[i];
        unsigned j = i;
        for (; j > 0 && seen[j - 1] > v; j--) {
            seen[j] = seen[j - 1];
        }
        seen[j] = v;
    }
    unsigned clusters = n ? 1u : 0u;
    for (unsigned i = 1; i < n; i++) {
        clusters += seen[i] - seen[i - 1] > 1;
    }
    if (hits) {
        *hits = h;
    }
    return clusters;
}

static unsigned window_hits(uint64_t now_us, unsigned *snaps, uint8_t sf, int8_t slope, unsigned *agree)
{
    unsigned hits = 0, n = 0, same = 0;
    for (unsigned k = 0; k < s.count; k++) {
        const snap_t *h = &s.hist[(s.head + HISTORY - 1 - k) % HISTORY];
        if ((uint32_t)(now_us / 1000u) - h->t_ms > WINDOW_US / 1000u) {
            break;
        }
        n++;
        hits += h->hit;
        same += h->weak && h->sf == sf && h->slope == slope;
    }
    if (snaps) {
        *snaps = n;
    }
    if (agree) {
        *agree = same;
    }
    return hits;
}

void chirp_watch_step(uint64_t now_us)
{
    if (now_us < s.next_us) {
        return;
    }
    const int64_t t_step = esp_timer_get_time();

    size_t scratch_bytes = 0;
    float *work = sdr_engine_scratch(&scratch_bytes);
    if (work == NULL || scratch_bytes < LORA_WORK_FLOATS * sizeof(float)) {
        return;   /* sink not empty: skip this pass */
    }
    s.work_mod16 = (unsigned)((uintptr_t)work & 15u);

    sdr_capture_t cap;
    sdr_engine_capture(s.rate_code, CAPTURE_UNITS, &cap);
    if (cap.status != 0 || cap.nseg == 0) {
        s.cap_errors++;
        s.next_us = now_us + 100000u;   /* do not retry a failing capture every pass */
        return;
    }
    lora_snapshot_t snap;
    memset(&snap, 0, sizeof(snap));
    snap.fs_hz = cap.fs_hz;
    for (unsigned k = 0; k < cap.nseg && k < LORA_MAX_SEGS; k++) {
        snap.seg[k] = (lora_seg_t){ cap.seg[k], cap.len[k] };
        snap.nseg++;
    }

    const int64_t t0 = esp_timer_get_time();
    lora_result_t res;
    if (!lora_scan(&snap, work, LORA_WORK_FLOATS, &s.floor, &res)) {
        s.cap_errors++;
        return;
    }
    s.scan_us_last = (uint32_t)(esp_timer_get_time() - t0);
    s.scan_us_max = s.scan_us_last > s.scan_us_max ? s.scan_us_last : s.scan_us_max;
    s.last = res;
    s.snaps_total++;

    s.hist[s.head] = (snap_t){ (uint32_t)(now_us / 1000u), res.detected, res.weak, res.sf, res.slope,
                               (int8_t)floorf(res.freq_hz / 1e6f) };
    s.head = (s.head + 1) % HISTORY;
    s.count += s.count < HISTORY;
    if (now_us >= s.active_until_us) {
        s.have_ref = false;               /* no detection running: next one starts afresh */
    }
    s.hits_total += res.detected;
    s.weak_total += res.weak && !res.detected;

    /* Does this snapshot confirm a link? A full pass always counts as
     * evidence; a weak one only when enough agree with it. */
    unsigned agree = 0;
    const unsigned hits = res.weak ? window_hits(now_us, NULL, res.sf, res.slope, &agree) : 0;
    const bool by_full = res.detected && hits >= MIN_HITS;
    const bool by_weak = res.weak && agree >= MIN_WEAK_AGREE;
    if (by_full || by_weak) {
        s.by_weak = !by_full && (s.by_weak || now_us >= s.active_until_us);
        s.active_until_us = now_us + HOLD_US;
    }
    if (res.detected || by_weak) {
        s.snr_smooth = s.have_ref ? s.snr_smooth + SNR_SMOOTH * (res.snr_db - s.snr_smooth) : res.snr_db;
        if (!s.have_ref) {
            s.snr_ref = res.snr_db;
            s.have_ref = true;
        }
        s.last_hit_us = now_us;
        s.last_sf = res.sf;
        s.last_hit = res;
    }
    if (s.dump_requested) {
        s.dump_requested = false;
        dump(&snap, work, &res);
    }
    const int64_t t_end = esp_timer_get_time();
    s.step_us_last = (uint32_t)(t_end - t_step);
    s.next_us = (uint64_t)t_end + COST_FACTOR * (uint64_t)s.step_us_last;
}

void chirp_watch_status(uint64_t now_us, chirp_status_t *out)
{
    unsigned snaps = 0;
    const unsigned hits = window_hits(now_us, &snaps, 0, 0, NULL);
    out->active = now_us < s.active_until_us;
    out->by_weak = out->active && s.by_weak;
    out->sf = s.last_sf;
    out->hits = (uint16_t)hits;
    out->snaps = (uint16_t)snaps;
    out->last_hit_us = s.last_hit_us;
    out->snr_db = s.last_hit.snr_db;
    out->level_dbfs = s.last_hit.level_dbfs;
    out->agc = s.last_hit.agc;
    out->snr_smooth_db = s.snr_smooth;
    out->delta_db = s.have_ref ? s.snr_smooth - s.snr_ref : 0.0f;
    out->clip_frac = s.last.clip_frac;
    unsigned lhits = 0;
    const unsigned clusters = window_clusters(now_us, s.last_hit.sf, &lhits);
    out->hop_clusters = (uint8_t)(clusters > 255u ? 255u : clusters);
    out->fixed_channel = out->active && lhits >= FIXED_MIN_HITS && clusters == 1u;
}

void chirp_watch_log(uint64_t now_us)
{
    unsigned snaps = 0, agree = 0;
    const unsigned hits = window_hits(now_us, &snaps, s.last_hit.sf, s.last_hit.slope, &agree);
    /* The confirming scan if there was one in the window, else the latest. */
    const bool recent = s.last_hit_us && now_us - s.last_hit_us <= WINDOW_US;
    const lora_result_t *r = recent ? &s.last_hit : &s.last;
    char sfs[96];
    size_t w = 0;
    for (unsigned k = 0; k <= LORA_SF_MAX - LORA_SF_MIN && w < sizeof(sfs); k++) {
        const lora_sf_metric_t *m = &r->sf_metric[k];
        const int n = snprintf(sfs + w, sizeof(sfs) - w, "%s[%.0f,%.1f,%.2f,%.2f]", k ? "," : "",
                               (double)m->score, (double)m->asym, (double)m->hit_ratio,
                               (double)m->hit_ratio_weak);
        if (n < 0) {
            break;
        }
        w += (size_t)n;
    }
    wids_log_event("lora",
                   "\"window_snaps\":%u,\"window_hits\":%u,\"window_agree\":%u,\"active\":%s,"
                   "\"tier\":\"%s\",\"weak\":%s,\"sf\":%u,\"slope\":%d,"
                   "\"freq_mhz\":%.2f,\"cand_db\":%.1f,\"snr_db\":%.1f,\"dbfs\":%.1f,\"agc\":%u,"
                   "\"snr_smooth\":%.1f,\"delta_db\":%.1f,"
                   "\"hop_clusters\":%u,\"cands\":%u,\"clip_pct\":%.1f,\"work_mod16\":%u,\"sf5_8\":[%s],\"scan_ms\":%.1f,"
                   "\"scan_ms_max\":%.1f,\"step_ms\":%.1f,\"snaps\":%" PRIu32 ",\"hits\":%" PRIu32
                   ",\"weak_only\":%" PRIu32 ",\"cap_err\":%" PRIu32,
                   snaps, hits, agree, now_us < s.active_until_us ? "true" : "false",
                   now_us >= s.active_until_us ? "none" : s.by_weak ? "weak" : "full",
                   r->weak && !r->detected ? "true" : "false", r->sf, r->slope,
                   (double)r->freq_hz / 1e6, (double)r->cand_db, (double)r->snr_db, (double)r->level_dbfs,
                   r->agc, (double)s.snr_smooth, (double)(s.have_ref ? s.snr_smooth - s.snr_ref : 0.0f),
                   window_clusters(now_us, s.last_hit.sf, NULL), r->candidates,
                   (double)(100.0f * s.last.clip_frac), s.work_mod16, sfs,
                   (double)s.scan_us_last / 1000.0, (double)s.scan_us_max / 1000.0,
                   (double)s.step_us_last / 1000.0, s.snaps_total, s.hits_total, s.weak_total, s.cap_errors);
}
