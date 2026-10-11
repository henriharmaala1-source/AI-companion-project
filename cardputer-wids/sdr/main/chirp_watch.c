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
#define EVERY_N_PASSES  2u          /* one snapshot per this many slices */
#define WINDOW_US       5000000ull  /* evidence window */
#define MIN_HITS        2u          /* positives in the window for a verdict */
#define HOLD_US         10000000ull /* verdict held after the last confirming hit */
#define HISTORY         32u         /* > snapshots in WINDOW_US (~23) */
#define DUMP_SAMPLES_PER_LINE 48u   /* 192 bytes -> 384 hex chars per log line */
#define DUMP_RAW_WORDS  384u

typedef struct {
    uint32_t t_ms;                  /* ms since boot: wraps after 49 days */
    bool     hit;
} snap_t;

static struct {
    unsigned rate_code;
    unsigned pass;
    snap_t   hist[HISTORY];
    unsigned head, count;
    uint64_t active_until_us, last_hit_us;
    uint8_t  last_sf;
    lora_result_t last;              /* most recent scan */
    lora_result_t last_hit;          /* most recent positive scan */
    uint32_t snaps_total, hits_total, cap_errors, scan_us_max, scan_us_last;
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

static unsigned window_hits(uint64_t now_us, unsigned *snaps)
{
    unsigned hits = 0, n = 0;
    for (unsigned k = 0; k < s.count; k++) {
        const snap_t *h = &s.hist[(s.head + HISTORY - 1 - k) % HISTORY];
        if ((uint32_t)(now_us / 1000u) - h->t_ms > WINDOW_US / 1000u) {
            break;
        }
        n++;
        hits += h->hit;
    }
    if (snaps) {
        *snaps = n;
    }
    return hits;
}

void chirp_watch_step(uint64_t now_us)
{
    if (++s.pass < EVERY_N_PASSES) {
        return;
    }
    s.pass = 0;

    size_t scratch_bytes = 0;
    float *work = sdr_engine_scratch(&scratch_bytes);
    if (work == NULL || scratch_bytes < LORA_WORK_FLOATS * sizeof(float)) {
        return;   /* sink not empty: skip this pass */
    }

    sdr_capture_t cap;
    sdr_engine_capture(s.rate_code, CAPTURE_UNITS, &cap);
    if (cap.status != 0 || cap.nseg == 0) {
        s.cap_errors++;
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
    if (!lora_scan(&snap, work, LORA_WORK_FLOATS, &res)) {
        s.cap_errors++;
        return;
    }
    s.scan_us_last = (uint32_t)(esp_timer_get_time() - t0);
    s.scan_us_max = s.scan_us_last > s.scan_us_max ? s.scan_us_last : s.scan_us_max;
    s.last = res;
    s.snaps_total++;

    s.hist[s.head] = (snap_t){ (uint32_t)(now_us / 1000u), res.detected };
    s.head = (s.head + 1) % HISTORY;
    s.count += s.count < HISTORY;
    if (res.detected) {
        s.hits_total++;
        s.last_hit_us = now_us;
        s.last_sf = res.sf;
        s.last_hit = res;
        if (window_hits(now_us, NULL) >= MIN_HITS) {
            s.active_until_us = now_us + HOLD_US;
        }
    }
    if (s.dump_requested) {
        s.dump_requested = false;
        dump(&snap, work, &res);
    }
}

void chirp_watch_status(uint64_t now_us, chirp_status_t *out)
{
    unsigned snaps = 0;
    const unsigned hits = window_hits(now_us, &snaps);
    out->active = now_us < s.active_until_us;
    out->sf = s.last_sf;
    out->hits = (uint16_t)hits;
    out->snaps = (uint16_t)snaps;
    out->last_hit_us = s.last_hit_us;
}

void chirp_watch_log(uint64_t now_us)
{
    unsigned snaps = 0;
    const unsigned hits = window_hits(now_us, &snaps);
    /* The positive scan if there was one in the window, else the latest. */
    const lora_result_t *r = hits ? &s.last_hit : &s.last;
    char sfs[96];
    size_t w = 0;
    for (unsigned k = 0; k <= LORA_SF_MAX - LORA_SF_MIN && w < sizeof(sfs); k++) {
        const lora_sf_metric_t *m = &r->sf_metric[k];
        const int n = snprintf(sfs + w, sizeof(sfs) - w, "%s[%.0f,%.1f,%.2f]", k ? "," : "", (double)m->score,
                               (double)m->asym, (double)m->hit_ratio);
        if (n < 0) {
            break;
        }
        w += (size_t)n;
    }
    wids_log_event("lora",
                   "\"window_snaps\":%u,\"window_hits\":%u,\"active\":%s,\"sf\":%u,\"slope\":%d,"
                   "\"freq_mhz\":%.2f,\"cand_db\":%.1f,\"cands\":%u,\"sf5_8\":[%s],\"scan_ms\":%.1f,"
                   "\"scan_ms_max\":%.1f,\"snaps\":%" PRIu32 ",\"hits\":%" PRIu32 ",\"cap_err\":%" PRIu32,
                   snaps, hits, now_us < s.active_until_us ? "true" : "false", r->sf, r->slope,
                   (double)r->freq_hz / 1e6, (double)r->cand_db, r->candidates, sfs,
                   (double)s.scan_us_last / 1000.0, (double)s.scan_us_max / 1000.0, s.snaps_total, s.hits_total,
                   s.cap_errors);
}
