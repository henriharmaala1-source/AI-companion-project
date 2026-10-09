/*
 * elrs_detect.h - recognise an ExpressLRS 2.4 GHz link from spectrum frames.
 *
 * PRESENCE ONLY. This looks at power per frequency bin over time. It never
 * demodulates, decodes or identifies anyone, and it stores no identifiers
 * because there are none in a power spectrum.
 *
 * What it looks for (from the ExpressLRS source, see docs/ELRS_SDR_PLAN.md):
 *   - narrow bursts, 0.6-0.8 MHz wide;
 *   - centred on the ELRS grid 2400.4 + k MHz (k = 0..79), i.e. at x.4 MHz,
 *     where Bluetooth Classic, BLE and 802.15.4 (all on integer MHz) and
 *     Wi-Fi (20 MHz wide) do not sit. Integer-MHz bursts are counted apart,
 *     so a room full of Bluetooth does not hide an ELRS link;
 *   - spread evenly over the whole grid, because every 80-hop block visits
 *     each channel exactly once;
 *   - held for a few milliseconds per hop (2-40 ms depending on packet rate).
 *
 * Verdicts are deliberately graded:
 *   ELRS_QUIET   nothing that fits on 2.4 GHz - NOT "all clear";
 *   ELRS_HOPPER  narrowband frequency hopping, grid not confirmed;
 *   ELRS_LIKELY  hopping that matches the ELRS grid and its uniform coverage.
 *
 * Pure C, no ESP-IDF dependency: unit-tested on the host.
 */
#ifndef ELRS_DETECT_H
#define ELRS_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_CHANNELS     80u
#define ELRS_BASE_KHZ     2400400u   /* channel 0 */
#define ELRS_STEP_KHZ     1000u
#define ELRS_SYNC_CHANNEL 40u        /* 2440.4 MHz, first hop of every block */
#define ELRS_MAX_BINS     2048u
#define ELRS_DWELL_BUCKETS 7u

typedef enum { ELRS_QUIET = 0, ELRS_HOPPER = 1, ELRS_LIKELY = 2 } elrs_state_t;

typedef struct {
    uint32_t center_khz;      /* receiver LO */
    uint32_t sample_rate_hz;  /* complex rate, equals the displayed span */
    uint32_t usable_khz;      /* analogue bandwidth to trust, centred on the LO */
    bool     invert;          /* flip sign of bin offsets (verify on hardware, M0) */
    uint8_t  threshold_db;    /* burst must exceed the floor by this much */
    uint16_t grid_tol_khz;    /* max distance from a grid line to count as on-grid */
    uint16_t int_tol_khz;     /* max distance from an integer MHz to count as Bluetooth-like */
    uint16_t narrow_min_khz, narrow_max_khz;
    uint16_t wide_min_khz;    /* wider than this is Wi-Fi / microwave, ignored */
    float    window_s;        /* statistics time constant */
    float    hold_s;          /* how long a verdict is held before downgrading */
} elrs_config_t;

typedef struct {
    elrs_state_t state;
    float    score;            /* 0..1 confidence in ELRS_LIKELY */
    uint16_t channels_seen;    /* on-grid channels visited within the window */
    uint16_t channels_visible; /* grid channels inside the usable band */
    uint16_t distinct_narrow;  /* 1 MHz slots with narrow bursts, on- or off-grid */
    float    on_grid_ratio;    /* on-grid / (on-grid + off-grid), integer-MHz bursts excluded */
    float    int_grid_ratio;   /* integer-MHz (Bluetooth-like) share of all narrow bursts */
    float    on_now_per_s;     /* on-grid bursts per second, 1 s time constant */
    float    uniformity_cv;    /* spread of visits over seen channels, lower = flatter */
    float    narrow_per_s, wide_per_s;
    float    dwell_ms;         /* typical hop dwell (bucket centre) */
    int16_t  floor_db, peak_db;/* code/db_step, i.e. dB relative to FFT full scale */
    uint64_t last_seen_us;     /* last on-grid burst (noise can produce one) */
    uint64_t last_likely_us;   /* last evaluation that ended in ELRS_LIKELY, 0 = never */
    uint32_t frames;
    uint8_t  activity[ELRS_CHANNELS]; /* 0..255 per channel, for the screen */
} elrs_status_t;

typedef struct {
    elrs_config_t cfg;
    unsigned bins;
    uint8_t  db_step;
    bool     floor_ready;
    uint16_t floor_q4[ELRS_MAX_BINS];  /* per-bin floor, code * 16 */
    uint8_t  visible[ELRS_CHANNELS];
    uint16_t n_visible;
    /* decayed accumulators */
    float    visits[ELRS_CHANNELS];     /* on-grid hops per channel */
    float    spread[ELRS_CHANNELS];     /* any narrow burst, by nearest channel */
    float    on_bursts, int_bursts, off_bursts, wide_runs;
    float    recent_narrow;   /* same bursts, 1 s time constant: "is it happening now" */
    float    recent_on;       /* on-grid bursts, 1 s time constant */
    float    dwell_hist[ELRS_DWELL_BUCKETS];
    /* per-channel visit tracking */
    uint32_t last_hit_frame[ELRS_CHANNELS];
    uint64_t visit_start_us[ELRS_CHANNELS];
    uint64_t visit_last_us[ELRS_CHANNELS];
    bool     in_visit[ELRS_CHANNELS];
    /* bookkeeping */
    uint32_t frame_no;
    uint64_t last_t_us;
    uint64_t last_seen_us;
    uint64_t last_likely_us;
    uint8_t  last_peak_code;
    elrs_state_t state;
    elrs_state_t pending;
    unsigned pending_count;
    uint64_t support_until_us; /* current state is held until here */
} elrs_detector_t;

void elrs_default_config(elrs_config_t *cfg, uint32_t center_khz, uint32_t sample_rate_hz);
void elrs_init(elrs_detector_t *d, const elrs_config_t *cfg);

/* One spectrum frame. `codes` are esp-sdr power codes (db_step per dB),
 * natural FFT order. `t_us` is the frame's start on a monotonic clock and
 * `dur_us` the time it spans. Frames with an unexpected bin count are ignored. */
void elrs_push_frame(elrs_detector_t *d, const uint8_t *codes, unsigned bins,
                     uint8_t db_step, uint64_t t_us, uint32_t dur_us);

/* Recompute statistics and the verdict. Returns true when the state changed. */
bool elrs_evaluate(elrs_detector_t *d, uint64_t now_us, elrs_status_t *out);

const char *elrs_state_name(elrs_state_t s);

#ifdef __cplusplus
}
#endif
#endif /* ELRS_DETECT_H */
