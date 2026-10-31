/*
 * band_watch.h - microwave ovens and wide signals, from spectrum frames.
 *
 * PRESENCE ONLY, 2.4 GHz ONLY. Works on the same max-hold power frames the
 * ELRS grid detector gets (~2 ms each, 256 bins of 312.5 kHz). It never
 * demodulates anything and keeps no identifiers: a power spectrum has none.
 *
 * 1. Microwave ovens. A household oven's magnetron runs from a half-wave
 *    voltage doubler, so it radiates for about half of every mains cycle and
 *    is silent for the other half: on/off at exactly the mains rate (50 Hz
 *    in Finland, a 20 ms period), mostly around 2.45-2.47 GHz, sweeping a few
 *    MHz while on (NIST microwave-oven recordings; IEEE 802.11-98/240 measured
 *    fourteen ovens: ~50 % duty, one burst per mains cycle). Nothing else on
 *    the band is locked to the mains.
 *    Per frame, every narrow occupied bin in 2440-2480 MHz is noted in 1 MHz
 *    buckets. Bins inside a wide (Wi-Fi-size) run are left out: a busy access
 *    point on channels 6-13 overlaps the same range, and counting its bursts
 *    as "oven on" broke the rhythm in the simulator. Per capture slice
 *    (100 ms, five cycles) the 10 MHz window holding most of that activity is
 *    the candidate, and its on/off series is tested for:
 *      - a strong component at the mains rate (Rayleigh-style: the first
 *        Fourier coefficient of the on/off series over its mean deviation;
 *        0.64 for a 50 % square wave, ~0.14 for random bursts);
 *      - an on-share between 20 and 80 %;
 *      - something on in (nearly) every mains cycle;
 *      - most of the range's activity inside the window (ELRS and Bluetooth
 *        hop everywhere; an oven stays within a few MHz).
 *    Inverter ovens drive the magnetron from a high-frequency supply and may
 *    not pulse at the mains rate: those are NOT recognised here. A non-hopping
 *    narrowband transmitter keyed at exactly 50 Hz would pass the same tests;
 *    the caller drops the oven label when the chirp detector sees fixed LoRa.
 *
 * 2. Wide occupancy. Per 1 MHz bucket, how often a run of above-noise bins
 *    at least 6 MHz wide covers it (decaying average, ~3 s). Wi-Fi traffic,
 *    DJI-style video links and other wide signals light it; Bluetooth, BLE
 *    and ELRS do not. ofdm_detect.h uses it to choose where to look.
 *
 * Frequencies inside this module are offsets from the LO in the frames' own
 * (raw) orientation; band_rf_mhz() maps them to RF using `invert`, the same
 * flag the ELRS detector uses.
 *
 * Pure C, no ESP-IDF: unit-tested on the host (test/test_band_detect.c).
 */
#ifndef BAND_WATCH_H
#define BAND_WATCH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BAND_BINS          256u   /* frame size this module accepts */
#define BAND_SLICE_FRAMES  64u    /* frames kept per slice (100 ms / ~2 ms = ~50; heap is tight) */
#define BAND_BUCKETS       66u    /* 1 MHz wide-occupancy buckets, LO-33 .. LO+33 MHz */
#define BAND_MW_HISTORY    40u    /* slice verdicts kept (~5 s of slices) */

typedef struct {
    uint32_t center_khz;   /* receiver LO */
    uint32_t fs_hz;        /* complex sample rate = frame span */
    uint32_t usable_khz;   /* analog bandwidth to trust, centred on the LO */
    bool     invert;       /* mirrored spectrum (same meaning as elrs_config_t) */
    float    on_db;        /* "occupied": this far above the frame's quiet level */
    uint32_t mw_lo_khz, mw_hi_khz;   /* RF range where household ovens radiate */
} band_config_t;

#define BAND_MW_BUCKETS 40u   /* 1 MHz buckets of the oven range (mw_lo..mw_hi) */
#define BAND_MW_WINDOW  11u   /* candidate window, buckets (+-5 MHz) */

/* One frame, as the slice analysis needs it. */
typedef struct {
    uint32_t t_us;       /* frame middle, from the slice's first frame */
    uint64_t occ;        /* bit j: narrow activity in oven-range bucket j */
} band_frame_t;

typedef struct {
    uint32_t t_ms;
    bool     pass;
    uint8_t  mains_hz;
    float    r;
    float    rf_mhz;     /* centre of the candidate window */
} band_slice_t;

typedef struct {
    bool     active;
    float    rf_mhz;      /* where the oven radiates (median of recent slices) */
    uint8_t  mains_hz;    /* 50 or 60 */
    unsigned passes, slices;  /* slices that passed / analysed, last 3 s */
    /* the last analysed slice, for logs */
    float    r, duty, cover;
    float    stable;      /* share of the range's activity inside the window */
    float    width_mhz;   /* mean occupied MHz per on-frame inside the window */
    unsigned frames;
} band_mw_t;

typedef struct {
    band_config_t cfg;
    float    bin_khz;
    int      half_bins;                  /* usable half-width in bins */
    float    duty[BAND_BUCKETS];         /* wide occupancy per bucket, 0..1 */
    band_frame_t fr[BAND_SLICE_FRAMES];
    unsigned nfr;
    uint64_t slice_t0_us;
    band_slice_t hist[BAND_MW_HISTORY];
    unsigned head, count;
    uint64_t mw_until_us;
    float    mw_rf_mhz;
    uint8_t  mw_mains;
    band_mw_t last;                      /* diagnostics of the last slice */
    uint32_t frames;
} band_watch_t;

void band_default_config(band_config_t *c, uint32_t center_khz, uint32_t fs_hz, uint32_t usable_khz);
void band_init(band_watch_t *w, const band_config_t *c);

/* One spectrum frame (esp-sdr codes, db_step per dB, natural FFT order),
 * starting at t_us and lasting dur_us. Other bin counts are ignored. */
void band_push_frame(band_watch_t *w, const uint8_t *codes, unsigned bins, uint8_t db_step,
                     uint64_t t_us, uint32_t dur_us);

/* Call after the frames of one capture slice: analyses them for the oven
 * rhythm and starts the next slice. */
void band_slice_done(band_watch_t *w, uint64_t now_us);

void band_microwave(const band_watch_t *w, uint64_t now_us, band_mw_t *out);

/* Highest wide occupancy within +-half_mhz of a raw offset (MHz). */
float band_wide_duty(const band_watch_t *w, float off_mhz, float half_mhz);

/* Raw LO offset (MHz) to RF (MHz). */
float band_rf_mhz(const band_watch_t *w, float off_mhz);

#ifdef __cplusplus
}
#endif
#endif /* BAND_WATCH_H */
