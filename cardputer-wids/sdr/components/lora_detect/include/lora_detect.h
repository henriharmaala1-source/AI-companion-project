/*
 * lora_detect.h - recognise LoRa chirps in a raw I/Q snapshot.
 *
 * PRESENCE ONLY. This finds out whether a LoRa transmission is on the air
 * and which spreading factor it uses. It never recovers symbols, packets,
 * stick data or identities: the dechirp below measures how much energy
 * collapses into one FFT bin, nothing more.
 *
 * Why chirps: a LoRa symbol is a tone sweeping linearly across its bandwidth.
 * Multiply it by a sweep of the opposite slope and it becomes a pure tone,
 * a single sharp FFT peak. Use the wrong slope and it spreads out. Wi-Fi,
 * Bluetooth, carriers and noise respond the same to both slopes. "One slope
 * concentrates, the other does not" is therefore a fingerprint for LoRa.
 *
 * ExpressLRS 2.4 GHz LoRa (src/src/common.cpp, ExpressLRS master):
 *   bandwidth SX1280_LORA_BW_0800 = 812.5 kHz for every LoRa rate
 *   500 Hz SF5, 333 Hz SF5, 250 Hz SF6, 150 Hz SF7, 100 Hz SF7, 50 Hz SF8
 * and invertIQ = UID[5] & 1 (tx_main.cpp), so about half of all links send
 * down-chirps. Both slopes are tested. The FLRC rates (F500, F1000, D-modes)
 * are not LoRa and are not seen by this module.
 *
 * Input: esp-sdr S3 ring words, signed 10-bit I in bits 0..9 and Q in bits
 * 10..19 (esp-sdr s3_unpack.S). Pure C, no ESP-IDF: tested on the host.
 */
#ifndef LORA_DETECT_H
#define LORA_DETECT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_BW_HZ        812500.0f  /* SX1280 BW_0800 */
#define LORA_SF_MIN       5u
#define LORA_SF_MAX       8u
#define LORA_MAX_SEGS     6u         /* 3 ring units, each may wrap once */
#define LORA_MAX_CAND     6u         /* spectral peaks tested per snapshot */
#define LORA_COARSE_N     256u       /* coarse spectrum used to pick candidates */
#define LORA_DECIM_RATE   1250000u   /* baseband rate after decimation */
#define LORA_MAX_BASEBAND 800u       /* decimated samples kept per candidate */
#define LORA_FFT_MAX      256u       /* SF8's 394-sample symbol is cut to 256: still ~30x over noise */

/* Scratch the caller provides (floats). The firmware lends it the empty
 * spectrum sink between slices; the heap is too small. */
#define LORA_FIR_TAPS     96u        /* decimator stage 2, kept in the scratch */
/* +4: lora_scan() aligns `work` to 16 bytes itself (the S3 FFT needs it). */
#define LORA_WORK_FLOATS  (LORA_COARSE_N + 2u * LORA_FFT_MAX * 2u + 2u * LORA_MAX_BASEBAND + LORA_FIR_TAPS + 4u)

typedef struct {
    const uint32_t *p;
    uint32_t n;              /* words */
} lora_seg_t;

/* A gapless run of samples, possibly in several pieces (ring wrap). */
typedef struct {
    lora_seg_t seg[LORA_MAX_SEGS];
    unsigned   nseg;
    uint32_t   fs_hz;
} lora_snapshot_t;

typedef struct {
    float score;             /* concentration: top-bin energy / mean, best window */
    float asym;              /* that score over the other slope's, same window */
    float hit_ratio;         /* windows that passed both tests / all windows */
    float hit_ratio_weak;    /* the same at the lower "weak" score floor */
    int8_t slope;            /* +1 up-chirps, -1 down-chirps (in the snapshot's frame) */
} lora_sf_metric_t;

/* Two tiers of evidence per snapshot:
 *   detected - passes the full tests. One such snapshot is already rare
 *              without LoRa (0 in every host negative case).
 *   weak     - passes only at a lower score floor (detected implies weak).
 *              About 1 snapshot in 150 with Bluetooth/Wi-Fi-like signals
 *              is weak by chance, at a random SF and slope, so a weak hit
 *              means something only when several agree (chirp_watch.c). */
typedef struct {
    bool     detected;
    bool     weak;
    uint8_t  sf;             /* 5..8 when detected or weak, else 0 */
    int8_t   slope;
    float    freq_hz;        /* candidate: ELRS grid position, offset from the LO, snapshot frame.
                              * +-200 kHz: the x.4 and x.6 positions (normal and mirrored
                              * spectrum) both dechirp a burst between them, so this
                              * does NOT settle the spectrum orientation. */
    float    cand_db;        /* candidate channel over its neighbours (narrowband excess) */
    float    level_dbfs;     /* detected signal power, dB relative to ADC full scale */
    float    snr_db;         /* detected signal over the noise in its 1.2 MHz baseband */
    uint8_t  agc;            /* receiver gain index stamped in the snapshot (AGC moves it):
                              * dBFS is only comparable at the same index */
    unsigned candidates;     /* peaks tested */
    float    clip_frac;      /* samples at the ADC rail (of those read for the coarse
                              * spectrum): the Wi-Fi AGC lets a strong continuous
                              * signal clip (esp-sdr docs/rx-controls.md) */
    lora_sf_metric_t sf_metric[LORA_SF_MAX - LORA_SF_MIN + 1u]; /* for the reported candidate */
} lora_result_t;

/* The receiver's long-term noise floor per coarse FFT bin, kept by the
 * caller from one scan to the next (zero-initialise it once).
 *
 * Why: at 80 MS/s most of the spectrum lies outside the receiver's analog
 * filter, ~25 dB down (esp-sdr docs/rx-controls.md). Each filter edge is a
 * step that looks like a strong signal to a single-snapshot "stands out
 * from its neighbours" test, and edges filled every candidate slot (host
 * test: 6/20 detected at 0 dB behind a 30 MHz filter). An edge is in the
 * same place every snapshot; ELRS hops. Against a floor that follows the
 * quietest recent level of each bin, edges and steady carriers vanish and
 * a hopping signal stands out. A side effect: LoRa parked on one channel
 * fades into the floor over tens of seconds. */
typedef struct {
    float   bin[LORA_COARSE_N];
    bool    ready;
    uint8_t agc;        /* gain index the floor was learned at */
    uint8_t other_agc;  /* consecutive snapshots at a different gain index */
} lora_floor_t;

/* Samples in the snapshot. */
uint32_t lora_snapshot_len(const lora_snapshot_t *s);

/* Look for LoRa in the snapshot. `work` must hold LORA_WORK_FLOATS floats.
 * `floor` may be NULL (each snapshot judged alone); otherwise it is used
 * and updated. Returns false only on bad arguments. */
bool lora_scan(const lora_snapshot_t *s, float *work, size_t work_floats, lora_floor_t *floor,
               lora_result_t *out);

/* The candidate at freq_hz mixed to 0 Hz and decimated to 1.25 MS/s, as the
 * scan sees it (for logging a snapshot to check offline). Returns samples,
 * interleaved re/im in z (2 * max floats). */
unsigned lora_baseband(const lora_snapshot_t *s, float freq_hz, float *z, unsigned max);

/* Unpack one ring word (exposed for tests). */
void lora_unpack(uint32_t word, float *i, float *q);

/* In-place radix-2 complex FFT, n a power of two <= LORA_FFT_MAX, data
 * interleaved re/im (exposed for tests). */
void lora_fft(float *data, unsigned n);

#ifdef __cplusplus
}
#endif
#endif /* LORA_DETECT_H */
