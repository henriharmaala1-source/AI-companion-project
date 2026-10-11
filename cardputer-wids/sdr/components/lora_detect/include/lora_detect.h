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
#define LORA_FFT_MAX      512u       /* SF8 window at 1.25 MS/s is 394 samples */

/* Scratch the caller provides (floats). The firmware lends it the empty
 * spectrum sink between slices; the heap is too small. */
#define LORA_FIR_TAPS     96u        /* decimator stage 2, kept in the scratch */
#define LORA_WORK_FLOATS  (LORA_COARSE_N + 2u * LORA_FFT_MAX * 2u + 2u * LORA_MAX_BASEBAND + LORA_FIR_TAPS)

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
    int8_t slope;            /* +1 up-chirps, -1 down-chirps (in the snapshot's frame) */
} lora_sf_metric_t;

typedef struct {
    bool     detected;
    uint8_t  sf;             /* 5..8 when detected, else best guess or 0 */
    int8_t   slope;
    float    freq_hz;        /* candidate: ELRS grid position, offset from the LO, snapshot frame.
                              * +-200 kHz: the x.4 and x.6 positions (normal and mirrored
                              * spectrum) both dechirp a burst between them, so this
                              * does NOT settle the spectrum orientation. */
    float    cand_db;        /* candidate channel over its neighbours (narrowband excess) */
    unsigned candidates;     /* peaks tested */
    lora_sf_metric_t sf_metric[LORA_SF_MAX - LORA_SF_MIN + 1u]; /* for the reported candidate */
} lora_result_t;

/* Samples in the snapshot. */
uint32_t lora_snapshot_len(const lora_snapshot_t *s);

/* Look for LoRa in the snapshot. `work` must hold LORA_WORK_FLOATS floats.
 * Returns false only on bad arguments. */
bool lora_scan(const lora_snapshot_t *s, float *work, size_t work_floats, lora_result_t *out);

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
