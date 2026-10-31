/*
 * ofdm_detect.h - tell Wi-Fi from DJI-style OFDM in a raw I/Q snapshot.
 *
 * PRESENCE ONLY, 2.4 GHz ONLY. Nothing is demodulated or decoded: this
 * measures one number per candidate symbol length, how much the signal
 * repeats itself at that delay.
 *
 * Why that works: an OFDM symbol starts with a cyclic prefix, a copy of its
 * own last part. So the signal correlates with itself delayed by exactly the
 * useful symbol length Tu, and at no nearby delay. Tu is fixed by the
 * subcarrier spacing (Tu = 1 / spacing), and different systems use different
 * spacings:
 *
 *   Wi-Fi 802.11a/g/n/ac  312.5 kHz -> Tu  3.2 us, symbol 4.0 us (IEEE 802.11;
 *                         Wi-Fi 6 devices send their beacons and preambles so too)
 *   LTE-like, 30 kHz      Tu 33.3 us, prefix ~2.3 us
 *   LTE-like, 15 kHz      Tu 66.7 us, prefix ~4.7 us
 *
 * DJI OcuSync O3/O4 video links show LTE-like OFDM with 15 and 30 kHz
 * spacing (github.com/huangjiansheng321-cloud/dji-drone-ocusync-research,
 * one author's measurements), and DJI DroneID bursts use 15 kHz (NDSS 2023,
 * "Drone Security and the Mysterious Case of DJI's DroneID", and later
 * decoders). Phone networks also use LTE numerology but not in 2400-2483.5
 * MHz. So an LTE-like prefix in this band most likely means a drone video
 * link: shown as "DJI-like", never as a proven DJI (other drone links could
 * use the same numerology).
 *
 * Method, per 5 MHz slot: mix the slot to 0 Hz, sum blocks of 8 samples
 * (80 -> 10 MS/s through a third-order CIC filter, about +-2.6 MHz at -3 dB,
 * sidelobes ~39 dB down), then multiply
 * the stream by itself delayed by Tu (32, 333 and 667 samples at 10 MS/s).
 * The products are summed in timing bins across the symbol period (folded),
 * as in the classic cyclic-prefix timing estimator (van de Beek, Sandell,
 * Borjesson 1997): prefix pairs always land in the same few bins and add
 * up; everything else spreads evenly. Summing over the whole snapshot
 * instead buried an LTE prefix (6.6 % of the time) in the signal's own
 * random products (host probe: peak 0.04 against +-0.015 everywhere).
 *
 * The score is the best window of bins minus the median window, each
 * normalised by the power it sums. A carrier, Bluetooth, LoRa or noise has
 * no preferred timing and scores near the noise floor of this statistic
 * (~0.05-0.08); a clean OFDM signal that fills the snapshot scores ~0.6-0.9.
 *
 * One gate first: OFDM fills the slot's 10 MS/s band, so neighbouring
 * decimated samples are only loosely related (|lag-1 correlation| ~0.4-0.5,
 * set by the CIC's own shape). A
 * LoRa chirp, Bluetooth or a carrier occupies ~1 MHz of it, is oversampled
 * ten times and correlates ~0.95+ with the next sample (several at once:
 * ~0.75); strong slow LoRa
 * (SF7/SF8, whose delayed product is a steady tone) otherwise scored up to
 * 8.5 at the LTE delays (host probe). Scans of a slot that a narrow signal
 * dominates (> OFDM_NARROW_MAX) are not counted as evidence.
 *
 * Input: esp-sdr S3 ring words at 80 MS/s (lora_detect.h's snapshot type).
 * Pure C, no ESP-IDF: unit-tested on the host (test/test_band_detect.c).
 */
#ifndef OFDM_DETECT_H
#define OFDM_DETECT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lora_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { OFDM_WIFI = 0, OFDM_LTE30, OFDM_LTE15, OFDM_KINDS } ofdm_kind_t;

#define OFDM_DECIM       8u
#define OFDM_NARROW_MAX  0.6f      /* see the gate above; set from host statistics */
#define OFDM_MAX_LAG     667u      /* the LTE15 family's delay */
#define OFDM_RING        (OFDM_MAX_LAG + 1u)
/* Scratch: a 256-entry cos/sin table, the delay line (complex) and the
 * timing-folded correlation sums. */
#define OFDM_CORR_FLOATS 304u
#define OFDM_WORK_FLOATS (2u * 256u + 2u * OFDM_RING + OFDM_CORR_FLOATS)

typedef struct {
    float    score[OFDM_KINDS];    /* best timing window over the rest, robust z, see above */
    float    contrast[OFDM_KINDS]; /* best window minus the median window (logs) */
    float    narrow;               /* |lag-1 correlation| / power: ~0 wideband, ~1 narrowband */
    uint16_t timing[OFDM_KINDS];   /* timing bin of the best window (logs only) */
    unsigned samples;              /* decimated samples used */
    float    level_db;             /* mean power after decimation, dB re 1 ADC unit^2 */
} ofdm_result_t;

/* Score the slot at off_hz (raw LO offset, rounded to the 312.5 kHz grid).
 * `work` holds OFDM_WORK_FLOATS floats. False on bad arguments or a sample
 * rate other than 80 MS/s. */
bool ofdm_scan(const lora_snapshot_t *s, float off_hz, float *work, size_t work_floats, ofdm_result_t *out);

const char *ofdm_kind_name(ofdm_kind_t k);

/* ------------------------------------------------------------ evidence */

/* Slots every 5 MHz from LO-30 to LO+30 MHz (raw orientation). With the LO
 * on 2442 MHz (Wi-Fi channel 7) the slot centres are the 13 Wi-Fi channel
 * centres. */
#define OFDM_SLOTS          13u
#define OFDM_SLOT_STEP_KHZ  5000
#define OFDM_SLOT_FIRST_KHZ (-30000)

typedef struct {
    float    avg[OFDM_KINDS];       /* running mean of the score (one scan weighs 0.3) */
    uint16_t scans;                 /* since the slot was last reset */
    uint64_t last_scan_us;
    uint64_t on_until_us[OFDM_KINDS];
} ofdm_slot_t;

typedef struct {
    ofdm_slot_t slot[OFDM_SLOTS];
    uint32_t scans_total;
    uint32_t narrow_skips;          /* scans ignored: a narrowband signal filled the slot */
    uint32_t picks;                 /* alternates ofdm_watch_pick() between its two rules */
    uint32_t scan_us_last;          /* set by the caller, for logs */
} ofdm_watch_t;

void ofdm_watch_init(ofdm_watch_t *w);
static inline float ofdm_slot_off_mhz(unsigned slot)
{
    return (float)(OFDM_SLOT_FIRST_KHZ + (int)slot * OFDM_SLOT_STEP_KHZ) / 1000.0f;
}

/* Choose up to `max` slots for the next snapshot, least recently scanned
 * first: alternately among slots with wide activity (duty[] >= min_duty)
 * and among all slots, so a weak link under the spectrum threshold is
 * still visited while busy slots keep being watched. */
unsigned ofdm_watch_pick(const ofdm_watch_t *w, const float duty[OFDM_SLOTS], float min_duty, unsigned max,
                         unsigned *slots);

void ofdm_watch_update(ofdm_watch_t *w, unsigned slot, const ofdm_result_t *r, uint64_t now_us);

/* The running mean a kind needs in a slot (ofdm_detect.c). */
float ofdm_watch_threshold(ofdm_kind_t k);

/* The slot shows this kind now (enough scans, mean above threshold, held). */
bool ofdm_watch_on(const ofdm_watch_t *w, unsigned slot, ofdm_kind_t k, uint64_t now_us);

#ifdef __cplusplus
}
#endif
#endif /* OFDM_DETECT_H */
