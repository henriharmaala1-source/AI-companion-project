/*
 * rf_scene.h - a model of the 2.4 GHz air around the Cardputer.
 *
 * A scene is a list of transmitters, each switched on for part of the run.
 * rf_scene_render() answers one question: "what would a max-hold spectrum of
 * the window [t0, t1) look like?", which is exactly what one esp-sdr capture
 * unit produces. Transmitter behaviour comes from the published specs:
 *
 *   ELRS          2400.4 + k MHz, 80 channels, sync channel 40, 240-hop
 *                 sequence of three 80-channel blocks (ExpressLRS FHSS.cpp),
 *                 hop every N packets. Rates as in test/test_elrs_detect.c.
 *   BT Classic    2402 + k MHz, 79 channels, 625 us slots, AFH (Core Spec).
 *                 Modelled as audio streaming: a 5-slot packet then a 1-slot
 *                 reply, each on a new channel.
 *   BLE           advertising on 2402/2426/2480 MHz, data channels 2404..2478
 *                 (2 MHz grid) with channel selection algorithm #1.
 *   Wi-Fi         one 20 MHz channel: beacons every 102.4 ms plus traffic.
 *   Microwave     a household oven: on for ~45 % of each mains cycle, sweeping
 *                 a few MHz across the on-time (NIST oven recordings; IEEE
 *                 802.11-98/240), ~1.5 MHz wide at any instant (an assumption).
 *   DJI-like      continuous LTE-like OFDM, ~9 MHz wide (600 x 15 kHz or
 *                 300 x 30 kHz subcarriers), like an OcuSync video downlink;
 *                 the duty models its small gaps.
 *
 * Things it does NOT model, so a pass here is not proof on hardware: the
 * Cardputer's antenna and AGC, gain changes, spurs, a strong signal
 * desensitising the receiver, other SX1280 links (TBS Tracer, ImmersionRC
 * Ghost) whose channel plans are not public, inverter microwave ovens, DJI's
 * frequency changes and its DroneID bursts.
 */
#ifndef RF_SCENE_H
#define RF_SCENE_H

#include <stdbool.h>
#include <stdint.h>

#define RF_MAX_EMITTERS 8

typedef enum { RF_ELRS, RF_BT_CLASSIC, RF_BLE, RF_WIFI, RF_MICROWAVE, RF_DJI } rf_kind_t;

typedef struct {
    rf_kind_t kind;
    float  level_db;        /* in-band power per FFT bin, dB above the noise mean */
    double on_s, off_s;     /* transmitting during [on_s, off_s) */
    /* RF_ELRS */
    float    packet_ms;     /* packet interval */
    float    airtime_ms;    /* time on air per packet */
    unsigned packets_per_hop;
    float    bw_khz;        /* occupied bandwidth */
    uint32_t seed;          /* stands in for the binding phrase */
    unsigned sf;            /* LoRa spreading factor, 0 = FLRC (no chirp) */
    bool     invert_iq;     /* ExpressLRS invertIQ = UID[5] & 1 */
    unsigned fixed_chan;    /* 0 = hops like ELRS; else stays on grid channel fixed_chan - 1
                             * (stands in for fixed-channel LoRa: mesh, IoT, ELRS binding?) */
    float    ramp_db_per_s; /* level change over time: a transmitter approaching (+) or leaving (-) */
    /* RF_WIFI */
    unsigned wifi_channel;  /* 1..13 */
    float    duty;          /* fraction of time carrying traffic, 0..1 (RF_DJI too) */
    /* RF_MICROWAVE, RF_DJI */
    float    center_mhz;
    float    sweep_mhz;     /* oven: swept across each on-time */
    unsigned mains_hz;      /* oven: 50 (Finland) or 60 */
    float    scs_khz;       /* DJI-like: subcarrier spacing, 15 or 30 */
} rf_emitter_t;

typedef struct {
    rf_emitter_t em[RF_MAX_EMITTERS];
    unsigned     n;
    uint8_t      elrs_seq[RF_MAX_EMITTERS][240];
} rf_scene_t;

typedef struct {
    uint32_t center_khz;
    uint32_t fs_hz;
    unsigned bins;
    unsigned ffts;          /* FFTs merged into the window (max-hold) */
    bool     inverted;      /* the hardware delivers a mirrored spectrum */
} rf_view_t;

/* Oven: on-share of each mains cycle (a half-wave doubler conducts for a
 * bit less than half the cycle). */
#define RF_MW_ON_FRAC 0.45

/* Where an oven radiates at t_us, or false while it is off this cycle. */
static inline bool rf_mw_freq(const rf_emitter_t *e, double t_us, double *mhz)
{
    const double period = 1e6 / (e->mains_hz ? e->mains_hz : 50u);
    const double ph = (t_us - e->on_s * 1e6) / period;
    const double frac = ph - (double)(long long)ph;
    if (frac >= RF_MW_ON_FRAC) {
        return false;
    }
    *mhz = e->center_mhz - e->sweep_mhz / 2.0 + e->sweep_mhz * frac / RF_MW_ON_FRAC;
    return true;
}

/* Level at time t_us (on_s is where the ramp starts). */
static inline float rf_level_at(const rf_emitter_t *e, double t_us)
{
    const double since = t_us / 1e6 - e->on_s;
    return e->level_db + (float)(since > 0 ? since : 0) * e->ramp_db_per_s;
}

void rf_scene_init(rf_scene_t *s);
/* Returns the index of the new emitter, or -1 when the scene is full. */
int  rf_scene_add(rf_scene_t *s, const rf_emitter_t *e);

/* esp-sdr power codes (0.5 dB steps, natural FFT order) for one max-hold
 * window [t0_us, t1_us). `codes` holds view->bins bytes. */
void rf_scene_render(const rf_scene_t *s, const rf_view_t *view, double t0_us, double t1_us,
                     uint8_t *codes);

/* The noise mean, as a power code, so tests can reason about thresholds. */
#define RF_NOISE_CODE 50

#endif /* RF_SCENE_H */
