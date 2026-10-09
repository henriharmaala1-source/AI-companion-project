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
 *
 * Things it does NOT model, so a pass here is not proof on hardware: the
 * Cardputer's antenna and AGC, gain changes, spurs, a strong signal
 * desensitising the receiver, other SX1280 links (TBS Tracer, ImmersionRC
 * Ghost) whose channel plans are not public, microwave ovens.
 */
#ifndef RF_SCENE_H
#define RF_SCENE_H

#include <stdbool.h>
#include <stdint.h>

#define RF_MAX_EMITTERS 8

typedef enum { RF_ELRS, RF_BT_CLASSIC, RF_BLE, RF_WIFI } rf_kind_t;

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
    /* RF_WIFI */
    unsigned wifi_channel;  /* 1..13 */
    float    duty;          /* fraction of time carrying traffic, 0..1 */
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
