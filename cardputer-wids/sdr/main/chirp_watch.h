/*
 * chirp_watch.h - the LoRa chirp side of ELRS watch.
 *
 * Between spectrum slices, takes a raw I/Q snapshot (~0.46 ms), runs
 * lora_scan() on it in the empty sink's RAM, and keeps a short evidence
 * window. PRESENCE ONLY: no symbols, packets or identities are recovered.
 */
#ifndef CHIRP_WATCH_H
#define CHIRP_WATCH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool     active;          /* LoRa verdict holds now */
    uint8_t  sf;              /* SF of the last positive snapshot, 0 if none */
    uint16_t hits, snaps;     /* positive / all snapshots in the evidence window */
    uint64_t last_hit_us;     /* 0 = never */
    float    snr_db;          /* last hit: dB over the noise (proximity, AGC-proof) */
    float    level_dbfs;      /* last hit: dB re ADC full scale (needs the same AGC index) */
    uint8_t  agc;             /* last hit: receiver gain index */
} chirp_status_t;

void chirp_watch_init(unsigned rate_code);
/* Call once per main-loop pass, after the slice has been drained. */
void chirp_watch_step(uint64_t now_us);
void chirp_watch_status(uint64_t now_us, chirp_status_t *out);
/* NDJSON summary ("lora"), meant to be called every couple of seconds. */
void chirp_watch_log(uint64_t now_us);
/* Log the next snapshot's baseband and some raw words ("bb", "raw"). */
void chirp_watch_request_dump(void);

/* ExpressLRS 2.4 GHz rate for a LoRa SF (src/src/common.cpp). */
const char *chirp_elrs_rate(uint8_t sf);

#ifdef __cplusplus
}
#endif

#endif /* CHIRP_WATCH_H */
