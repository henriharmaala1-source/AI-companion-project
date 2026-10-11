/* ui.h - Cardputer LCD for ELRS watch mode (implemented in ui.cpp, M5GFX). */
#ifndef UI_H
#define UI_H

#include <stdbool.h>
#include <stdint.h>
#include "elrs_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned center_mhz;
    unsigned span_mhz;
    uint8_t  threshold_db;
    bool     invert;
    uint32_t slice_ms;
    uint32_t drops;        /* frames the engine dropped, cumulative */
    uint32_t crc_bad;      /* frames rejected by the decoder, cumulative */
    uint64_t uptime_us;
    bool     details;      /* false: big DETECTED / NOT DETECTED view */
    int      gain;         /* -1 = hardware AGC, else fixed PHY gain index */
    bool     chirp_active; /* LoRa chirp verdict holds */
    bool     chirp_weak;   /* ...on agreeing weak snapshots only */
    uint8_t  chirp_sf;     /* SF of the last LoRa hit, 0 = none */
    uint16_t chirp_hits, chirp_snaps; /* in the evidence window */
    uint64_t chirp_last_us;
    float    chirp_snr_db;    /* smoothed SNR of recent hits: closer transmitter = higher */
    float    chirp_delta_db;  /* SNR change since this detection began */
    float    chirp_dbfs;      /* last hit, raw level re ADC full scale */
    uint8_t  chirp_agc;       /* gain index for that level */
    uint64_t last_detected_us;  /* last time the screen said DETECTED, 0 = never */
} ui_info_t;

void ui_init(void);
void ui_message(const char *line1, const char *line2);   /* boot / fatal text */
void ui_update(const elrs_status_t *st, const ui_info_t *info);
void ui_alert(void);                                      /* red flash */

#ifdef __cplusplus
}
#endif
#endif /* UI_H */
