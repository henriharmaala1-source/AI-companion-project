/* ui.h - Cardputer LCD for ELRS watch mode (implemented in ui.cpp, M5GFX). */
#ifndef UI_H
#define UI_H

#include <stdbool.h>
#include <stdint.h>
#include "elrs_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One line in the list of other things seen (targets.c builds them). */
#define UI_MAX_TARGETS  8u
#define UI_TARGET_TEXT  39u     /* 38 characters fit beside the colour mark */
typedef enum {
    UI_LVL_DIM = 0,    /* everyday (Wi-Fi) */
    UI_LVL_INFO,       /* harmless but worth knowing (microwave oven) */
    UI_LVL_NOTE,       /* unidentified activity (other hoppers, wide signals) */
    UI_LVL_WARN,       /* a radio link worth a look (DJI-like video, fixed LoRa) */
} ui_level_t;
typedef struct {
    uint8_t kind;      /* target_kind_t */
    uint8_t level;     /* ui_level_t */
    float   mhz;       /* RF, 0 if unknown */
    char    text[UI_TARGET_TEXT];
} ui_target_t;

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
    bool     chirp_fixed;  /* LoRa chirps, but all on one channel: not ELRS-like */
    unsigned chirp_clusters; /* separate channel groups among the recent chirp hits */
    unsigned rx_filter_mhz;   /* analog filter width in force, 0 = unknown */
    bool     rx_filter_wide;  /* 'w' set the widest filter */
    float    chirp_clip;      /* last snapshot: samples at the ADC rail (0..1) */
    const char *sound;        /* "normal", "quiet" or "off" */
    uint8_t  chirp_sf;     /* SF of the last LoRa hit, 0 = none */
    uint16_t chirp_hits, chirp_snaps; /* in the evidence window */
    uint64_t chirp_last_us;
    float    chirp_snr_db;    /* smoothed SNR of recent hits: closer transmitter = higher */
    float    chirp_delta_db;  /* SNR change since this detection began */
    float    chirp_dbfs;      /* last hit, raw level re ADC full scale */
    uint8_t  chirp_agc;       /* gain index for that level */
    uint64_t last_detected_us;  /* last time the screen said DETECTED, 0 = never */
    const ui_target_t *targets; /* other things seen, most important first */
    unsigned n_targets;
} ui_info_t;

void ui_init(void);
void ui_message(const char *line1, const char *line2);   /* boot / fatal text */
void ui_update(const elrs_status_t *st, const ui_info_t *info);
void ui_alert(void);                                      /* red flash */

#ifdef __cplusplus
}
#endif
#endif /* UI_H */
