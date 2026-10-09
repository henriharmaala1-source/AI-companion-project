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
} ui_info_t;

void ui_init(void);
void ui_message(const char *line1, const char *line2);   /* boot / fatal text */
void ui_update(const elrs_status_t *st, const ui_info_t *info);
void ui_alert(void);                                      /* red flash */

#ifdef __cplusplus
}
#endif
#endif /* UI_H */
