/*
 * wids_platform.h - what silicon are we actually running on?
 *
 * The Cardputer ships with different ESP32-S3-WROOM-1 variants. Only the
 * N16R8 has 8 MB of PSRAM; other markings have 2 MB or none at all. The AP
 * baseline table wants PSRAM, so we probe for it at boot, report what we
 * found, and let the caller degrade gracefully rather than assuming.
 *
 * These numbers are MEASURED, not read off the module can. Compare them
 * against the marking printed on the shield - if they disagree, believe the
 * can and suspect the board-menu flash/PSRAM settings.
 */
#ifndef WIDS_PLATFORM_H
#define WIDS_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t chip_revision;   /* format MXX: wafer major/minor */
    uint8_t  cores;
    bool     has_wifi_bgn;    /* 2.4 GHz. There is no 5 GHz radio on this part. */
    bool     has_ble;

    bool     psram_present;   /* usable PSRAM heap was found */
    size_t   psram_bytes;     /* total SPIRAM-capable heap, bytes */
    uint32_t flash_bytes;     /* as configured for the default flash chip */

    /* Best-effort guess at the module marking from flash+PSRAM sizes, e.g.
     * "N16R8". Inferred, never authoritative - see the header comment. */
    char     inferred_variant[12];
} wids_platform_info_t;

/* Fills `out`. Safe to call once early in setup(). */
void wids_platform_probe(wids_platform_info_t *out);

/* Emits the probe result as one NDJSON "boot" record. */
void wids_platform_log(const wids_platform_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* WIDS_PLATFORM_H */
