/*
 * wids_platform.c - boot-time silicon probe. See wids_platform.h.
 */
#include "wids_platform.h"
#include "wids_log.h"

#include <stdio.h>
#include <string.h>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"

/*
 * Map measured flash+PSRAM sizes onto the usual ESP32-S3-WROOM-1 markings.
 *
 * Thresholds rather than exact equality: the SPIRAM heap is always a little
 * smaller than the raw chip because the allocator reserves some, and flash
 * size reflects the configured value from the board menu.
 */
static void infer_variant(uint32_t flash_bytes, size_t psram_bytes,
                          char *out, size_t out_cap)
{
    const unsigned flash_mb = (unsigned)(flash_bytes / (1024U * 1024U));

    unsigned psram_mb = 0;
    if (psram_bytes >= 4U * 1024U * 1024U) {
        psram_mb = 8; /* R8 */
    } else if (psram_bytes >= 1U * 1024U * 1024U) {
        psram_mb = 2; /* R2 */
    }

    if (flash_mb == 0) {
        snprintf(out, out_cap, "unknown");
    } else if (psram_mb == 0) {
        snprintf(out, out_cap, "N%u", flash_mb);
    } else {
        snprintf(out, out_cap, "N%uR%u", flash_mb, psram_mb);
    }
}

void wids_platform_probe(wids_platform_info_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    esp_chip_info_t chip;
    esp_chip_info(&chip);
    out->chip_revision = chip.revision;
    out->cores         = chip.cores;
    out->has_wifi_bgn  = (chip.features & CHIP_FEATURE_WIFI_BGN) != 0;
    out->has_ble       = (chip.features & CHIP_FEATURE_BLE) != 0;

    /*
     * heap_caps_get_total_size(MALLOC_CAP_SPIRAM) asks the allocator what it
     * actually has, which is the number that matters to us. esp_psram_get_size()
     * would report the raw chip size instead; we care about usable bytes.
     *
     * Zero here on an N16R8 almost always means PSRAM is switched off in the
     * Arduino Tools menu - it must be set to OPI PSRAM for this module.
     */
    out->psram_bytes   = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    out->psram_present = (out->psram_bytes > 0);

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) { /* NULL = default chip */
        out->flash_bytes = flash_size;
    }

    infer_variant(out->flash_bytes, out->psram_bytes,
                  out->inferred_variant, sizeof(out->inferred_variant));
}

void wids_platform_log(const wids_platform_info_t *info)
{
    if (info == NULL) {
        return;
    }

    /*
     * "band":"2.4GHz-only" is not decoration. Every consumer of this log must
     * be able to see that a quiet result covers 2.4 GHz alone.
     */
    wids_log_event("boot",
                   "\"chip\":\"ESP32-S3\",\"rev\":%u,\"cores\":%u,"
                   "\"psram\":%s,\"psram_bytes\":%u,\"flash_bytes\":%u,"
                   "\"variant_guess\":\"%s\",\"ble\":%s,\"band\":\"2.4GHz-only\"",
                   (unsigned)info->chip_revision,
                   (unsigned)info->cores,
                   info->psram_present ? "true" : "false",
                   (unsigned)info->psram_bytes,
                   (unsigned)info->flash_bytes,
                   info->inferred_variant,
                   info->has_ble ? "true" : "false");
}
