/*
 * sdr_engine.h - esp-sdr's ESP32-S3 receive engine, without its USB protocol.
 *
 * RECEIVE ONLY. This wraps the upstream continuous-capture engine
 * (third_party/esp-sdr, GPL-3.0-or-later) so the Cardputer can consume
 * spectrum frames itself instead of streaming them to a PC.
 *
 * The one rule a caller must respect: sdr_engine_run_slice() masks interrupts
 * on core 0 for its whole duration. No FreeRTOS tick, no SPI DMA completion,
 * no keyboard, no USB while it runs. Draw the screen and poll keys BETWEEN
 * slices. That is upstream's design, not a bug here.
 *
 * Only esp-sdr's S3 path is supported, and only when built with the ESP-IDF
 * commit upstream pins (see the project CMakeLists.txt): the engine relies on
 * private PHY ABIs that change between IDF versions.
 */
#ifndef SDR_ENGINE_H
#define SDR_ENGINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* esp-sdr rate codes */
#define SDR_RATE_80MSPS 0u
#define SDR_RATE_40MSPS 1u
#define SDR_RATE_16MSPS 6u

typedef struct {
    unsigned rate_code;        /* SDR_RATE_* */
    unsigned nfft;             /* 256, 512, 1024 or 2048 */
    unsigned units_per_frame;  /* ring units merged per frame (frame duration) */
    bool     max_hold;         /* per-bin maximum instead of mean */
} sdr_spec_cfg_t;

typedef struct {
    uint32_t status;     /* 0 = ok, else upstream ring_status_t */
    uint32_t detail;
    uint32_t frames, drops, abandoned, ffts;
    uint64_t pairs, elapsed_us;
    size_t   sink_used;  /* bytes waiting after the slice (sizing aid) */
} sdr_slice_result_t;

/* Bring the radio up in receive-only NULL mode, tune, start the core-1 worker.
 * Needs NVS initialised. Returns ESP_FAIL if the FFT/worker could not start. */
esp_err_t sdr_engine_init(unsigned center_mhz);

/* Capture spectra for `duration_ms` (interrupts masked meanwhile). Frames go
 * to an internal RAM sink; read them with sdr_engine_read() afterwards. */
void sdr_engine_run_slice(const sdr_spec_cfg_t *cfg, uint32_t duration_ms, sdr_slice_result_t *out);

/* Drain captured bytes (raw SPC1 stream). Returns bytes copied. */
size_t sdr_engine_read(uint8_t *dst, size_t cap);

size_t sdr_engine_sink_capacity(void);
unsigned sdr_engine_rate_hz(unsigned rate_code);
bool sdr_engine_dual_core(void);
unsigned sdr_engine_center_mhz(void);

#ifdef __cplusplus
}
#endif
#endif /* SDR_ENGINE_H */
