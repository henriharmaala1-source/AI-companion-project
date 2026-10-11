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
    unsigned frame_us;         /* target frame length; see sdr_sink_pace(). The
                                * S3 engine ignores units_per_frame: frames are
                                * as long as the output takes to drain. */
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
 * to an internal RAM sink; read them with sdr_engine_read() afterwards. The
 * sink must hold the whole slice: duration_ms * 1000 / frame_us frames. */
void sdr_engine_run_slice(const sdr_spec_cfg_t *cfg, uint32_t duration_ms, sdr_slice_result_t *out);

/* RAM the engine's output sink takes from the heap. It must hold one whole
 * capture slice of frames; app_main.c checks that at compile time. */
#define SDR_SINK_BYTES (16u * 1024u)

/* Bytes in one SPC1 frame of `nfft` bins: header + codes + CRC. */
#define SDR_FRAME_BYTES(nfft) (28u + (nfft) + 4u)

/* Raw I/Q snapshot: up to 3 consecutive ring units (~0.46 ms at 80 MS/s),
 * gapless, left in the radio's ring memory. Valid until the next slice or
 * capture. Word format: signed 10-bit I in bits 0..9, Q in bits 10..19. */
#define SDR_CAPTURE_MAX_SEGS 6u   /* each unit may wrap around its bank once */
typedef struct {
    uint32_t status;              /* 0 = ok, else upstream ring_status_t */
    uint32_t fs_hz;
    unsigned nseg;
    const uint32_t *seg[SDR_CAPTURE_MAX_SEGS];
    uint32_t len[SDR_CAPTURE_MAX_SEGS];
} sdr_capture_t;
void sdr_engine_capture(unsigned rate_code, unsigned units, sdr_capture_t *out);

/* The output sink's RAM, lent as scratch while it is empty (after a slice
 * has been drained, before the next one). NULL while it holds data. */
void *sdr_engine_scratch(size_t *bytes);

/* Drain captured bytes (raw SPC1 stream). Returns bytes copied. */
size_t sdr_engine_read(uint8_t *dst, size_t cap);

/* Receiver gain, between slices only. code < 0: the radio's hardware AGC
 * (default). code >= 0: a fixed PHY gain index (not dB), clamped to the
 * calibrated maximum. Returns the code in force, or -1 for AGC. A fixed gain
 * avoids AGC jumps when a strong burst arrives; upstream does the same
 * ("GAIN MANUAL n"). */
int sdr_engine_set_gain(int code);
unsigned sdr_engine_gain_max(void);

size_t sdr_engine_sink_capacity(void);
unsigned sdr_engine_rate_hz(unsigned rate_code);
bool sdr_engine_dual_core(void);
unsigned sdr_engine_center_mhz(void);

#ifdef __cplusplus
}
#endif
#endif /* SDR_ENGINE_H */
