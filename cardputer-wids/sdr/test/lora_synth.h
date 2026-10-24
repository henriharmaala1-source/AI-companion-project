/*
 * lora_synth.h - synthetic 80 MS/s I/Q for testing lora_detect.
 *
 * A MODEL, built from the published waveforms:
 *   LoRa   SX1280 style: N preamble up-chirps, 2 sync-word symbols, 2.25
 *          down-chirps, then data symbols (cyclically shifted up-chirps).
 *          Inverted IQ conjugates everything (ExpressLRS invertIQ).
 *   GFSK   Bluetooth-like: 1 Mbit/s, +-160 kHz deviation, smoothed.
 *   OFDM   Wi-Fi-like: 52 subcarriers at 312.5 kHz, 4 us symbols, QPSK.
 *   CW     a pure carrier.
 * Output is quantised to the S3's signed 10-bit I/Q, clipped like an ADC,
 * and packed into esp-sdr ring words (I bits 0..9, Q bits 10..19).
 */
#ifndef LORA_SYNTH_H
#define LORA_SYNTH_H

#include <stdbool.h>
#include <stdint.h>

#define SYNTH_FS 80000000.0

typedef struct {
    double *i, *q;       /* accumulated signal, n samples */
    uint32_t n;
    uint32_t rng;
} synth_t;

void synth_init(synth_t *s, uint32_t n, uint32_t seed);
void synth_free(synth_t *s);

/* amp: peak amplitude in ADC units (full scale is 511). start_s: time of
 * the packet start relative to the first sample (negative = the snapshot
 * begins inside the packet). */
void synth_lora(synth_t *s, unsigned sf, bool invert_iq, double f_hz, double amp, double start_s,
                unsigned payload_symbols);
void synth_gfsk(synth_t *s, double f_hz, double amp);
void synth_ofdm(synth_t *s, double f_hz, double amp);
void synth_cw(synth_t *s, double f_hz, double amp);
void synth_noise(synth_t *s, double sigma);
/* Low-pass everything so far to bw_hz (two-sided), then add ADC noise. */
void synth_analog_filter(synth_t *s, double bw_hz, double adc_sigma);

/* Gain index stamped into bits 20-27 of every packed word (default 40). */
extern uint8_t synth_gain_index;

/* Quantise and pack into words; returns clipped-sample count. */
uint32_t synth_pack(const synth_t *s, uint32_t *words);

#endif /* LORA_SYNTH_H */
