/*
 * spc1.h - streaming decoder for esp-sdr "SPC1" spectrum frames.
 *
 * Wire layout (esp-sdr docs/spectrum.md, all little-endian):
 *   0   u32  magic "SPC1"
 *   4   u32  frame sequence
 *   8   u64  index of the first I/Q pair (gapless sample clock)
 *   16  u32  pairs spanned by the frame
 *   20  u16  FFTs merged
 *   22  u8   flags (bit0 max-hold, bit1 work abandoned, bit2 earlier drop)
 *   23  u8   gain metadata
 *   24  u16  cumulative drops
 *   26  u8   log2(bins)
 *   27  u8   dB step (2 -> one code = 0.5 dB)
 *   28  N    power codes, natural FFT order (bin 0 = LO, then +f, then -f)
 *   28+N u32 zlib CRC-32 of everything before it
 *
 * The bytes come out of the capture engine in arbitrary-sized chunks, so the
 * decoder is a byte-fed state machine. Nothing here trusts a length field
 * before checking it, and a frame is only delivered once its CRC matches.
 *
 * Pure C, no ESP-IDF dependency, so it is unit-tested on the host.
 */
#ifndef SPC1_H
#define SPC1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPC1_HEADER_LEN   28u
#define SPC1_MAX_BINS     2048u
#define SPC1_MIN_LOG2     8u      /* 256 bins */
#define SPC1_MAX_LOG2     11u     /* 2048 bins */

typedef struct {
    uint32_t sequence;
    uint64_t pair_index;
    uint32_t pairs;
    uint16_t ffts;
    uint8_t  flags;
    uint8_t  gain;
    uint16_t drops;
    uint16_t bins;      /* 1 << log2 */
    uint8_t  db_step;
    const uint8_t *codes; /* valid only during the callback */
} spc1_frame_t;

typedef void (*spc1_frame_fn)(const spc1_frame_t *frame, void *ctx);

typedef struct {
    uint8_t  buf[SPC1_HEADER_LEN + SPC1_MAX_BINS + 4u];
    size_t   have;      /* bytes collected for the current frame */
    size_t   need;      /* total frame length once the header is known, else 0 */
    uint32_t frames_ok;
    uint32_t crc_bad;
    uint32_t resyncs;   /* bytes skipped while hunting for a magic */
    spc1_frame_fn on_frame;
    void    *ctx;
} spc1_decoder_t;

void spc1_init(spc1_decoder_t *d, spc1_frame_fn on_frame, void *ctx);
void spc1_feed(spc1_decoder_t *d, const uint8_t *data, size_t len);

/* zlib-compatible CRC-32, exposed for tests. */
uint32_t spc1_crc32(uint32_t crc, const uint8_t *p, size_t n);

#ifdef __cplusplus
}
#endif
#endif /* SPC1_H */
