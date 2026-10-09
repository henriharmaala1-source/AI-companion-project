/* spc1.c - see spc1.h. */
#include "spc1.h"

#include <string.h>

static const uint8_t k_magic[4] = { 'S', 'P', 'C', '1' };

/* Bitwise CRC-32 (reflected, poly 0xEDB88320), identical to zlib.crc32(),
 * which is what esp-sdr's own host tools check against. A table would be
 * faster; at a few hundred frames per second this is not the bottleneck. */
uint32_t spc1_crc32(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t rd64(const uint8_t *p) { return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32); }

void spc1_init(spc1_decoder_t *d, spc1_frame_fn on_frame, void *ctx)
{
    memset(d, 0, sizeof(*d));
    d->on_frame = on_frame;
    d->ctx = ctx;
}

/* Drop the first byte of the buffer, then skip ahead to the next place a
 * magic could start (a partial match at the very end is kept). Always removes
 * at least one byte, so the processing loop cannot spin. */
static void resync(spc1_decoder_t *d)
{
    size_t start = 1;
    while (start < d->have) {
        size_t cmp = d->have - start < 4 ? d->have - start : 4;
        if (memcmp(d->buf + start, k_magic, cmp) == 0) {
            break;
        }
        start++;
    }
    d->resyncs += (uint32_t)start;
    memmove(d->buf, d->buf + start, d->have - start);
    d->have -= start;
    d->need = 0;
}

/* Header complete: validate the fields that decide the frame length before
 * believing them. A bad log2 is treated as corruption, not as a big frame. */
static bool header_ok(const spc1_decoder_t *d)
{
    const uint8_t log2n = d->buf[26];
    return log2n >= SPC1_MIN_LOG2 && log2n <= SPC1_MAX_LOG2 && d->buf[27] != 0;
}

static void deliver(spc1_decoder_t *d)
{
    const uint8_t *b = d->buf;
    spc1_frame_t f = {
        .sequence = rd32(b + 4), .pair_index = rd64(b + 8), .pairs = rd32(b + 16),
        .ffts = rd16(b + 20), .flags = b[22], .gain = b[23], .drops = rd16(b + 24),
        .bins = (uint16_t)(1u << b[26]), .db_step = b[27], .codes = b + SPC1_HEADER_LEN,
    };
    d->frames_ok++;
    if (d->on_frame) {
        d->on_frame(&f, d->ctx);
    }
}

/* Consume as much of the buffer as possible. Every path either returns
 * (waiting for more bytes) or removes at least one byte. */
static void process(spc1_decoder_t *d)
{
    for (;;) {
        if (d->need == 0) {
            if (d->have >= 4 && memcmp(d->buf, k_magic, 4) != 0) {
                resync(d);
                continue;
            }
            if (d->have < SPC1_HEADER_LEN) {
                return;
            }
            if (!header_ok(d)) {
                resync(d);
                continue;
            }
            d->need = SPC1_HEADER_LEN + ((size_t)1u << d->buf[26]) + 4u;
        }
        if (d->have < d->need) {
            return;
        }
        const size_t body = d->need - 4u;
        if (spc1_crc32(0, d->buf, body) != rd32(d->buf + body)) {
            d->crc_bad++;
            resync(d);
            continue;
        }
        deliver(d);
        memmove(d->buf, d->buf + d->need, d->have - d->need);
        d->have -= d->need;
        d->need = 0;
    }
}

void spc1_feed(spc1_decoder_t *d, const uint8_t *data, size_t len)
{
    size_t i = 0;
    while (i < len) {
        size_t take = sizeof(d->buf) - d->have;
        if (take > len - i) {
            take = len - i;
        }
        memcpy(d->buf + d->have, data + i, take);
        d->have += take;
        i += take;
        /* The buffer holds one maximum-size frame, so a full buffer always
         * contains either a complete frame or something to resync past. */
        process(d);
    }
}
