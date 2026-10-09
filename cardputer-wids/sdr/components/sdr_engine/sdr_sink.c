/*
 * sdr_sink.c - where esp-sdr's spectrum bytes land instead of USB.
 *
 * The engine calls sdr_sink_write() from its capture loop with interrupts
 * masked and hard bank-rotation deadlines, so the write path is a bounded
 * memcpy into a ring buffer, lives in IRAM (no flash-cache stall), never
 * blocks, and accepts only what fits. Returning less than `n` is how the
 * engine learns the "host" is behind: it then drops whole frames and counts
 * them, exactly as it would with a slow USB reader.
 *
 * Producer and consumer never overlap in time: the engine writes during a
 * slice, the app reads between slices, so no lock is needed.
 */
#include "sdr_sink.h"

#include <string.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"

static uint8_t *s_buf;
static size_t   s_cap;
static DRAM_ATTR volatile size_t s_head, s_tail; /* free-running counters */
static DRAM_ATTR volatile bool   s_stop;

bool sdr_sink_init(size_t bytes)
{
    if (s_buf) {
        return true;
    }
    /* Internal RAM only: the engine may write while the flash cache is busy. */
    s_buf = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    s_cap = s_buf ? bytes : 0;
    s_head = s_tail = 0;
    return s_buf != NULL;
}

IRAM_ATTR int sdr_sink_write(const uint8_t *p, unsigned n)
{
    const size_t free_bytes = s_cap - (s_head - s_tail);
    if (n > free_bytes) {
        n = (unsigned)free_bytes;
    }
    for (unsigned i = 0; i < n; i++) {   /* plain loop: memcpy may sit in flash */
        s_buf[(s_head + i) % s_cap] = p[i];
    }
    s_head += n;
    return (int)n;
}

size_t sdr_sink_read(uint8_t *dst, size_t cap)
{
    size_t n = 0;
    while (n < cap && s_tail != s_head) {
        dst[n++] = s_buf[s_tail % s_cap];
        s_tail++;
    }
    return n;
}

size_t sdr_sink_used(void) { return s_head - s_tail; }
size_t sdr_sink_capacity(void) { return s_cap; }
IRAM_ATTR bool sdr_sink_stop_requested(void) { return s_stop; }
IRAM_ATTR void sdr_sink_take_stop(void) { s_stop = false; }
