/*
 * sdr_sink.c - where esp-sdr's spectrum bytes land instead of USB.
 *
 * The engine calls sdr_sink_write() from its capture loop with interrupts
 * masked and hard bank-rotation deadlines, so the write path is a bounded
 * byte copy into a ring buffer, lives in IRAM (no flash-cache stall), never
 * blocks, and accepts only what it allows. Returning less than `n` is how the
 * engine learns the "host" is behind: the unsent frame stays in its queue and
 * new FFTs keep merging into the next frame until it has gone.
 *
 * That last fact is why the sink is PACED. esp-sdr's S3 path has no frame
 * timer: it emits a frame as soon as the previous one has drained. A sink
 * that takes everything at once is drained instantly, so frames would be one
 * ring unit (~0.15 ms) long, the buffer would be full ~7 ms into a 100 ms
 * slice, and the remaining 93 ms would merge into one useless frame. Letting
 * bytes in at one frame per `frame_us` makes every frame ~frame_us long.
 *
 * Producer and consumer never overlap in time: the engine writes during a
 * slice, the app reads between slices, so no lock is needed.
 */
#include "sdr_sink.h"

#include <stdint.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

static uint8_t *s_buf;
static size_t   s_cap;
static DRAM_ATTR volatile size_t s_head, s_tail; /* free-running counters */
static DRAM_ATTR volatile bool   s_stop;

/* pacing: bytes accepted since s_pace_t0 may not exceed elapsed * rate */
static DRAM_ATTR int64_t  s_pace_t0;
static DRAM_ATTR uint32_t s_pace_bytes, s_pace_us;
static DRAM_ATTR uint32_t s_paced_in;

/* Longest stretch the pace is computed over: a slice plus upstream's 500 ms
 * drain fits easily, and elapsed * 2080 bytes (2048 bins) stays in 32 bits. */
#define PACE_MAX_US 1000000u

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

void sdr_sink_pace(uint32_t frame_bytes, uint32_t frame_us)
{
    s_pace_bytes = frame_bytes;
    s_pace_us = frame_us;
    s_paced_in = 0;
    s_pace_t0 = esp_timer_get_time();
}

/* Bytes the pace allows right now. 32-bit arithmetic only: the S3 divides
 * 32-bit integers in hardware, 64-bit division is a library call that may sit
 * in flash. esp_timer_get_time() is safe here: the upstream capture loop calls
 * it in the same masked-interrupt context. */
static IRAM_ATTR size_t pace_allowance(void)
{
    if (s_pace_us == 0) {
        return SIZE_MAX;   /* not paced */
    }
    int64_t elapsed = esp_timer_get_time() - s_pace_t0;
    if (elapsed < 0) {
        elapsed = 0;
    } else if (elapsed > PACE_MAX_US) {
        elapsed = PACE_MAX_US;
    }
    const uint32_t budget = (uint32_t)elapsed * s_pace_bytes / s_pace_us;
    return budget > s_paced_in ? budget - s_paced_in : 0;
}

IRAM_ATTR int sdr_sink_write(const uint8_t *p, unsigned n)
{
    size_t room = s_cap - (s_head - s_tail);
    const size_t allowed = pace_allowance();
    if (allowed < room) {
        room = allowed;
    }
    if (n > room) {
        n = (unsigned)room;
    }
    for (unsigned i = 0; i < n; i++) {   /* plain loop: memcpy may sit in flash */
        s_buf[(s_head + i) % s_cap] = p[i];
    }
    s_head += n;
    s_paced_in += n;
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
