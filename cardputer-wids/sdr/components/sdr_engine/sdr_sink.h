/* sdr_sink.h - RAM byte queue standing in for esp-sdr's USB FIFO. Private. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool   sdr_sink_init(size_t bytes);

/* Before each capture run: accept at most one `frame_bytes` frame per
 * `frame_us` from now on. On the S3, esp-sdr closes a frame only once the
 * previous one has left its queue, so the rate this sink accepts bytes at IS
 * the frame length. Without pacing the sink fills in a few milliseconds and
 * the rest of the slice collapses into one long frame. See BUGLOG.md. */
void   sdr_sink_pace(uint32_t frame_bytes, uint32_t frame_us);

int    sdr_sink_write(const uint8_t *p, unsigned n);   /* called with IRQs masked */
size_t sdr_sink_read(uint8_t *dst, size_t cap);
size_t sdr_sink_used(void);
size_t sdr_sink_capacity(void);
bool   sdr_sink_stop_requested(void);
void   sdr_sink_take_stop(void);
