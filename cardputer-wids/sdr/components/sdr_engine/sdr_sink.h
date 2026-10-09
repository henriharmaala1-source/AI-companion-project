/* sdr_sink.h - RAM byte queue standing in for esp-sdr's USB FIFO. Private. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool   sdr_sink_init(size_t bytes);
int    sdr_sink_write(const uint8_t *p, unsigned n);   /* called with IRQs masked */
size_t sdr_sink_read(uint8_t *dst, size_t cap);
size_t sdr_sink_used(void);
size_t sdr_sink_capacity(void);
bool   sdr_sink_stop_requested(void);
void   sdr_sink_take_stop(void);
