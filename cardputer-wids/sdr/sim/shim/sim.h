/*
 * sim.h - the simulator's side of the shims: the virtual clock, the console,
 * and the hooks the harness needs. Everything the firmware calls that would
 * touch hardware lands here instead.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int64_t sim_now_us(void);
void    sim_advance_us(int64_t us);          /* time passes, nothing else */
void    sim_task_delay_ms(uint32_t ms);      /* may end the run (longjmp) */
void    sim_console_write(const char *data, size_t len);
void    sim_fail(const char *why) __attribute__((noreturn));

/* Display hooks, implemented in sim_display.cpp. */
void     sim_display_flushed(void);          /* called by M5GFX::waitDMA() */
int      sim_display_ready(void);
uint16_t sim_display_pixel(int x, int y);    /* RGB565 */
int      sim_display_save_png(const char *path);

#ifdef __cplusplus
}
#endif
