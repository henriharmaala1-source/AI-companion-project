/*
 * sim_beep_hw.c - the speaker backend on the host: no sound, but time passes
 * as it would on the device, so the simulated radio is deaf for as long as
 * the real beep blocks it.
 */
#include "beep.h"
#include "sim.h"

int beep_hw_open(void) { return 1; }

void beep_hw_tone(unsigned hz, unsigned ms, int amp)
{
    (void)hz;
    (void)amp;
    sim_advance_us((int64_t)ms * 1000);
}

void beep_hw_close(void)
{
    sim_advance_us(16000);   /* the silence that flushes the last samples */
}
