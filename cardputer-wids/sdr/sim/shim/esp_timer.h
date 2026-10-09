/* Host shim: time is the simulator's virtual clock (us since boot). */
#pragma once
#include <stdint.h>
#include "sim.h"
static inline int64_t esp_timer_get_time(void) { return sim_now_us(); }
