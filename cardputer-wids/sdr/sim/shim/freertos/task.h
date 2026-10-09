/* Host shim: a delay advances the virtual clock; this is also where the
 * simulator ends a run (app_main never returns on the device). */
#pragma once
#include "FreeRTOS.h"
static inline void vTaskDelay(TickType_t ticks) { sim_task_delay_ms(ticks); }
