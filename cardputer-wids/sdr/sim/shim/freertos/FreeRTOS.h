/* Host shim: one thread, a 1 kHz tick on the virtual clock. */
#pragma once
#include <stdint.h>
#include "sim.h"
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdTRUE  1
#define pdFALSE 0
#define portMAX_DELAY      0xffffffffu
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))   /* CONFIG_FREERTOS_HZ=1000 */
typedef struct { int unused; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
