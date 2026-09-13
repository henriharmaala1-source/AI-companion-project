#pragma once
#include "freertos/FreeRTOS.h"
typedef void *SemaphoreHandle_t;
typedef struct { int dummy; } StaticSemaphore_t;
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *b);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
