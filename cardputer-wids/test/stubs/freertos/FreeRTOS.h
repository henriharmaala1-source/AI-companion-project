#pragma once
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
