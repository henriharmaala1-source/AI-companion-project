#pragma once
#include <stddef.h>
#include <stdint.h>
#define MALLOC_CAP_SPIRAM (1<<10)
size_t heap_caps_get_total_size(uint32_t caps);
