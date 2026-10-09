/* Host shim: plain malloc. The simulator cannot model the S3 heap, so the
 * size queries report 0 and the boot log says so by showing heap_free 0. */
#pragma once
#include <stddef.h>
#include <stdlib.h>
#define MALLOC_CAP_INTERNAL (1u << 11)
#define MALLOC_CAP_8BIT     (1u << 2)
static inline void *heap_caps_malloc(size_t n, unsigned caps) { (void)caps; return malloc(n); }
static inline void *heap_caps_calloc(size_t c, size_t n, unsigned caps) { (void)caps; return calloc(c, n); }
static inline size_t heap_caps_get_free_size(unsigned caps) { (void)caps; return 0; }
static inline size_t heap_caps_get_largest_free_block(unsigned caps) { (void)caps; return 0; }
