/* Host shim: just enough of esp_err.h for the code under simulation.
 * Only the identity of these codes matters on the host, not their values. */
#pragma once
#include "sim.h"
typedef int esp_err_t;
#define ESP_OK                         0
#define ESP_FAIL                       (-1)
#define ESP_ERR_NO_MEM                 0x101
#define ESP_ERR_INVALID_STATE          0x103
#define ESP_ERR_NVS_NO_FREE_PAGES      0x110d
#define ESP_ERR_NVS_NEW_VERSION_FOUND  0x1110
#define ESP_ERROR_CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) { sim_fail("ESP_ERROR_CHECK failed: " #x); } } while (0)
