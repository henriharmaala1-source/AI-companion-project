/* Host shim: the USB console is a log file the simulator also parses. */
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef struct {
    uint32_t tx_buffer_size;
    uint32_t rx_buffer_size;
} usb_serial_jtag_driver_config_t;
static inline esp_err_t usb_serial_jtag_driver_install(usb_serial_jtag_driver_config_t *c) { (void)c; return ESP_OK; }
static inline int usb_serial_jtag_write_bytes(const void *src, size_t size, TickType_t wait)
{
    (void)wait;
    sim_console_write((const char *)src, size);
    return (int)size;
}
