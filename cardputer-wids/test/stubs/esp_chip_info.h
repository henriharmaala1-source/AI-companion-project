#pragma once
#include <stdint.h>
#define CHIP_FEATURE_WIFI_BGN (1<<1)
#define CHIP_FEATURE_BLE      (1<<4)
typedef enum { CHIP_ESP32S3 = 9 } esp_chip_model_t;
typedef struct {
    esp_chip_model_t model;
    uint32_t features;
    uint16_t revision;
    uint8_t  cores;
} esp_chip_info_t;
void esp_chip_info(esp_chip_info_t *out_info);
