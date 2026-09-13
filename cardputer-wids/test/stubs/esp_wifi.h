#pragma once
/* Stub mirroring the fields/signatures verified in ESP-IDF v5.5.5 headers.
 * Bitfield widths copied from esp_wifi_types_native.h so the host build sees
 * the same integer promotions the target will. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef struct {
    signed   rssi: 8;
    unsigned rate: 5;
    unsigned : 1;
    unsigned sig_mode: 2;
    unsigned : 16;
    unsigned mcs: 7;
    unsigned cwb: 1;
    unsigned : 16;
    unsigned smoothing: 1;
    unsigned not_sounding: 1;
    unsigned : 1;
    unsigned aggregation: 1;
    unsigned stbc: 2;
    unsigned fec_coding: 1;
    unsigned sgi: 1;
    unsigned : 8;
    unsigned ampdu_cnt: 8;
    unsigned channel: 4;
    unsigned secondary_channel: 4;
    unsigned : 8;
    unsigned timestamp: 32;
    unsigned : 32;
    signed   noise_floor: 8;
    unsigned : 24;
    unsigned : 32;
    unsigned : 31;
    unsigned ant: 1;
    unsigned : 32;
    unsigned : 32;
    unsigned : 32;
    unsigned sig_len: 12;
    unsigned : 12;
    unsigned rx_state: 8;
} wifi_pkt_rx_ctrl_t;

typedef struct {
    wifi_pkt_rx_ctrl_t rx_ctrl;
    uint8_t payload[0];
} wifi_promiscuous_pkt_t;

typedef enum { WIFI_PKT_MGMT, WIFI_PKT_CTRL, WIFI_PKT_DATA, WIFI_PKT_MISC } wifi_promiscuous_pkt_type_t;
typedef void (*wifi_promiscuous_cb_t)(void *buf, wifi_promiscuous_pkt_type_t type);

#define WIFI_PROMIS_FILTER_MASK_MGMT (1)
typedef struct { uint32_t filter_mask; } wifi_promiscuous_filter_t;

typedef enum { WIFI_SECOND_CHAN_NONE = 0, WIFI_SECOND_CHAN_ABOVE, WIFI_SECOND_CHAN_BELOW } wifi_second_chan_t;
typedef enum { WIFI_MODE_NULL = 0, WIFI_MODE_STA, WIFI_MODE_AP, WIFI_MODE_APSTA } wifi_mode_t;
typedef enum { WIFI_STORAGE_FLASH, WIFI_STORAGE_RAM } wifi_storage_t;
typedef enum { WIFI_PS_NONE, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM } wifi_ps_type_t;
typedef enum { WIFI_COUNTRY_POLICY_AUTO, WIFI_COUNTRY_POLICY_MANUAL } wifi_country_policy_t;

typedef struct {
    char cc[3];
    uint8_t schan;
    uint8_t nchan;
    int8_t  max_tx_power;
    wifi_country_policy_t policy;
} wifi_country_t;

typedef struct { int placeholder; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() { .placeholder = 0 }

esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_deinit(void);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_stop(void);
esp_err_t esp_wifi_set_storage(wifi_storage_t storage);
esp_err_t esp_wifi_set_country(const wifi_country_t *country);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t esp_wifi_set_promiscuous(bool en);
esp_err_t esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb_t cb);
esp_err_t esp_wifi_set_promiscuous_filter(const wifi_promiscuous_filter_t *filter);
esp_err_t esp_wifi_set_event_mask(uint32_t mask);
