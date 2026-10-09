/* Host shim. */
#pragma once
typedef enum { ESP_LOG_NONE = 0 } esp_log_level_t;
static inline void esp_log_level_set(const char *tag, esp_log_level_t level) { (void)tag; (void)level; }
