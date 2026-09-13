/*
 * wids_log.c - NDJSON emitter. See wids_log.h for the format contract.
 *
 * NOT safe to call from the promiscuous callback. That callback runs in the
 * Wi-Fi driver task and must stay short and non-blocking; formatting JSON and
 * writing a UART there is how you get dropped frames and watchdog resets.
 * The monitor pushes compact records onto a queue instead, and the logger task
 * calls in here.
 */
#include "wids_log.h"
#include "wids_config.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static wids_log_sink_fn s_sinks[WIDS_LOG_MAX_SINKS];
static size_t           s_sink_count;

/* Serialises sink writes so two tasks cannot interleave halves of a line and
 * produce unparseable NDJSON. */
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_storage;

void wids_log_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_storage);
    }
}

bool wids_log_add_sink(wids_log_sink_fn sink)
{
    if (sink == NULL || s_sink_count >= WIDS_LOG_MAX_SINKS) {
        return false;
    }
    s_sinks[s_sink_count++] = sink;
    return true;
}

bool wids_is_printable_ascii(const uint8_t *in, size_t len)
{
    if (in == NULL) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (in[i] < 0x20 || in[i] > 0x7E) {
            return false;
        }
    }
    return true;
}

size_t wids_json_escape(const uint8_t *in, size_t in_len, char *out, size_t out_cap)
{
    if (out == NULL || out_cap == 0) {
        return 0;
    }
    if (in == NULL) {
        out[0] = '\0';
        return 0;
    }

    size_t w = 0;
    for (size_t i = 0; i < in_len; i++) {
        const uint8_t c = in[i];

        /* Worst case for one input byte is 6 output bytes (\u00XX). Stop early
         * unless that plus the terminating NUL still fits. */
        if (w + 6 + 1 > out_cap) {
            break;
        }

        if (c == '"') {
            out[w++] = '\\';
            out[w++] = '"';
        } else if (c == '\\') {
            out[w++] = '\\';
            out[w++] = '\\';
        } else if (c >= 0x20 && c <= 0x7E) {
            out[w++] = (char)c;
        } else {
            /* Control bytes and anything >= 0x80. Guaranteed to fit by the
             * check above, so the return value cannot indicate truncation. */
            const int n = snprintf(out + w, out_cap - w, "\\u%04x", (unsigned)c);
            if (n < 0) {
                break;
            }
            w += (size_t)n;
        }
    }

    out[w] = '\0';
    return w;
}

size_t wids_hex_encode(const uint8_t *in, size_t in_len, char *out, size_t out_cap)
{
    static const char digits[] = "0123456789abcdef";

    if (out == NULL || out_cap == 0) {
        return 0;
    }
    if (in == NULL) {
        out[0] = '\0';
        return 0;
    }

    size_t w = 0;
    for (size_t i = 0; i < in_len; i++) {
        if (w + 2 + 1 > out_cap) { /* two nibbles plus the NUL */
            break;
        }
        out[w++] = digits[(in[i] >> 4) & 0x0F];
        out[w++] = digits[in[i] & 0x0F];
    }

    out[w] = '\0';
    return w;
}

void wids_format_mac(const uint8_t mac[6], char *out, size_t out_cap)
{
    if (out == NULL || out_cap == 0) {
        return;
    }
    if (mac == NULL) {
        out[0] = '\0';
        return;
    }
    snprintf(out, out_cap, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* snprintf reports the length it WOULD have written; clamp it to the buffer so
 * a truncated record still produces a valid, closed JSON object. */
static size_t clamp_written(size_t want, size_t cap)
{
    return (want < cap) ? want : (cap - 1);
}

void wids_log_event(const char *ev, const char *fields, ...)
{
    if (ev == NULL || s_sink_count == 0) {
        return;
    }

    char line[WIDS_LOG_LINE_MAX];

    /* Reserve two bytes so we can always append the closing '}' and '\n'. */
    const size_t cap = sizeof line - 2;
    size_t w = 0;

    /* `ev` is always one of our own string literals, so it needs no escaping. */
    int n = snprintf(line, cap, "{\"t\":%" PRId64 ",\"ev\":\"%s\"",
                     esp_timer_get_time(), ev);
    if (n < 0) {
        return;
    }
    w = clamp_written((size_t)n, cap);

    if (fields != NULL && fields[0] != '\0' && w + 1 < cap) {
        line[w++] = ',';

        va_list ap;
        va_start(ap, fields);
        n = vsnprintf(line + w, cap - w, fields, ap);
        va_end(ap);

        if (n > 0) {
            w = clamp_written(w + (size_t)n, cap);
        }
    }

    line[w++] = '}';
    line[w++] = '\n';

    if (s_lock != NULL && xSemaphoreTake(s_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }
    for (size_t i = 0; i < s_sink_count; i++) {
        s_sinks[i](line, w);
    }
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}
