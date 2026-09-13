/*
 * wids_log.h - NDJSON event log, one JSON object per line.
 *
 * The log format is deliberately identical across monitor mode and honeypot
 * mode so a single parser handles both. Every record carries:
 *
 *   {"t":<int64>,"ev":"<name>", ...mode-specific fields... }
 *
 * `t` is MICROSECONDS SINCE BOOT (esp_timer_get_time). The Cardputer has no
 * battery-backed RTC, so there is no wall-clock time to record. Do not invent
 * one. If an RTC module is added later, add a separate absolute field rather
 * than changing the meaning of `t`.
 *
 * Output goes to one or more sinks. A sink is a plain function pointer so the
 * C code here never has to know about Arduino's C++ Serial object or the SD
 * library - the .ino installs the sinks at boot. That indirection also dodges
 * a real trap: with USB-CDC on boot, printf() and Serial do not necessarily
 * land on the same UART.
 */
#ifndef WIDS_LOG_H
#define WIDS_LOG_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UART now; SD is the second sink in roadmap step 5. */
#define WIDS_LOG_MAX_SINKS 2

/* One NDJSON line, including the trailing newline. Records longer than this
 * are truncated rather than overflowing - see wids_log_event(). */
#define WIDS_LOG_LINE_MAX  512

typedef void (*wids_log_sink_fn)(const char *data, size_t len);

/* Create the internal lock. Call once from setup() before starting tasks. */
void wids_log_init(void);

/* Returns false if the sink table is full. */
bool wids_log_add_sink(wids_log_sink_fn sink);

/*
 * Emit one NDJSON record. `ev` is the event name; `fields` is a printf-style
 * format producing the remaining JSON members WITHOUT a leading comma, or NULL
 * for none. The printf attribute lets -Wformat catch argument mismatches,
 * which is the whole point of building with -Wall -Wextra.
 */
void wids_log_event(const char *ev, const char *fields, ...)
    __attribute__((format(printf, 2, 3)));

/*
 * Escape `in_len` bytes into a JSON string body (no surrounding quotes).
 *
 * Anything off the air is hostile until proven otherwise: an SSID may contain
 * quotes, backslashes, control bytes or invalid UTF-8, any of which would
 * corrupt the log or let an attacker inject fields. We therefore pass through
 * only printable ASCII and escape everything else as \u00XX.
 *
 * That is lossy for legitimate UTF-8 SSIDs, so callers should also log the raw
 * bytes as hex when the input is not pure printable ASCII - the hex field is
 * the forensic ground truth, the escaped string is for reading.
 *
 * Returns the number of bytes written (always NUL-terminated when out_cap > 0).
 */
size_t wids_json_escape(const uint8_t *in, size_t in_len, char *out, size_t out_cap);

/* Lowercase hex, NUL-terminated. Returns bytes written excluding the NUL. */
size_t wids_hex_encode(const uint8_t *in, size_t in_len, char *out, size_t out_cap);

/* True if every byte is in [0x20, 0x7E]. Empty input counts as printable. */
bool wids_is_printable_ascii(const uint8_t *in, size_t len);

/* Format a MAC as aa:bb:cc:dd:ee:ff. `out` needs 18 bytes. */
void wids_format_mac(const uint8_t mac[6], char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* WIDS_LOG_H */
