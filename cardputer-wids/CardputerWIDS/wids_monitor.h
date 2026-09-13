/*
 * wids_monitor.h - passive 2.4 GHz monitor mode.
 *
 * Brings the radio up in WIFI_MODE_NULL (never associated, never transmitting),
 * enables promiscuous capture filtered to management frames, and hops channels
 * 1..13. Observations are parsed in the Wi-Fi driver's callback, handed to a
 * logger task over a queue, and emitted as NDJSON.
 *
 * This mode TRANSMITS NOTHING. It is a receiver. There is no deauth, no beacon
 * spam, no probing - it listens and writes down what it heard.
 *
 * Monitor mode and honeypot mode are mutually exclusive: one radio cannot hop
 * the band and hold a SoftAP on a fixed channel at the same time. Stop one
 * before starting the other.
 */
#ifndef WIDS_MONITOR_H
#define WIDS_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frames_total;   /* mgmt frames handed to us by the driver */
    uint32_t mgmt_frames;    /* those that parsed as management */
    uint32_t beacons;
    uint32_t probe_reqs;
    uint32_t probe_resps;
    uint32_t deauths;
    uint32_t disassocs;
    uint32_t queue_drops;    /* logger fell behind - tune dwell or queue depth */
    uint32_t malformed;      /* failed a bounds check or had a bad rx_state */
    uint8_t  channel;        /* channel currently dwelling on */
    bool     running;
} wids_monitor_stats_t;

/* Returns false if the radio or tasks could not be brought up. */
bool wids_monitor_start(void);

/* Stops capture and tears the radio down. Safe to call when not running. */
void wids_monitor_stop(void);

/* Copies a snapshot of the counters, for the LCD. Safe from any task. */
void wids_monitor_get_stats(wids_monitor_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WIDS_MONITOR_H */
