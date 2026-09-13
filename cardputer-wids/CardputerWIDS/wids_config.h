/*
 * wids_config.h - compile-time tunables for the Cardputer WIDS.
 *
 * Everything here is a knob you may want to turn while tuning on real
 * hardware. Keeping them in one header means you never go hunting through
 * the .c files for a magic number.
 */
#ifndef WIDS_CONFIG_H
#define WIDS_CONFIG_H

/*
 * EU / ETSI regulatory domain: 2.4 GHz channels 1..13.
 *
 * This is a hard constraint, not a preference. Channel 14 is Japan-only and
 * channels above that do not exist in 2.4 GHz. There is no 5 GHz here at all:
 * the Cardputer has a single 2.4 GHz radio, so a clean scan means "nothing
 * seen on 2.4", never "all clear".
 */
#define WIDS_CHAN_FIRST     1
#define WIDS_CHAN_LAST      13
#define WIDS_CHAN_COUNT     (WIDS_CHAN_LAST - WIDS_CHAN_FIRST + 1)

/*
 * Dwell time per channel, milliseconds.
 *
 * Beacons are typically sent every 102.4 ms (100 TU), so a dwell shorter than
 * ~120 ms risks missing an AP entirely on a given pass. 250 ms gives us two
 * beacon intervals of margin at the cost of a ~3.25 s full-band sweep.
 */
#define WIDS_CHAN_DWELL_MS  250

/*
 * Depth of the queue between the promiscuous callback and the logger task.
 *
 * The callback runs in the Wi-Fi driver task and must never block, so it does
 * a non-blocking send and increments a drop counter if the queue is full.
 * A visible drop count is far better than a silently wedged radio.
 */
#define WIDS_EVENT_QUEUE_LEN 64

/* Max SSID length in 802.11 (bytes on the air, not NUL-terminated). */
#define WIDS_SSID_MAX       32

/* Stack sizes (bytes) for the tasks we spawn ourselves. */
#define WIDS_MONITOR_TASK_STACK 4096
#define WIDS_LOGGER_TASK_STACK  4096

#endif /* WIDS_CONFIG_H */
