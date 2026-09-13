/*
 * wids_monitor.c - passive monitor mode. See wids_monitor.h.
 *
 * Structure:
 *   promiscuous_cb()  runs in the Wi-Fi driver task. Parses, bounds-checks,
 *                     pushes a small record onto a queue. Never blocks,
 *                     never formats JSON, never touches a UART.
 *   logger_task()     drains the queue and emits NDJSON at normal priority.
 *   hop_task()        walks channels 1..13 on a fixed dwell.
 *
 * Splitting it this way is not ceremony: doing the formatting inside the
 * callback stalls the driver, drops frames, and eventually trips the watchdog.
 */
#include "wids_monitor.h"
#include "wids_config.h"
#include "wids_log.h"

#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* ---------------------------------------------------------------------------
 * 802.11 layout constants
 *
 * Management frame:
 *   [0..1]   frame control
 *   [2..3]   duration
 *   [4..9]   addr1 (destination)
 *   [10..15] addr2 (transmitter / source)
 *   [16..21] addr3 (BSSID for the frames we care about)
 *   [22..23] sequence control
 *   = 24 byte header, then the frame body.
 *
 * Beacon and probe-response bodies start with 12 fixed bytes:
 *   timestamp(8) + beacon interval(2) + capability info(2)
 * and only then the tagged parameters (information elements). Reading the
 * capability field and then stepping to the IEs is exactly the pointer
 * arithmetic that bites people, so it is a named constant here and every
 * offset is checked against the real frame length before use.
 * ------------------------------------------------------------------------ */
#define IEEE80211_TYPE_MGMT          0
#define IEEE80211_MGMT_HDR_LEN       24
#define IEEE80211_BEACON_FIXED_LEN   12
#define IEEE80211_FCS_LEN            4
#define IEEE80211_IE_SSID            0

#define MGMT_SUBTYPE_ASSOC_REQ       0x00
#define MGMT_SUBTYPE_ASSOC_RESP      0x01
#define MGMT_SUBTYPE_PROBE_REQ       0x04
#define MGMT_SUBTYPE_PROBE_RESP      0x05
#define MGMT_SUBTYPE_BEACON          0x08
#define MGMT_SUBTYPE_DISASSOC        0x0A
#define MGMT_SUBTYPE_AUTH            0x0B
#define MGMT_SUBTYPE_DEAUTH          0x0C

/*
 * Log the raw rx_ctrl.sig_len of the first few frames.
 *
 * IDF v5.5 documents sig_len as "length of packet including Frame Check
 * Sequence(FCS)", so we subtract 4. That has not always been true across IDF
 * versions, and the Arduino core pins its own IDF, so CHECK IT on this unit:
 * capture a beacon from an AP whose beacon length you know and compare.
 * Set this to 0 once you have confirmed the behaviour.
 */
#define WIDS_FCS_PROBE_FRAMES 5

/* One parsed observation, sized to stay cheap to copy through the queue. */
typedef struct {
    int64_t t_us;
    uint8_t subtype;
    uint8_t channel;
    int8_t  rssi;
    uint8_t sa[6];      /* addr2 - who transmitted it */
    uint8_t bssid[6];   /* addr3 */
    uint8_t ssid_len;
    uint8_t ssid[WIDS_SSID_MAX];
} wids_obs_t;

static QueueHandle_t       s_queue;
static TaskHandle_t        s_logger_task;
static TaskHandle_t        s_hop_task;
static volatile bool       s_running;
static wids_monitor_stats_t s_stats;
static volatile uint32_t   s_fcs_probes_left = WIDS_FCS_PROBE_FRAMES;

/* ------------------------------------------------------------------ parsing */

/*
 * Return the management subtype, or -1 if this is not a management frame.
 * Only reads frame[0], so it needs at least one byte.
 */
static int mgmt_subtype(const uint8_t *frame, size_t len)
{
    if (len < 1) {
        return -1;
    }
    const uint8_t type = (uint8_t)((frame[0] >> 2) & 0x03);
    if (type != IEEE80211_TYPE_MGMT) {
        return -1;
    }
    return (int)((frame[0] >> 4) & 0x0F);
}

/*
 * Copy the SSID out of a tagged-parameter (IE) block.
 *
 * `body` is the first byte of the tagged parameters and `body_len` is how many
 * bytes are genuinely present. Each IE is [id][len][len bytes]. A hostile or
 * corrupt frame can declare a length that runs past the end of the buffer;
 * following it would walk straight off the driver's DMA buffer, so we stop the
 * walk instead of trusting the claim.
 *
 * Returns the number of SSID bytes copied. 0 means absent, zero-length
 * (a hidden/broadcast SSID), or a malformed IE chain.
 */
static uint8_t parse_ssid_ie(const uint8_t *body, size_t body_len,
                             uint8_t *ssid_out, uint8_t ssid_cap)
{
    size_t pos = 0;

    while (pos + 2 <= body_len) {
        const uint8_t id     = body[pos];
        const uint8_t ie_len = body[pos + 1];

        if (pos + 2 + (size_t)ie_len > body_len) {
            break; /* truncated or lying - trust nothing further in this frame */
        }
        if (id == IEEE80211_IE_SSID) {
            const uint8_t n = (ie_len < ssid_cap) ? ie_len : ssid_cap;
            memcpy(ssid_out, &body[pos + 2], n);
            return n;
        }
        pos += 2 + (size_t)ie_len;
    }
    return 0;
}

/*
 * Offset of the tagged parameters for a given subtype, or 0 if this subtype
 * carries no IEs we want. Beacons and probe responses have the 12-byte fixed
 * block first; probe requests go straight to IEs.
 */
static size_t body_offset_for_subtype(int subtype)
{
    switch (subtype) {
    case MGMT_SUBTYPE_BEACON:
    case MGMT_SUBTYPE_PROBE_RESP:
        return IEEE80211_MGMT_HDR_LEN + IEEE80211_BEACON_FIXED_LEN;
    case MGMT_SUBTYPE_PROBE_REQ:
        return IEEE80211_MGMT_HDR_LEN;
    default:
        return 0;
    }
}

/* Bump the per-subtype counters. Single writer (the Wi-Fi task), so plain
 * increments are fine; the UI only ever reads. */
static void count_subtype(int subtype)
{
    switch (subtype) {
    case MGMT_SUBTYPE_BEACON:      s_stats.beacons++;     break;
    case MGMT_SUBTYPE_PROBE_REQ:   s_stats.probe_reqs++;  break;
    case MGMT_SUBTYPE_PROBE_RESP:  s_stats.probe_resps++; break;
    case MGMT_SUBTYPE_DEAUTH:      s_stats.deauths++;     break;
    case MGMT_SUBTYPE_DISASSOC:    s_stats.disassocs++;   break;
    default:                                              break;
    }
}

/* ----------------------------------------------------------------- callback */

static void promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (buf == NULL || type != WIFI_PKT_MGMT) {
        return;
    }

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    s_stats.frames_total++;

    /* rx_state 0 means the driver saw no error. Anything else is a corrupt
     * receive; parsing it would just manufacture false positives. */
    if (pkt->rx_ctrl.rx_state != 0) {
        s_stats.malformed++;
        return;
    }

    const uint32_t sig_len = (uint32_t)pkt->rx_ctrl.sig_len;

    if (s_fcs_probes_left > 0) {
        s_fcs_probes_left--;
        /* Deliberately raw: we are checking the driver's own number here, not
         * a derived one. Compare against a beacon of known length. */
        wids_log_event("fcs_probe",
                       "\"sig_len\":%u,\"assumed_fcs\":%u,\"note\":"
                       "\"verify sig_len includes FCS on this IDF\"",
                       (unsigned)sig_len, (unsigned)IEEE80211_FCS_LEN);
    }

    if (sig_len <= IEEE80211_FCS_LEN) {
        s_stats.malformed++;
        return;
    }
    const size_t frame_len = (size_t)sig_len - IEEE80211_FCS_LEN;

    if (frame_len < IEEE80211_MGMT_HDR_LEN) {
        s_stats.malformed++;
        return;
    }

    const uint8_t *frame = pkt->payload;
    const int subtype = mgmt_subtype(frame, frame_len);
    if (subtype < 0) {
        return;
    }

    s_stats.mgmt_frames++;
    count_subtype(subtype);

    wids_obs_t obs;
    memset(&obs, 0, sizeof(obs));
    obs.t_us    = esp_timer_get_time();
    obs.subtype = (uint8_t)subtype;
    obs.channel = (uint8_t)pkt->rx_ctrl.channel;
    obs.rssi    = (int8_t)pkt->rx_ctrl.rssi;
    memcpy(obs.sa,    &frame[10], 6);
    memcpy(obs.bssid, &frame[16], 6);

    const size_t body_off = body_offset_for_subtype(subtype);
    if (body_off > 0 && frame_len > body_off) {
        obs.ssid_len = parse_ssid_ie(&frame[body_off], frame_len - body_off,
                                     obs.ssid, WIDS_SSID_MAX);
    }

    /* Non-blocking: the driver task must not wait on us. A rising drop count
     * is the signal to lengthen the dwell or deepen the queue. */
    if (xQueueSend(s_queue, &obs, 0) != pdTRUE) {
        s_stats.queue_drops++;
    }
}

/* -------------------------------------------------------------------- tasks */

static const char *subtype_name(uint8_t subtype)
{
    switch (subtype) {
    case MGMT_SUBTYPE_ASSOC_REQ:  return "assoc_req";
    case MGMT_SUBTYPE_ASSOC_RESP: return "assoc_resp";
    case MGMT_SUBTYPE_PROBE_REQ:  return "probe_req";
    case MGMT_SUBTYPE_PROBE_RESP: return "probe_resp";
    case MGMT_SUBTYPE_BEACON:     return "beacon";
    case MGMT_SUBTYPE_DISASSOC:   return "disassoc";
    case MGMT_SUBTYPE_AUTH:       return "auth";
    case MGMT_SUBTYPE_DEAUTH:     return "deauth";
    default:                      return "mgmt_other";
    }
}

static void emit_observation(const wids_obs_t *obs)
{
    char sa_str[18];
    char bssid_str[18];
    wids_format_mac(obs->sa, sa_str, sizeof(sa_str));
    wids_format_mac(obs->bssid, bssid_str, sizeof(bssid_str));

    /* Worst case 6 output bytes per input byte, plus the NUL. */
    char ssid_esc[WIDS_SSID_MAX * 6 + 1];
    wids_json_escape(obs->ssid, obs->ssid_len, ssid_esc, sizeof(ssid_esc));

    /*
     * When the SSID is not plain printable ASCII the escaped form is lossy, so
     * the raw bytes go out as hex alongside it. The hex is ground truth; the
     * string is for reading.
     */
    if (obs->ssid_len > 0 && !wids_is_printable_ascii(obs->ssid, obs->ssid_len)) {
        char ssid_hex[WIDS_SSID_MAX * 2 + 1];
        wids_hex_encode(obs->ssid, obs->ssid_len, ssid_hex, sizeof(ssid_hex));
        wids_log_event("frame",
                       "\"sub\":\"%s\",\"ch\":%u,\"rssi\":%d,\"sa\":\"%s\","
                       "\"bssid\":\"%s\",\"ssid\":\"%s\",\"ssid_hex\":\"%s\"",
                       subtype_name(obs->subtype), (unsigned)obs->channel,
                       (int)obs->rssi, sa_str, bssid_str, ssid_esc, ssid_hex);
    } else {
        wids_log_event("frame",
                       "\"sub\":\"%s\",\"ch\":%u,\"rssi\":%d,\"sa\":\"%s\","
                       "\"bssid\":\"%s\",\"ssid\":\"%s\"",
                       subtype_name(obs->subtype), (unsigned)obs->channel,
                       (int)obs->rssi, sa_str, bssid_str, ssid_esc);
    }
}

static void logger_task(void *arg)
{
    (void)arg;
    wids_obs_t obs;

    while (s_running) {
        if (xQueueReceive(s_queue, &obs, pdMS_TO_TICKS(200)) == pdTRUE) {
            emit_observation(&obs);
        }
    }
    vTaskDelete(NULL);
}

static void hop_task(void *arg)
{
    (void)arg;
    uint8_t ch = WIDS_CHAN_FIRST;

    while (s_running) {
        if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) == ESP_OK) {
            s_stats.channel = ch;
        }
        vTaskDelay(pdMS_TO_TICKS(WIDS_CHAN_DWELL_MS));

        ch++;
        if (ch > WIDS_CHAN_LAST) {
            ch = WIDS_CHAN_FIRST;
        }
    }
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------ start / stop */

/* Under Arduino the core may already have initialised parts of this stack,
 * depending on what else the sketch touched. "Already done" is not a failure. */
static bool ok_or_already(esp_err_t err)
{
    return (err == ESP_OK) || (err == ESP_ERR_INVALID_STATE);
}

bool wids_monitor_start(void)
{
    if (s_running) {
        return true;
    }

    memset(&s_stats, 0, sizeof(s_stats));
    s_fcs_probes_left = WIDS_FCS_PROBE_FRAMES;

    s_queue = xQueueCreate(WIDS_EVENT_QUEUE_LEN, sizeof(wids_obs_t));
    if (s_queue == NULL) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"queue alloc failed\"");
        return false;
    }

    if (!ok_or_already(esp_netif_init()) ||
        !ok_or_already(esp_event_loop_create_default())) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"netif/event init failed\"");
        return false;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (!ok_or_already(esp_wifi_init(&cfg))) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"esp_wifi_init failed\"");
        return false;
    }

    /* Keep credentials out of NVS entirely - we never associate to anything. */
    (void)esp_wifi_set_storage(WIFI_STORAGE_RAM);

    /*
     * EU / ETSI: channels 1..13, enforced manually rather than inferred from
     * whatever AP happens to be loudest. WIFI_COUNTRY_POLICY_AUTO would let a
     * nearby AP's country IE move our channel set, which is precisely the kind
     * of thing an attacker controls.
     */
    wifi_country_t country;
    memset(&country, 0, sizeof(country));
    country.cc[0]  = 'F';
    country.cc[1]  = 'I';
    country.cc[2]  = '\0';
    country.schan  = WIDS_CHAN_FIRST;
    country.nchan  = WIDS_CHAN_COUNT;
    country.policy = WIFI_COUNTRY_POLICY_MANUAL;
    (void)esp_wifi_set_country(&country);

    /* NULL mode: the radio is on, but there is no station and no AP. We never
     * associate and never transmit. */
    if (esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"set_mode NULL failed\"");
        return false;
    }
    if (!ok_or_already(esp_wifi_start())) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"esp_wifi_start failed\"");
        return false;
    }

    /* Power save would put the radio to sleep between beacons and we would
     * miss frames. A fixed monitoring station can afford to stay awake. */
    (void)esp_wifi_set_ps(WIFI_PS_NONE);

    /* Management frames only, for now. Data and control frames are far more
     * numerous and we have no use for them yet. */
    wifi_promiscuous_filter_t filter;
    memset(&filter, 0, sizeof(filter));
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    (void)esp_wifi_set_promiscuous_filter(&filter);

    if (esp_wifi_set_promiscuous_rx_cb(promiscuous_cb) != ESP_OK ||
        esp_wifi_set_promiscuous(true) != ESP_OK) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"promiscuous enable failed\"");
        return false;
    }

    s_running = true;
    s_stats.running = true;

    if (xTaskCreate(logger_task, "wids_log", WIDS_LOGGER_TASK_STACK,
                    NULL, 4, &s_logger_task) != pdPASS ||
        xTaskCreate(hop_task, "wids_hop", WIDS_MONITOR_TASK_STACK,
                    NULL, 3, &s_hop_task) != pdPASS) {
        wids_log_event("error", "\"where\":\"monitor_start\",\"msg\":\"task create failed\"");
        wids_monitor_stop();
        return false;
    }

    wids_log_event("mode",
                   "\"mode\":\"monitor\",\"state\":\"started\",\"chan_first\":%u,"
                   "\"chan_last\":%u,\"dwell_ms\":%u,\"band\":\"2.4GHz-only\","
                   "\"tx\":\"none\"",
                   (unsigned)WIDS_CHAN_FIRST, (unsigned)WIDS_CHAN_LAST,
                   (unsigned)WIDS_CHAN_DWELL_MS);
    return true;
}

void wids_monitor_stop(void)
{
    if (!s_running) {
        return;
    }

    /* Clearing the flag lets both tasks fall out of their loops and delete
     * themselves; give them a beat before tearing the radio down under them. */
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(WIDS_CHAN_DWELL_MS + 100));

    (void)esp_wifi_set_promiscuous(false);
    (void)esp_wifi_stop();

    if (s_queue != NULL) {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }
    s_logger_task = NULL;
    s_hop_task    = NULL;
    s_stats.running = false;

    wids_log_event("mode", "\"mode\":\"monitor\",\"state\":\"stopped\"");
}

void wids_monitor_get_stats(wids_monitor_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    /* 32-bit aligned reads are atomic on this core, and there is a single
     * writer, so a plain copy cannot tear an individual counter. Fields may be
     * from marginally different instants, which is fine for a display. */
    *out = s_stats;
}
