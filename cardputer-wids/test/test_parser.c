/*
 * Host-side test for the frame parser. Includes the .c directly so the static
 * parsing helpers are reachable. Built with AddressSanitizer and frames placed
 * in exactly-sized heap allocations, so any read past the end of a frame is a
 * hard failure rather than a silent success.
 */
#include "wids_monitor.c"   /* -I points at the sketch dir */
#include "freertos/semphr.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- minimal stub implementations so the test links ---- */
static wids_obs_t g_captured;
static int        g_capture_count;

int64_t esp_timer_get_time(void) { return 1234567; }
QueueHandle_t xQueueCreate(UBaseType_t a, UBaseType_t b) { (void)a; (void)b; return (QueueHandle_t)1; }
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t w) {
    (void)q; (void)w; memcpy(&g_captured, item, sizeof(g_captured)); g_capture_count++; return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *i, TickType_t w) { (void)q; (void)i; (void)w; return pdFALSE; }
void vQueueDelete(QueueHandle_t q) { (void)q; }
BaseType_t xTaskCreate(TaskFunction_t f, const char *n, uint32_t s, void *a, UBaseType_t p, TaskHandle_t *o)
{ (void)f; (void)n; (void)s; (void)a; (void)p; (void)o; return pdPASS; }
void vTaskDelete(TaskHandle_t t) { (void)t; }
void vTaskDelay(TickType_t t) { (void)t; }
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *b) { (void)b; return (SemaphoreHandle_t)1; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s; (void)t; return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return pdTRUE; }
esp_err_t esp_netif_init(void) { return ESP_OK; }
esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }
esp_err_t esp_wifi_init(const wifi_init_config_t *c) { (void)c; return ESP_OK; }
esp_err_t esp_wifi_deinit(void) { return ESP_OK; }
esp_err_t esp_wifi_set_mode(wifi_mode_t m) { (void)m; return ESP_OK; }
esp_err_t esp_wifi_start(void) { return ESP_OK; }
esp_err_t esp_wifi_stop(void) { return ESP_OK; }
esp_err_t esp_wifi_set_storage(wifi_storage_t s) { (void)s; return ESP_OK; }
esp_err_t esp_wifi_set_country(const wifi_country_t *c) { (void)c; return ESP_OK; }
esp_err_t esp_wifi_set_ps(wifi_ps_type_t t) { (void)t; return ESP_OK; }
esp_err_t esp_wifi_set_channel(uint8_t p, wifi_second_chan_t s) { (void)p; (void)s; return ESP_OK; }
esp_err_t esp_wifi_set_promiscuous(bool e) { (void)e; return ESP_OK; }
esp_err_t esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_cb_t cb) { (void)cb; return ESP_OK; }
esp_err_t esp_wifi_set_promiscuous_filter(const wifi_promiscuous_filter_t *f) { (void)f; return ESP_OK; }
esp_err_t esp_wifi_set_event_mask(uint32_t m) { (void)m; return ESP_OK; }

static void stdout_sink(const char *data, size_t len) { fwrite(data, 1, len, stdout); }

/*
 * Build a frame in an exactly-sized allocation and push it through the real
 * callback. ASan guards the byte right after `body`.
 */
static void feed(uint8_t fc0, const uint8_t *body, size_t body_len)
{
    const size_t hdr = IEEE80211_MGMT_HDR_LEN;
    const size_t frame_len = hdr + body_len;
    const size_t total = sizeof(wifi_pkt_rx_ctrl_t) + frame_len;

    wifi_promiscuous_pkt_t *pkt = malloc(total);
    assert(pkt != NULL);
    memset(pkt, 0, total);

    pkt->rx_ctrl.rssi     = -42;
    pkt->rx_ctrl.channel  = 6;
    pkt->rx_ctrl.rx_state = 0;
    pkt->rx_ctrl.sig_len  = (unsigned)(frame_len + IEEE80211_FCS_LEN); /* includes FCS */

    pkt->payload[0] = fc0;
    memset(&pkt->payload[4],  0xFF, 6);                 /* addr1 broadcast */
    memset(&pkt->payload[10], 0xAB, 6);                 /* addr2 */
    memset(&pkt->payload[16], 0xCD, 6);                 /* addr3 / BSSID */
    if (body_len > 0) { memcpy(&pkt->payload[hdr], body, body_len); }

    promiscuous_cb(pkt, WIFI_PKT_MGMT);
    free(pkt);
}

/* 12 fixed bytes (timestamp/interval/capability) then tagged params. */
static size_t beacon_body(uint8_t *out, const uint8_t *ies, size_t ie_len)
{
    memset(out, 0, IEEE80211_BEACON_FIXED_LEN);
    memcpy(out + IEEE80211_BEACON_FIXED_LEN, ies, ie_len);
    return IEEE80211_BEACON_FIXED_LEN + ie_len;
}

#define BEACON 0x80
#define DEAUTH 0xC0

int main(void)
{
    wids_log_init();
    wids_log_add_sink(stdout_sink);
    s_fcs_probes_left = 0;   /* silence the FCS probe records for this test */

    uint8_t body[512];
    uint8_t ies[256];
    size_t n;

    printf("--- 1. well-formed beacon, SSID \"TestNet\" ---\n");
    ies[0] = 0x00; ies[1] = 7; memcpy(&ies[2], "TestNet", 7);
    n = beacon_body(body, ies, 9);
    feed(BEACON, body, n);
    assert(g_captured.ssid_len == 7);
    emit_observation(&g_captured);

    printf("--- 2. SSID with JSON metacharacters ---\n");
    { const char *evil = "Evil\"Twin\\";
      ies[0] = 0x00; ies[1] = (uint8_t)strlen(evil); memcpy(&ies[2], evil, strlen(evil));
      n = beacon_body(body, ies, 2 + strlen(evil)); }
    feed(BEACON, body, n);
    emit_observation(&g_captured);

    printf("--- 3. LYING IE length (claims 200 bytes in a short frame) ---\n");
    ies[0] = 0x00; ies[1] = 200; memcpy(&ies[2], "short", 5);
    n = beacon_body(body, ies, 7);
    feed(BEACON, body, n);
    assert(g_captured.ssid_len == 0);   /* must refuse, not read past the end */
    printf("    ssid_len=%u (refused, as required)\n", g_captured.ssid_len);

    printf("--- 4. non-ASCII SSID -> ssid_hex must appear ---\n");
    { const char *utf8 = "Kotiverkk\xc3\xb6";  /* Kotiverkkö */
      ies[0] = 0x00; ies[1] = (uint8_t)strlen(utf8); memcpy(&ies[2], utf8, strlen(utf8));
      n = beacon_body(body, ies, 2 + strlen(utf8)); }
    feed(BEACON, body, n);
    emit_observation(&g_captured);

    printf("--- 5. max-length 32-byte SSID ---\n");
    ies[0] = 0x00; ies[1] = 32; memset(&ies[2], 'A', 32);
    n = beacon_body(body, ies, 34);
    feed(BEACON, body, n);
    assert(g_captured.ssid_len == 32);
    emit_observation(&g_captured);

    printf("--- 6. SSID after a vendor IE (chain walk) ---\n");
    ies[0] = 0xDD; ies[1] = 4; memset(&ies[2], 0x11, 4);    /* vendor */
    ies[6] = 0x00; ies[7] = 3; memcpy(&ies[8], "Lab", 3);   /* ssid   */
    n = beacon_body(body, ies, 11);
    feed(BEACON, body, n);
    assert(g_captured.ssid_len == 3);
    emit_observation(&g_captured);

    printf("--- 7. deauth (no SSID) ---\n");
    feed(DEAUTH, body, 0);
    emit_observation(&g_captured);

    printf("--- 8. zero-length (hidden) SSID ---\n");
    ies[0] = 0x00; ies[1] = 0;
    n = beacon_body(body, ies, 2);
    feed(BEACON, body, n);
    assert(g_captured.ssid_len == 0);
    emit_observation(&g_captured);

    printf("--- 9. runt frame, shorter than the 24-byte header ---\n");
    { const uint32_t before = s_stats.malformed;
      wifi_promiscuous_pkt_t *p = malloc(sizeof(wifi_pkt_rx_ctrl_t) + 10);
      assert(p != NULL);
      memset(p, 0, sizeof(wifi_pkt_rx_ctrl_t) + 10);
      p->rx_ctrl.sig_len = 10 + IEEE80211_FCS_LEN;
      p->payload[0] = BEACON;
      promiscuous_cb(p, WIFI_PKT_MGMT);
      free(p);
      assert(s_stats.malformed == before + 1);
      printf("    rejected, malformed=%u\n", (unsigned)s_stats.malformed); }

    printf("--- 10. bad rx_state (CRC error) must be dropped ---\n");
    { const uint32_t before = s_stats.malformed;
      wifi_promiscuous_pkt_t *p = malloc(sizeof(wifi_pkt_rx_ctrl_t) + 64);
      assert(p != NULL);
      memset(p, 0, sizeof(wifi_pkt_rx_ctrl_t) + 64);
      p->rx_ctrl.sig_len  = 64 + IEEE80211_FCS_LEN;
      p->rx_ctrl.rx_state = 1;
      promiscuous_cb(p, WIFI_PKT_MGMT);
      free(p);
      assert(s_stats.malformed == before + 1);
      printf("    dropped, malformed=%u\n", (unsigned)s_stats.malformed); }

    printf("\nALL PARSER TESTS PASSED\n");
    printf("stats: frames=%u mgmt=%u beacons=%u deauth=%u malformed=%u\n",
           (unsigned)s_stats.frames_total, (unsigned)s_stats.mgmt_frames,
           (unsigned)s_stats.beacons, (unsigned)s_stats.deauths,
           (unsigned)s_stats.malformed);
    return 0;
}
