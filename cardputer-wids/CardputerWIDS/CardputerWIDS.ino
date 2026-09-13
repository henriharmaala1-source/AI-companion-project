/*
 * CardputerWIDS.ino - Cardputer 2.4 GHz wireless intrusion detection.
 *
 * SCOPE, honestly stated:
 *   This device watches the 2.4 GHz band around it and writes down what it
 *   hears. It is ONE LAYER. It can see an attacker operating RF nearby - a
 *   spoofed AP, an evil twin, a deauth/MITM rig, a planted 2.4 GHz device.
 *   It CANNOT see the account-level intrusion path a capable adversary
 *   actually uses: a compromised mailbox, an implant on a device, a companion
 *   device quietly linked to a messaging account. Run it alongside hardware
 *   security keys and canary tokens, not instead of them.
 *
 *   There is one 2.4 GHz radio and no 5 GHz. A quiet screen means "nothing
 *   seen on 2.4", never "all clear".
 *
 * This file is the Arduino/C++ layer only: hardware bring-up, the LCD, the
 * keyboard, and the serial sink. All detection logic lives in the .c files
 * next to it and is plain ESP-IDF C, so it stays portable and readable.
 *
 * Board settings that matter (Arduino IDE -> Tools):
 *   Board:  M5Cardputer
 *   PSRAM:  OPI PSRAM      <- required on an N16R8 module, else the baseline
 *                             table has nowhere to live. The boot log prints
 *                             what was actually found; check it.
 */
#include <M5Cardputer.h>

extern "C" {
#include "wids_config.h"
#include "wids_log.h"
#include "wids_monitor.h"
#include "wids_platform.h"
}

/*
 * RGB565 literals rather than the library's TFT_* names: those live inside a
 * LovyanGFX namespace and their visibility depends on include order. Spelling
 * the values out removes a whole class of "why won't this compile" evenings.
 */
static constexpr uint16_t COL_BG    = 0x0000; /* black  */
static constexpr uint16_t COL_FG    = 0xFFFF; /* white  */
static constexpr uint16_t COL_DIM   = 0x8410; /* grey   */
static constexpr uint16_t COL_OK    = 0x07E0; /* green  */
static constexpr uint16_t COL_WARN  = 0xFFE0; /* yellow */
static constexpr uint16_t COL_ALERT = 0xF800; /* red    */

/* Where the live counters start, below the static header chrome. */
static constexpr int STATS_TOP   = 34;
static constexpr int LINE_HEIGHT = 11;

static constexpr uint32_t UI_REFRESH_MS = 250;
static uint32_t s_last_ui_ms = 0;

enum class Mode { Stopped, Monitor };
static Mode s_mode = Mode::Stopped;

/* ---------------------------------------------------------------- log sink */

/*
 * The C logger knows nothing about Arduino; it just calls whatever sinks were
 * installed. This one puts NDJSON on the USB serial port.
 *
 * Deliberately Serial.write() and not printf(): with USB-CDC on boot those two
 * do not necessarily reach the same place, and silently logging to a UART that
 * is not connected to anything is a miserable thing to debug.
 */
static void serial_sink(const char *data, size_t len)
{
    Serial.write(reinterpret_cast<const uint8_t *>(data), len);
}

/* --------------------------------------------------------------------- UI */

static void ui_draw_chrome()
{
    auto &d = M5Cardputer.Display;

    d.fillScreen(COL_BG);

    d.setTextSize(1);
    d.setTextColor(COL_FG, COL_BG);
    d.setCursor(2, 2);
    d.print("CARDPUTER WIDS");

    /* The scope caveat is permanent screen furniture, not a splash message. */
    d.setTextColor(COL_DIM, COL_BG);
    d.setCursor(2, 13);
    d.print("2.4GHz only - never 'all clear'");

    d.drawFastHLine(0, 26, d.width(), COL_DIM);
    d.drawFastHLine(0, d.height() - 13, d.width(), COL_DIM);

    d.setTextColor(COL_DIM, COL_BG);
    d.setCursor(2, d.height() - 10);
    d.print("[m]onitor [h]oneypot [s]top [t]est");
}

static const char *mode_label()
{
    switch (s_mode) {
    case Mode::Monitor: return "MONITOR";
    case Mode::Stopped: return "STOPPED";
    }
    return "?";
}

static void ui_draw_stats()
{
    auto &d = M5Cardputer.Display;

    wids_monitor_stats_t st;
    wids_monitor_get_stats(&st);

    /* Repaint just the counter block so the screen does not flicker. */
    const int block_h = (d.height() - 13) - STATS_TOP;
    d.fillRect(0, STATS_TOP, d.width(), block_h, COL_BG);

    d.setTextSize(1);
    int y = STATS_TOP;

    d.setTextColor(s_mode == Mode::Monitor ? COL_OK : COL_WARN, COL_BG);
    d.setCursor(2, y);
    d.printf("%-8s ch %2u", mode_label(), (unsigned)st.channel);
    y += LINE_HEIGHT;

    d.setTextColor(COL_FG, COL_BG);
    d.setCursor(2, y);
    d.printf("frames %-6u bcn %u", (unsigned)st.frames_total, (unsigned)st.beacons);
    y += LINE_HEIGHT;

    d.setCursor(2, y);
    d.printf("preq %-6u presp %u", (unsigned)st.probe_reqs, (unsigned)st.probe_resps);
    y += LINE_HEIGHT;

    /* Deauth and disassoc counts are the interesting ones: a flood here is the
     * classic prelude to a handshake capture or a MITM. Colour them once they
     * are non-zero so they catch the eye across a room. */
    d.setTextColor((st.deauths || st.disassocs) ? COL_ALERT : COL_FG, COL_BG);
    d.setCursor(2, y);
    d.printf("deauth %-5u disas %u", (unsigned)st.deauths, (unsigned)st.disassocs);
    y += LINE_HEIGHT;

    d.setTextColor((st.queue_drops || st.malformed) ? COL_WARN : COL_DIM, COL_BG);
    d.setCursor(2, y);
    d.printf("drop %-6u bad %u", (unsigned)st.queue_drops, (unsigned)st.malformed);
}

/*
 * Alert signal: red flash plus a short beep. Wired up now so the hardware path
 * is proven before the detectors that will call it exist (roadmap steps 2-3).
 * Press 't' to fire it by hand.
 */
static void ui_alert_flash()
{
    auto &d = M5Cardputer.Display;

    d.fillScreen(COL_ALERT);
    M5Cardputer.Speaker.tone(3000, 120);
    delay(120);

    ui_draw_chrome();
    ui_draw_stats();
}

/* ------------------------------------------------------------ mode control */

static void mode_stop()
{
    if (s_mode == Mode::Monitor) {
        wids_monitor_stop();
    }
    s_mode = Mode::Stopped;
}

static void mode_start_monitor()
{
    if (s_mode == Mode::Monitor) {
        return;
    }
    mode_stop();
    s_mode = wids_monitor_start() ? Mode::Monitor : Mode::Stopped;
}

static void handle_keys()
{
    if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) {
        return;
    }

    if (M5Cardputer.Keyboard.isKeyPressed('m')) {
        mode_start_monitor();
        ui_draw_chrome();
    } else if (M5Cardputer.Keyboard.isKeyPressed('s')) {
        mode_stop();
        ui_draw_chrome();
    } else if (M5Cardputer.Keyboard.isKeyPressed('t')) {
        ui_alert_flash();
    } else if (M5Cardputer.Keyboard.isKeyPressed('h')) {
        /*
         * Honeypot is roadmap step 6 and is NOT built yet. Saying so on screen
         * is better than a key that silently does nothing. It can never run at
         * the same time as monitor mode anyway: one radio cannot hop the band
         * and hold a SoftAP on a fixed channel simultaneously.
         */
        auto &d = M5Cardputer.Display;
        d.fillRect(0, STATS_TOP, d.width(), LINE_HEIGHT * 2, COL_BG);
        d.setTextColor(COL_WARN, COL_BG);
        d.setCursor(2, STATS_TOP);
        d.print("honeypot: not built yet");
        d.setCursor(2, STATS_TOP + LINE_HEIGHT);
        d.print("(roadmap step 6)");
        wids_log_event("mode", "\"mode\":\"honeypot\",\"state\":\"not_implemented\"");
        delay(1200);
        ui_draw_chrome();
    }
}

/* ------------------------------------------------------------ setup / loop */

void setup()
{
    auto cfg = M5.config();
    cfg.internal_mic = false;  /* not used; keep the I2S peripheral free */
    cfg.internal_imu = false;  /* the Cardputer has none */
    cfg.internal_spk = true;   /* needed for the alert beep */
    cfg.clear_display = true;

    M5Cardputer.begin(cfg, true /* enable keyboard */);

    Serial.begin(115200);

    wids_log_init();
    wids_log_add_sink(serial_sink);

    /* Report the silicon before anything else: if PSRAM is missing, every
     * later decision about the baseline table depends on knowing that now. */
    wids_platform_info_t info;
    wids_platform_probe(&info);
    wids_platform_log(&info);

    M5Cardputer.Display.setRotation(1);
    ui_draw_chrome();
    ui_draw_stats();

    /* Start listening straight away; 's' stops it. */
    mode_start_monitor();
    ui_draw_chrome();
}

void loop()
{
    M5Cardputer.update();
    handle_keys();

    const uint32_t now = millis();
    if (now - s_last_ui_ms >= UI_REFRESH_MS) {
        s_last_ui_ms = now;
        ui_draw_stats();
    }

    delay(5);
}
