# BUGLOG

Real mistakes and real ESP-IDF / Arduino gotchas, with the symptom that gave
them away and the fix. Kept current deliberately — the point of this project is
learning embedded C, and the traps are most of the learning.

Newest at the top.

---

## esp-sdr is welded to one ESP-IDF commit

**What the source says.** esp-sdr's S3 receiver calls PHY ROM functions, wraps
private libphy symbols (`--wrap=chip_v7_set_chan_ana`, `set_rx_gain_cal_dc`) and
pokes `phy_param` at fixed offsets. Its own comment: *"These private ABIs and
offsets belong to the PHY archives pinned in firmware-targets.json."* The S3
pin is ESP-IDF commit `25fe69f` — **6.2.0-dev**, GCC 16.1 — not a release.

**Consequence.** The SDR image must use exactly that IDF. Building it against
v5.5.5 (what the rest of this repo uses) may compile and still misbehave on RF.
Never bump the IDF for the SDR image without re-running the M0 hardware checks
in `docs/ELRS_SDR_PLAN.md`.

---

## A same-directory `#include "x.h"` cannot be overridden with `-I`

**Symptom (found while planning).** Wanted to swap esp-sdr's `ring_io.h`
(USB transport) for a RAM-buffer version by putting our directory first on the
include path.

**Why it fails.** For `#include "file"`, GCC searches the *including file's own
directory* before any `-I` path. `ring_capture.c` and `ring_io.h` share a
directory, so upstream's header always wins.

**Fix.** Compile a build-time copy of `ring_capture.c` that sits next to our
`ring_io.h`; the submodule itself stays untouched.

---

## Host test failed to compile: `unknown type name 'SemaphoreHandle_t'`

**Symptom.** The parser test `#include`s `wids_monitor.c` directly (to reach its
`static` helpers), then defines stub FreeRTOS functions. Four errors, all
semaphore types.

**Cause.** `wids_monitor.c` includes `queue.h` and `task.h` but not `semphr.h`
— it has no semaphores. The semaphore stubs in the test needed types that no
include in that translation unit had pulled in. `wids_log.c` includes `semphr.h`,
but that is a *separate* translation unit; headers do not leak between them.

**Fix.** `#include "freertos/semphr.h"` explicitly in the test.

**Lesson.** In C, every translation unit stands alone. "It compiles in the other
file" means nothing.

---

## `sig_len` and the 4-byte FCS — ground truth for this IDF

**What the header actually says.** ESP-IDF v5.5.5,
`components/esp_wifi/include/local/esp_wifi_types_native.h:81`:

```c
unsigned sig_len: 12;   /**< length of packet including Frame Check Sequence(FCS) */
```

So on v5.5.5 you subtract 4 to get the 802.11 frame length. That is what
`wids_monitor.c` does.

**Why it still needs checking.** This has not been consistent across IDF
versions, and the Arduino core pins *its own* bundled IDF, which is very
unlikely to be v5.5.5. The documented contract you get is whatever that core
shipped with.

**How to check.** `WIDS_FCS_PROBE_FRAMES` in `wids_monitor.c` makes the first
few frames log their raw `sig_len` as `{"ev":"fcs_probe",...}`. Point the
device at an AP whose beacon length you know, compare, then set the constant
to 0. Do not skip this; a 4-byte error shifts every information element.

**Also worth knowing.** `rx_ctrl.rx_state` is 0 when the driver saw no receive
error. Frames with anything else are dropped — parsing corrupt frames only
manufactures false positives.

---

## There are two Cardputer hardware revisions, with different keyboards

**Discovered while reading the M5Cardputer library** rather than the hard way,
thankfully. `Keyboard_Class::begin()` branches on `M5.getBoard()`:

| Board enum              | Value | Keyboard hardware       |
|-------------------------|-------|-------------------------|
| `board_M5Cardputer`     | 14    | GPIO matrix (IOMatrix)  |
| `board_M5CardputerADV`  | 24    | TCA8418 I2C controller  |

M5GFX tells them apart from a panel-ID read (`(result & 0x0C) == 0x0C`).

**Consequence.** Use the library's keyboard API and never hardcode matrix pins
— it already handles both. More importantly: **the SD card SPI pins must be
confirmed against the schematic for the revision actually in hand** before
roadmap step 5. Do not copy pin numbers off a forum post.

**LCD, confirmed from the M5GFX source** (same for both revisions): ST7789 on
`SPI3_HOST`, MOSI 35, SCLK 36, DC 34, RST 33, CS 37, 3-wire, `offset_x` 52,
`offset_y` 40, rotation 1.

---

## LovyanGFX `TFT_*` colour names are not plain macros

**Symptom (anticipated, avoided).** `TFT_BLACK` and friends are not global
`#define`s. In `M5GFX/src/lgfx/v1/misc/enum.hpp` the library *undefines* any
existing `TFT_BLACK` macro and then declares `static constexpr int TFT_BLACK`
inside its own namespace. Whether the bare name resolves depends on include
order and `using` declarations.

**Fix.** `CardputerWIDS.ino` uses explicit RGB565 literals with a comment
naming each colour. One less thing to debug at midnight.

---

## `printf()` and `Serial` do not necessarily go to the same place

With USB-CDC-on-boot, Arduino's `Serial` is the USB CDC device while `printf()`
may still be routed to UART0 on physical pins the Cardputer does not
conveniently expose. Logging into a UART nobody is reading is a bleak debugging
session.

**Fix.** `wids_log.c` never calls `printf`. It writes through sink function
pointers, and the `.ino` installs one backed by `Serial.write()`. As a bonus
this is how SD logging gets added in step 5 — a second sink, no changes to the
C code.

---

## Arduino freezes `sdkconfig`, so Wi-Fi RX buffers cannot be tuned

The Arduino core ships a **precompiled** ESP-IDF. There is no `menuconfig`, so
`CONFIG_ESP_WIFI_*` receive-buffer counts are whatever that core was built
with. This is the standard knob for a promiscuous sniffer that starts dropping
frames under load, and here it is unavailable.

**Mitigation.** The levers we *do* have: `WIDS_CHAN_DWELL_MS` (longer dwell,
fewer channel switches) and `WIDS_EVENT_QUEUE_LEN` (deeper handoff queue). The
`queue_drops` counter on the LCD says when either is needed. Accepted cost of
choosing Arduino for the M5Cardputer display/keyboard support.

---

## `esp_wifi_set_event_mask` — bits set *disable* delivery

From `esp_wifi.h:1175-1190`, verbatim: *"Events which have corresponding bit set
in the mask will not be delivered to the system event handler"*, and *"Default
WiFi event mask is `WIFI_EVENT_MASK_AP_PROBEREQRECVED`"*.

So probe-request events are masked **off** by default, and
`esp_wifi_set_event_mask(0)` is what turns them on.

**Scope note.** This matters for **honeypot mode** (roadmap step 6), where probe
requests arrive as driver events. Monitor mode reads frames straight from the
promiscuous callback and does not depend on it. Do not "fix" a quiet monitor
mode by reaching for this.

---

## PSRAM is a board-menu dropdown, not a runtime decision

On an N16R8 module PSRAM must be set to **OPI PSRAM** in Arduino IDE →
Tools. Get it wrong and the chip still boots, the baseline table just silently
has nowhere to live.

**Fix.** `wids_platform.c` probes `heap_caps_get_total_size(MALLOC_CAP_SPIRAM)`
at boot and the `{"ev":"boot"}` record reports `psram`, `psram_bytes` and an
inferred module variant. Check that line before trusting anything else.
