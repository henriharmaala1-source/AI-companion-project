# BUGLOG

Real mistakes and real ESP-IDF / Arduino gotchas, with the symptom that gave
them away and the fix. Kept current deliberately — the point of this project is
learning embedded C, and the traps are most of the learning.

Newest at the top.

---

## Found by the simulator: the RAM sink starved the receiver (listening 17 %)

**Symptom.** The first run of the host simulator (`sdr/sim/`) failed every
scenario. Every ELRS rate went undetected, the receiver listened only 17 % of
the time, all 66 slices ended in a 500 ms stall, and the decoder saw corrupt
frames (`crc_bad` rising by 9 every 5 s).

**Cause.** I assumed `units_per_frame` set the frame length. On the S3, esp-sdr
ignores it. `ring_capture.c:626` closes a frame "as soon as the previous output
has drained", and `docs/spectrum.md` says so: *"There is no fixed frame timer"*.
My sink accepted bytes instantly, so frames came out every ring unit
(~0.15 ms), and the 12 KiB sink was full ~7 ms into a 100 ms slice. Then:

- the remaining 93 ms merged into one max-hold frame that smeared ~40 hops;
- the end-of-run drain (`:1764`) spun for its full 500 ms deadline, because
  nothing could be written;
- the next run reset the queue (`:1340`) and threw away a frame already half
  written into the sink, hence the CRC errors.

The adaptive slice length never reacted, because frames merged instead of
being dropped, so `drops` stayed 0.

**Fix.** The sink is paced: `sdr_sink_pace()` lets in one frame's bytes per
`frame_us` (2 ms). Because upstream emits only when its queue is empty, the
transport rate *is* the frame clock. The slice is now a fixed 100 ms, and a
`_Static_assert` proves the 16 KiB sink holds it. The simulator now shows 93 %
listening, 2.0 ms frames, 0 corrupt frames and 0 stalls. The pace math is
32-bit, so no library division is called with interrupts masked. The
disassembly shows the only call in `sdr_sink_write` is `esp_timer_get_time`,
which is in IRAM.

**Lesson.** Read what the engine *does*, not what its parameters are called.
And a model of the upstream rules found this in seconds, where on hardware it
would have looked like "the antenna can't see ELRS".

---

## Found by the simulator: Bluetooth hid ELRS from the detector

**Symptom.** ELRS 250 Hz beside Bluetooth audio, BLE and Wi-Fi: never
detected. In a quieter flat it only just passed (on-grid 0.71 against a 0.70
threshold).

**Cause.** `on_grid_ratio` was ELRS-grid bursts over *all* narrow bursts.
Bluetooth Classic, BLE and 802.15.4 send hundreds of narrow bursts per
second, every one counted as evidence against ELRS.

**Fix.** Their channels are centred on integer MHz, which the detector can
already tell from ELRS's x.4 MHz. They are now counted separately (`int_grid`
in the log) and left out of the ratio. As a guard, on-grid hits must also be at
least 8 % of the integer-grid count. ELRS-LIKELY also needs on-grid bursts
*now* (1 s time constant). Without that the verdict outlived the link by 23 s;
now it is ~15 s, which is the 10 s hold plus decay.

## Found by the simulator: clipped bursts landed on the ELRS grid

**Symptom.** While checking that guard: loud Bluetooth alone scored 0.165
on-grid per integer-grid burst, double the guard. All of it was on channels 9,
41 and 74.

**Cause.** Those are the edges of the usable band and the DC guard beside the
2442 MHz LO. A burst cut off by an ignored bin has a biased centre, and some
were pulled onto the x.4 grid.

**Fix.** A narrow run that touches an ignored bin is not graded. A clipped
wide run still counts as wide. Bluetooth alone now scores 0.000; 50 Hz ELRS
beside it scores 0.26.

## Found by the simulator: "last ELRS-grid burst 5 s ago", 35 s after the link stopped

Single noise spikes that happen to sit on the grid updated `last_seen_us`. The
verdict ignored them correctly, but the screen line did not. The screen now
says **"ELRS last detected N s ago"**, taken from the verdict.

---

## Link error: "S3 RF ring overlaps BSS"

**Symptom.** The first full build of the ELRS-watch firmware compiled cleanly,
then the link stopped on upstream's guard in `sram_guard.ld`: static data
ended at `0x3fcb0bc0`, 3,008 bytes past the RF ring's start (`0x3fcb0000`).

**Cause.** On the ESP32-S3, DRAM starts where IRAM code ends: they share the
same SRAM. esp-sdr's own engine already carries ~68 KiB of BSS (16 KiB USB
queue, 16 KiB accumulator, FFT and window buffers, an 8 KiB core-1 stack).
Linking M5GFX, the SPI driver and the USB driver added enough IRAM code to
push the DRAM start up and the end of BSS into the ring.

**Fix.** `CONFIG_HEAP_PLACE_FUNCTION_INTO_FLASH=y` moves ~6 KiB of allocator
code out of IRAM, and with it the SPI master ISR, since that option depends on
it. BSS now ends at `0x3fcad8f0`, 10,000 bytes clear. This is safe here: no heap
function is called from an IRAM ISR while the flash cache is off, and the
capture loop never allocates.

**Lesson.** Upstream's guard did its job: without it, this would have been a
heap silently overlapping memory the radio writes into. Watch that margin
whenever a component is added.

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
