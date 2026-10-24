# BUGLOG

Real mistakes and real ESP-IDF / Arduino gotchas, with the symptom that gave
them away and the fix. Kept current deliberately — the point of this project is
learning embedded C, and the traps are most of the learning.

Newest at the top.

---

## Detection lost on hardware: the S3 FFT and a 4-byte-aligned heap buffer

**Symptom.** After the "software range" update, the Cardputer stopped
detecting a controller 2 m away. The `lora` log showed `"cands":0` in every
line: the chirp detector never found a channel worth dechirping. The
spectrum frames in the same log showed the ELRS hops clearly, 0.9 MHz wide
and up to 48 dB above the noise, at 2405, 2426, 2430, 2455 and 2481 MHz. The
transmitter was fine. An "old" image also failed, but it showed `33 snaps`,
and the history of the firmware that detected at 100–200 m holds only 32.
So that image was already a later build.

**Cause (probable; not proven on the device).**
- On the S3, `lora_detect` runs its FFTs through esp-dsp's assembly routine,
  which loads data in 64-bit pairs (`ee.ldf.64.ip`).
- The FFT buffer is the spectrum sink's RAM, from `heap_caps_malloc`.
  ESP-IDF's allocator only promises 4-byte alignment (TLSF,
  `ALIGN_SIZE_LOG2 = 2`).
- The sink is allocated right after `app_t`. That struct grew by 4 bytes
  (the fixed-gain setting), then by 8 more (empty-frame count, filter
  setting).
- An 8-byte-aligned buffer would have become 4 mod 8. The routine then reads
  shifted float pairs, every spectrum is garbage, and no channel ever stands
  out.
- The firmware that worked had simply landed on an aligned address.
- Neither the host tests nor the simulator could see this: they use the
  plain-C FFT.

**Fix.**
- `lora_scan()` aligns its work area to 16 bytes itself (`LORA_WORK_FLOATS`
  has 4 floats of slack for it).
- `fft()` falls back to the plain-C FFT if handed a misaligned buffer.
- The `lora` log carries `work_mod16`, the scratch address mod 16, so the
  next hardware run confirms or refutes this cause.
- A host test runs the scan on a deliberately misaligned buffer.

**Lesson.** A library's fast path can carry rules the portable path does
not, and a test suite that only ever runs the portable path cannot catch
them. Anything handed to vendor SIMD code gets explicit alignment, never
whatever the heap happens to return.

---

## Upstream review (esp-sdr e74f2a4 → f86da81): three problems on our side

esp-sdr gained 48 commits in a week, all with the same ESP-IDF pin. Reading
them against our port turned up three problems. None could have shown in the
host tests or the simulator, because neither modelled what upstream had just
documented.

**1. Empty frames dragged the grid detector's floor down.** Upstream fixed
this in `3b030e0`: when core 1 falls behind, it abandons a unit's FFT work.
At our pin, a frame is closed mid-run only if it holds an FFT (`:1191`), but
the final frame of a run goes out whenever it holds a unit (`:1287`), from
the zeroed accumulator. A slice ending in abandoned units therefore delivered
an all-zero spectrum. We end a run every 100 ms, and the first hardware run
abandoned 88 % of its FFTs. The simulator now models abandonment: at 88 %,
146 empty frames in 60 s. The grid detector's floor tracker drops fast on low
readings, so its floor sagged from 33 to 25–30 dB, and with the threshold at
+10 dB, noise started to count as bursts. That matches the "ELRS on and off
look the same" symptom of the first hardware run, though it may not be the
whole cause.
**Fix:** `on_frame()` skips frames with `ffts == 0` and counts them
(`empty_frames` in `diag`). The floor now holds at 30 dB.

**2. The chirp detector's SNR read ~23 dB high.** Upstream
`docs/rx-controls.md`: "At 80 MS/s most FFT bins lie outside the analog
filter, and their median is about 25 dB lower than the noise inside it."
`lora_detect` took its noise from exactly that median. A new host model (a
30 MHz analog low-pass, plus ADC noise 25 dB lower) reproduced it: a 23 dB
signal measured 46 dB. Detection itself was unaffected, because the dechirp
score normalises itself.
**Fix:** noise comes from the quieter neighbour band of each candidate. With
or without the filter, it is within 1 dB of the truth.

**3. Filter edges took every candidate slot.** Found while testing fix 2. A
channel enters the dechirp by standing out from its quieter neighbour. At an
edge of the analog filter, the outer neighbour is 25 dB down, so the edge
looks like a strong signal. Edges filled all six slots: behind the filter,
detection at 0 dB fell from 20/20 to 5/20.
- Ranking by the louder neighbour fixed the edges, but lost LoRa next to a
  carrier 1.9 MHz away. A single snapshot cannot tell those two cases apart.

**Fix:** a long-term per-bin noise floor (`lora_floor_t`). It falls fast and
rises 0.05 dB per snapshot, so it learns the filter shape and steady carriers.
A candidate must also stand 1 dB above that floor.
- Filter edges are always in the same place, so they never stand above it.
- ELRS hops, so a given channel holds it in only ~1 % of snapshots, and it
  stays above the floor.
- Result: behind the filter, 20/20 at 0 dB and 0/60 false.
- Side effect: LoRa parked on one channel fades into the floor over tens of
  seconds, which fits the "require hopping" to-do.
- The floor resets on any gain, filter or threshold change.
- **Hardware AGC.** The floor remembers the gain index it was learned at
  (bits 20–27 of each sample). A snapshot at another index is judged the
  old way and leaves the floor alone; after 8 such snapshots in a row (~1 s)
  the floor is relearned. Host test: 19/20 right after a 12 dB gain drop.
  (A first version of that test scaled only the signal, which tests SNR,
  not gain; a real AGC step moves the signal and the analog noise together.)

**Still unknown: the S3's actual default filter width.** `diag` now reports
`rx_filter_code` and `rx_filter_mhz`, read back from the register around
each run using upstream's own calls and S3 curve. `w` switches to the widest
setting (~69 MHz) the way upstream's `BANDWIDTH 0` does. Whether wider is
better (more ELRS channels at full sensitivity, but more Wi-Fi into the AGC)
is for the field test to decide.

**Lesson.** A pinned dependency is not a frozen one. Its authors keep
learning things about the hardware that change what our code should assume.
Reading their commit log was worth more than another week of guessing.

---

## The chirp detector threw away signals it could have found (3 dB gate)

**Symptom.** Host sensitivity sweep, SF6: detected 10/10 at +5 dB in-band
SNR, 2/10 at 0 dB. The misses printed `cand 0.0 dB`. The dechirp never ran,
because no channel had been picked to test. Where it did run, the SF6 score
at +5 dB was 66 against a floor of 25, which is plenty of margin.

**Cause.** `lora_scan` dechirps only channels that stand out in a coarse
spectrum: ≥ 3 dB over their neighbours and over the median. A LoRa signal at
0 dB SNR fills its channel with as much power as the noise: (S+N)/N = 2,
which is exactly 3 dB. So the gate, meant to save time, set the sensitivity
limit, 5+ dB above what the dechirp itself can do.

**Fix.** Gate at 1 dB. Averaged over 48 blocks, the coarse power estimate
spreads by a few tenths of a dB, so 1 dB is still clear of noise. A false
candidate only costs scan time, since the dechirp decides. Host sweep: the
50 % point moved from about +1 to about −3 dB, with 0 false alarms in 2,400
negative snapshots. 144 blocks instead of 48 bought almost nothing, so the
scan time is unchanged. A regression check in `test_lora_detect.c` asserts
19/20 at 0 dB.

**Lesson.** When a detector has a cheap pre-filter, measure what the
pre-filter alone lets through. Here the expensive stage had ~10 dB of margin
that the cheap stage never gave it a chance to use.

---

## First real detection: chirps work on hardware

**Result** (Cardputer, ExpressLRS 250 Hz, controller about 2 m away): with
the controller on, ELRS DETECTED. With it off, NOT DETECTED, and no false
alarm afterwards. That held with the drone powered on as well.

**Why the drone alone is invisible.** An ELRS receiver transmits only
telemetry, and only while linked. At 250 Hz the default is `TLM_RATIO_1_64`
(ExpressLRS `common.cpp`): about 4 packets/s, roughly 1 % airtime. With the
controller off it is silent, so no passive receiver can see it. An unlinked
receiver can later start a Wi-Fi AP ("ExpressLRS RX"), which only the Wi-Fi
monitor mode can see.

**Measured** from the `lora` log: SF6 (correct for 250 Hz) with slope −1,
so this binding uses inverted IQ. Score 105–115 against a perfect-chirp
value of ~98, against ~10–13 with the controller off. Asymmetry 21–24×. 6–7
of 12 snapshots hit, and the reported channels are on the ELRS grid
(2447.4, 2458.4 MHz).

**Too slow:** `scan_ms` was 242 when nothing was found (all 6 candidates ×
4 SFs) and 72 on an early hit. Fixes, all host-tested:
- esp-dsp's S3 FFT in place of plain C. It is checked against the host FFT:
  same sign convention, largest difference 1.6e-5. Note that it does nothing
  until `dsps_fft2r_init_fc32` has run, and that init keeps a global pointer
  to the twiddle table, so the 1 KB table is static.
- 48 coarse blocks instead of 160.
- FFTs capped at 256 points.

The new `scan_ms` comes from the next hardware log.

**First range result:** detected at **100–200 m** with the controller inside
a closed car and no line of sight. That is a hard case: a car body alone
typically costs 10–20 dB. The controller's TX power setting and whether
Dynamic Power was on were not recorded. Dynamic Power lowers output when
the link is good, so a real pilot flying far away would probably transmit
more.

**Still to measure:** SF at other packet rates, open line-of-sight range,
range against a known fixed TX power.

---

## Chirp detector: two flaws the host tests caught before hardware

**1. Strong neighbours took every candidate slot.** A carrier and a
Bluetooth burst (37–41 dB) each lit several adjacent grid positions, so the 4
"strongest peaks" never included the 15 dB LoRa signal. Fix: merge
candidates closer than 1 MHz, test 6, and rank by how far a channel stands
out from its neighbours rather than by raw power.

**2. The decimator let neighbours bury the chirp.** A plain 64-sample
integrate-and-dump has −13 dB sidelobes. A carrier 1.9 MHz away leaked into
the 1.25 MHz baseband 12 dB *above* the LoRa signal. Fix: two stages,
a triangular 8× filter (double null at 10 MHz) then a 96-tap
windowed-sinc 8× filter (~50 dB beyond 0.8 MHz). Same cost order, no extra
memory (streaming).

**Also learned from the ExpressLRS source.** `invertIQ = UID[5] & 1`
(`tx_main.cpp`), so about half of all links send down-chirps. A detector that
only looked for up-chirps would miss half of them; both directions are tested.

---

## First hardware run: ELRS on and ELRS off look the same to the detector

**What the logs showed** (Cardputer, 250 Hz ELRS at ~2 m, `diag`/`frame`
records, ~100 s with the transmitter on and ~75 s with it off):

- **On-grid share is 0.5–0.6 whether ELRS is on or off.** The phase histogram
  of burst centres is roughly flat in both captures. A healthy link (simulator)
  piles up within ±100 kHz of the grid. So the earlier "62/62 channels, 59 %
  on-grid" was clutter landing evenly across channels by chance, not ELRS that
  only just failed one test.
- **Most narrow runs are 1–2 bins wide** (~500–1000 per second), on- and off-air.
  A real 0.8 MHz LoRa burst is 2–4 bins. These are fragments: ragged Wi-Fi
  and noise peaks crossing the threshold.
- **With ELRS on:** clipped runs rise ~10×, there is a huge spike at the LO
  (86 dB at 2442.0 MHz) in almost every strongest frame, and there are several
  flat-topped 2–3 MHz plateaus in one 2 ms frame. That looks like a receiver
  overloaded by a transmitter 2 m away, with the hardware AGC jumping (DC steps
  that DC tracking can't follow), not like clean bursts.
- **88 % of FFTs are abandoned.** `ffts` ~3,750 against `abandoned` ~27,450
  per 100 ms slice. At 80 MS/s, core 1 computes only about 6 of the 48 FFTs per
  ring unit. The simulator assumed all of them; upstream counts the rest as
  abandoned work.
- **Free heap at boot is 12 KB** (largest block 7.7 KB), not the 90–100 KB I
  estimated. It works, but there is no room to grow.

**Not yet known.** Whether a fixed receiver gain (new `g` key, upstream's
`GAIN MANUAL`) or a weaker or more distant transmitter gives clean bursts on
the grid. The detector is unchanged until that data exists. The simulator needs
FFT abandonment and AGC behaviour before its passes mean much for hardware.

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
