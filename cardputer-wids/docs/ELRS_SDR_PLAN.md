# ELRS detection on the Cardputer: feasibility and plan

**Goal.** A Cardputer mode that listens to the 2.4 GHz band through
[esp-sdr](https://github.com/ESPARGOS/esp-sdr)'s raw-radio path, recognises an
ExpressLRS (ELRS) control link nearby, and shows what it found on the LCD.

**Verdict: plausible, not yet proven.** Nothing in the source reading or the
numbers below rules it out. Three things can only be settled on real hardware,
and the plan is ordered so they are tested first (milestone M0), before any
firmware is written.

Scope stays inside the brief: receive only, 2.4 GHz only, presence detection
only. Nothing here decodes, jams or transmits. ELRS also exists at 868/915 MHz;
this hardware cannot see that band and the UI must say so.

---

## 1. What is verified (read from source, not assumed)

### ELRS on the air (ExpressLRS `src/lib/FHSS/FHSS.cpp`, `src/src/common.cpp`, commit 15c7899)

| Property | Value |
|---|---|
| Band | 2400.4–2479.4 MHz, 80 channels, 1 MHz apart: channel *k* = 2400.4 + *k* MHz |
| Radio | Semtech SX1280 / LR1121: LoRa 800 kHz, FLRC 0.6 MHz or GFSK. Not 802.11, not BLE |
| Packet rate | 50–1000 Hz (interval 1–20 ms) |
| Hop | every 2 or 4 packets, so each channel is held for about 2–40 ms |
| Sequence | 240 hops = 3 blocks of 80. **Each block visits all 80 channels exactly once** and starts on the sync channel (*k* = 40, 2440.4 MHz) |

That last row is the fingerprint. Wi-Fi is 20 MHz wide. BLE sits on even-MHz
centres (2402, 2404, …). ELRS is a narrow (≤ 0.8 MHz) burst train on an
**x.4 MHz grid** that covers the whole band uniformly and returns to 2440.4 MHz
every 80 hops.

### esp-sdr on the S3 (commit e74f2a4)

- The S3 is supported. A continuous on-chip FFT runs at 16/40/80 MS/s with
  256–2048 bins, and analogue bandwidth is 13–69 MHz.
- The method is not an Espressif API. It uses raw registers (dump control
  `0x60033d5c`, SRAM owner `0x600c101c`), PHY ROM calls, the RF
  certification-test archive, linker `--wrap`s of PHY internals, and private
  `phy_param` offsets. Upstream's own comment: *"These private ABIs and offsets
  belong to the PHY archives pinned in firmware-targets.json."*
  **Consequence: the firmware must be built with exactly the pinned ESP-IDF**
  (commit `25fe69f`, ESP-IDF **6.2.0-dev**, dated 2026-07-11, GCC 16.1), not
  the v5.5.5 used elsewhere in this repo.
- The RF writer takes **192 KiB of internal SRAM** (three 64 KiB banks at
  `0x3fcb0000–0x3fce0000`). Static data and bss must end below `0x3fcb0000`, so
  everything else (FreeRTOS, Wi-Fi/PHY, our code and its heap) has to fit in
  about 160 KiB of DRAM.
- `CONFIG_FREERTOS_UNICORE=y`. Core 1 runs a bare-metal SIMD worker during
  capture.
- `ring_capture_run()` **masks interrupts on core 0 for the whole run.** There
  is no tick, no SPI DMA completion and no keyboard while it runs. It accepts a
  `duration_ms`, resets its state on every call, and re-enables interrupts at
  the end, so short repeated runs are supported.
- Spectrum frames leave the engine through `txq_push()`, then `ring_write()` in
  `ring_io.h`, which writes straight into USB FIFO registers.

### Cardputer side

- M5GFX builds as a plain ESP-IDF component and explicitly handles IDF 6+. Its
  own build-test project uses `EXTRA_COMPONENT_DIRS` plus `display.init()`.
- The keyboard driver (M5Cardputer) is Arduino-only. The matrix is small:
  select lines GPIO 8/9/11 (3-bit code) and active-low inputs GPIO
  13/15/3/4/5/6/7, giving 8 × 7 = 56 keys, which matches the 4 × 14 keymap.
  Porting it is about 40 lines of `driver/gpio` code.
- The LCD is an ST7789 on SPI3 (pins 33–37). The SD card is on 12/14/39/40.
  esp-sdr's own GPIO module would claim the keyboard and SD pins at boot, so it
  is left out.

## 2. Numbers (estimates — reproduce with `python3 tools/elrs_budget.py`)

Assumptions: 100 mW plus 2 dBi transmitter (the EU EIRP cap), −3 dBi Cardputer
antenna, 10 dB noise figure, 80 MS/s with a 256-bin FFT (312 kHz bins).

| Distance | Line of sight: SNR per bin | Through one wall (−15 dB) |
|---|---|---|
| 200 m | ~38 dB | ~23 dB |
| 1 km | ~24 dB | ~9 dB |
| 3 km | ~14 dB | below the noise floor |

- **Time coverage.** At the upstream 80 MS/s / 256-bin profile, one FFT is
  analysed about every 32 µs. Even a 0.2 ms FLRC packet gets around 6 looks,
  and a 2–40 ms dwell gets hundreds.
- **Detection time.** One full 80-channel block takes 0.16 s at 1000 Hz and
  3.2 s at 50/100 Hz. A verdict within about 5–10 s is a reasonable target.
- **Frequency coverage.** At most 69 MHz of the 79 MHz hop span is visible
  from one centre frequency (about 70 of the 80 channels). Centre on 2442 MHz,
  which is Wi-Fi channel 7, so upstream uses its standard `set_chanfreq` path
  rather than the PLL-offset path. No retuning is needed.

None of this accounts for the real antenna, the uncalibrated gain, the AGC, or
a busy home Wi-Fi band. That is what M0 measures.

## 3. Architecture

```
cardputer-wids/sdr/                 new ESP-IDF 6.2-dev project (separate image)
  third_party/esp-sdr   (submodule, pinned e74f2a4, never edited)
  third_party/M5GFX     (submodule, pinned)
  components/sdr_engine/
      CMakeLists.txt    copies upstream ring_capture.c into the build dir next to
                        our ring_io.h (see below), builds the S3 .S kernels,
                        rx_recalibration.c, rx_calibration_state.c, spectrum_stats.c,
                        applies upstream's linker.lf, sram_guard.ld and --wraps
      ring_io.h         memory transport: ring_write() appends to a RAM frame FIFO,
                        ring_input_available() reads a stop flag
      s3_rx.c           receiver bring-up and tuning, extracted from upstream
                        receiver.c with provenance comments (no command parser,
                        no burst_gpio, no USB lease)
  components/elrs_detect/           pure C, no IDF dependency, host-testable
  components/cp_keyboard/           GPIO-matrix scanner (pins above)
  main/                             app_main, UI, mode loop, NDJSON logger
```

**Why copy `ring_capture.c` into the build directory.** It includes
`"ring_io.h"` with quotes, so the compiler finds upstream's copy in the same
directory before any `-I` path. The only way to substitute a transport without
editing upstream is to compile a copy that sits next to ours. CMake regenerates
the copy on each configure, and the submodule stays pristine.

**Main loop (time-sliced):**

```
loop:
    ring_capture_run(SPEC, 80 MS/s, 256 bins, max-hold, duration ≈ 120 ms)
        → frames land in the RAM FIFO (interrupts masked, nothing else runs)
    interrupts back on:
        drain FIFO into elrs_detect          (~a few ms)
        redraw changed screen regions        (~10–20 ms)
        scan keyboard, write NDJSON events
```

That gives roughly 80–85 % RF duty cycle, which is plenty against 2–40 ms
dwells. The FIFO only has to hold one slice of frames, and its size sets the
slice length. Both get tuned against the real heap left over (M1).

**Mode switching** between this image and the Wi-Fi monitor goes through a
reboot (constraint 2). esp-sdr leaves the PHY in a hand-configured state, so
switching back without a reset isn't safe. The two images can later be merged
into one app with a boot-time mode flag in NVS.

## 4. Detector (`elrs_detect`, pure C, unit-tested on the host)

Input: SPC1 frames (uint8 power code per bin, 0.5 dB steps, max-hold).

1. **Noise floor per bin**: a slow low-percentile tracker that is robust to
   bursts.
2. **Burst extraction**: contiguous bins at least *T* dB above the floor, 0.3–1.2 MHz
   wide. Anything 5 MHz or wider counts as Wi-Fi or a microwave oven and is
   discarded.
3. **Grid match**: assign each burst to ELRS channel *k* if it lies within
   ±0.2 MHz of 2400.4 + *k* MHz. Count on-grid and off-grid bursts separately,
   since BLE lands off-grid.
4. **Window statistics** (sliding 10 s):
   - distinct channels hit
   - uniformity of hits across visible channels
   - on-grid fraction
   - dwell lengths in frames, matched to the discrete set {2, 4, 8, 12, 16, 27, 40} ms
   - periodic activity on 2440.4 MHz
5. **Verdict with hysteresis**: `QUIET` → `HOPPER` (narrowband hopping on a
   1 MHz grid) → `ELRS-LIKE` (x.4 grid + uniform coverage + plausible dwell).
   Each state change becomes one NDJSON event. No identifiers are stored,
   because there are none to store.

Thresholds come from **real recordings**, not guesses. See M0 and M2.

## 5. Screen (240 × 135)

```
ELRS WATCH  2442 MHz ±34           2.4GHz only
┌ 80-column channel activity strip (10 s) ────┐
└─────────────────────────────────────────────┘
State: ELRS-LIKE   score 0.91   ch 61/70
Dwell ~8 ms (≈500 Hz)   last seen 0.4 s
Floor −97  Peak −58 dBFS-ish   drops 0
[m]ode  [c]lear  [+/-] threshold  [t]est
```

Red flash and a beep on entering `ELRS-LIKE`. The footer is permanent:
*2.4 GHz only — quiet ≠ clear; 868/915 MHz ELRS and 5.8 GHz video are invisible
to this device.*

## 6. Milestones (each one gates the next)

| # | What | Done when |
|---|---|---|
| **M0** | **Hardware truth, no code from us.** Flash stock esp-sdr (S3) onto the Cardputer (ESP-WebSDR browser flasher, or a local build). Watch 2442 MHz in the viewer with your own ELRS transmitter on, at 500 Hz and at 50 Hz. Record frames with upstream's tools and run `tools/check_spectrum.py --stats` | The hops are visibly on the x.4 grid at a usable SNR indoors. SPS1 reports FFTs/s, coverage ‰ and free heap. Recordings are saved for M2 |
| M1 | Skeleton IDF project: pinned IDF, `sdr_engine` overlay, M5GFX "hello", keyboard scanner. No detector yet | Builds clean at `-Wall -Wextra` (warnings in our code are errors). `idf.py size` shows DRAM headroom. On device, the LCD draws and keys work between capture slices without a crash or watchdog |
| M2 | `elrs_detect` developed **offline on the M0 recordings**, plus synthetic edge cases (ASan/UBSan, as with the frame parser) | Detects your link in every recording. Zero `ELRS-LIKE` verdicts on Wi-Fi/BLE-only recordings |
| M3 | Integrate the detector, screen and NDJSON log on the device | Live verdict within 10 s at 500 Hz and 50 Hz. A one-hour run with no crash and no heap drift |
| M4 | Field test: indoors/outdoors at 50 / 200 / 500 m, Wi-Fi busy and quiet | Range and false-positive table written into this doc |
| M5 | Merge with the Wi-Fi monitor as one image, mode switch via reboot | Both modes run from the keyboard |

**Before M0: the host simulator (`sdr/sim/`).** The real firmware runs on a PC
against a modelled radio scene: `app_main.c`, `ui.cpp` through the real M5GFX
drawing code, the detector, the decoder, the sink and the log. The esp-sdr
engine is reduced to its frame-emission rules, each one cited to the upstream
line. It checks the log, the verdict timeline and the screen in 11 scenarios.
Its first run found the sink bug and two detector bugs in `BUGLOG.md`. It
validates logic and timing. It cannot validate the antenna, the AGC or real
signal levels: M0 still decides whether the idea works at all.

## 7. Risks

| Risk | Why it matters | How it's caught |
|---|---|---|
| Cardputer antenna/AGC hides weak or nearby-saturated signals | The whole idea fails | M0 |
| Not enough DRAM left beside the 192 KiB ring for M5GFX plus the frame FIFO | The UI starves | M1 `idf.py size` and the SPS1 free-heap readout. Fallback: draw with small DMA strips, no full-screen sprite |
| Upstream esp-sdr is weeks old, partly AI-generated, and depends on private PHY ABIs | A silent break on any IDF bump | Pin everything. Never bump IDF without re-running M0 checks |
| Other 2.4 GHz FHSS links (FrSky, Spektrum, TBS Tracer) | False `ELRS-LIKE` | The grid offset and uniformity tests separate them, and they report as `HOPPER` instead. M2 checks this |
| ELRS EU LBT builds or dual-band LR1121 links behave differently | Missed detections | M0/M4 with a CE-LBT build if available |
| Interrupt-masked slices starve Wi-Fi/PHY housekeeping | Calibration drift over hours | Upstream already handles this (`rx_recalibration_stale()` re-measures DC). M3 one-hour soak |

## 8. Decisions and blockers for the owner

1. **Build permission.** This sandbox's permission system refused to run
   esp-sdr's build, because it executes the project's own CMake and Python
   scripts. Either allow it here (add a Bash permission rule) or build locally.
   M0 itself needs no build: the ESP-WebSDR browser flasher works.
2. **License.** esp-sdr is GPL-3.0. Any image linking its engine is GPL-3.0
   when distributed, and this repo is public. Recommendation: license
   `cardputer-wids/sdr/` as GPL-3.0-or-later and keep the Wi-Fi monitor code
   under whatever you choose for the rest.
3. **Hardware for M0.** An ELRS transmitter you own, plus the board revision
   (original Cardputer or ADV; the keyboard differs). Also confirm that the
   USB-C is the S3's native USB, which the boot log will show.
4. **Sandbox note.** `dl.espressif.com` is blocked by this session's egress
   policy. The IDF Python environment was installed from PyPI instead.
