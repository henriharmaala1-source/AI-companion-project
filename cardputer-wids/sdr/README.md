# ELRS watch — Cardputer firmware (work in progress)

Receive-only, 2.4 GHz-only presence detection of ExpressLRS links, using
[esp-sdr](https://github.com/ESPARGOS/esp-sdr)'s ESP32-S3 capture engine and
the Cardputer screen and keyboard. Design and milestones:
[`../docs/ELRS_SDR_PLAN.md`](../docs/ELRS_SDR_PLAN.md).

This is a separate ESP-IDF image from the Arduino Wi-Fi monitor in
`../CardputerWIDS`. Both can't run at once: one radio, one mode (constraint 2).

## Screen

The main view is one big answer: **ELRS DETECTED** (red, with a flash) or
**NOT DETECTED** (green). The line under it says which kind of "not
detected": no hopping at all, or a hopping signal that isn't on the ELRS
grid, such as Bluetooth. "Not detected" never means "all clear". `d` toggles a
details view with the channel strip and detector numbers.

## Status

| Part | State |
|---|---|
| `components/elrs_detect`: SPC1 frame decoder and ELRS detector | written, **host-tested** (ASan/UBSan, `-Werror`) on a synthetic model |
| `components/cp_keyboard`: GPIO-matrix keyboard port | written. Keymap **host-tested**, GPIO side untested |
| `components/sdr_engine`: esp-sdr engine wrapper, RAM sink, transport overlay | source written, **never compiled** |
| `main/`: app loop, M5GFX screen, NDJSON log | source written, **never compiled** |
| Build files (project/component CMake, sdkconfig, partitions) | **not written**: blocked in the session that produced this code, see below |
| On hardware | **nothing has run on a Cardputer yet** |

The synthetic-model tests show the logic does what it claims on a model of
ELRS, Bluetooth and Wi-Fi taken from source. They do **not** show that the
Cardputer can see a real transmitter. That is milestone M0 in the plan.

## Layout

```
third_party/esp-sdr   submodule, pinned e74f2a4 (GPL-3.0-or-later), never edited
third_party/M5GFX     submodule, pinned c5a3fef (MIT)
components/
  elrs_detect/   pure C, no IDF: spc1.c (frame decoder), elrs_detect.c
  cp_keyboard/   cp_keymap.c (pure, tested), cp_keyboard.c (driver/gpio)
  sdr_engine/    s3_rx.c (extracted from upstream receiver.c), sdr_sink.c,
                 overlay/ring_io.h (replaces upstream's USB transport)
main/            app_main.c, ui.cpp, ui.h
test/            run_tests.sh, test_elrs_detect.c, test_keymap.c
```

## What the missing build files must do

The integration points are documented in the sources:

- **Pinned IDF.** Build only with ESP-IDF commit `25fe69f` (6.2.0-dev). The
  engine depends on private PHY ABIs.
- **sdkconfig.** Upstream's S3 settings, from
  `third_party/esp-sdr/sdkconfig.defaults.esp32s3`: unicore FreeRTOS,
  interrupt and task watchdogs off, PHY cert-test archive on, 240 MHz. Flash
  size should match the Cardputer (8 MB on the boards I know of; check yours).
- **sdr_engine.** Compile a build-time copy of upstream `ring_capture.c`
  placed next to `overlay/ring_io.h`. A same-directory `#include` beats `-I`
  (see BUGLOG). Add upstream's S3 `.S` kernels, `rx_recalibration.c`,
  `rx_calibration_state.c` and `spectrum_stats.c`. Apply upstream's
  `sram_guard.ld`, `--wrap=chip_v7_set_chan_ana` and
  `--wrap=set_rx_gain_cal_dc`, plus a linker fragment keeping the S3 kernels
  and `sdr_sink` in IRAM.
- **main.** Also builds `../CardputerWIDS/wids_log.c`, so both modes share one
  NDJSON format.

## Run the host tests

```sh
git submodule update --init --recursive   # first time only
./test/run_tests.sh
```

## Licensing

esp-sdr is GPL-3.0-or-later. `s3_rx.c` and `overlay/ring_io.h` are derived
from it and carry that SPDX tag. Any firmware image that links the engine is
GPL-3.0-or-later when distributed. The licence for the rest of this directory
is the repository owner's decision.
