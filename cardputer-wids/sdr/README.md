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
| `components/sdr_engine`: esp-sdr engine wrapper, RAM sink, transport overlay | **compiles and links** with the pinned IDF, 0 warnings |
| `main/`: app loop, M5GFX screen, NDJSON log | **compiles and links**, 0 warnings (our code builds with `-Werror`) |
| Build files | done. Static data ends 10,000 bytes below the RF ring; app image 796 KB |
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

## Build

Needs ESP-IDF at **exactly** commit `25fe69f946311abdaf9ad56591f25fedbc20ac98`
(6.2.0-dev). The project refuses any other commit unless you pass
`-DELRS_ALLOW_IDF_MISMATCH=ON`, because the radio engine uses private PHY ABIs.

```sh
git submodule update --init --recursive
# with that IDF commit installed and exported (. $IDF_PATH/export.sh):
idf.py -C cardputer-wids/sdr build
idf.py -C cardputer-wids/sdr -p /dev/ttyACM0 flash monitor
```

## Flash a prebuilt image (no IDF needed)

`cardputer-elrs-watch.bin` is one merged image: bootloader, partition table
and app, written at offset `0x0`, DIO, 8 MB.

```sh
pip install esptool
python -m esptool --chip esp32s3 -p /dev/ttyACM0 write-flash 0x0 cardputer-elrs-watch.bin
```

On Windows the port is `COMx`; on macOS it's `/dev/cu.usbmodem*`. If the
Cardputer doesn't enter download mode, hold **G0** while plugging in USB.
This overwrites whatever is on the device, including the Arduino Wi-Fi
monitor. Reflash that sketch to go back.

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
