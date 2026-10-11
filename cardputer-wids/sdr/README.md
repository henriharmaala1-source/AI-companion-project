# ELRS watch — Cardputer firmware (work in progress)

Receive-only, 2.4 GHz-only presence detection of ExpressLRS links, using
[esp-sdr](https://github.com/ESPARGOS/esp-sdr)'s ESP32-S3 capture engine and
the Cardputer screen and keyboard. Design and milestones:
[`../docs/ELRS_SDR_PLAN.md`](../docs/ELRS_SDR_PLAN.md).

This is a separate ESP-IDF image from the Arduino Wi-Fi monitor in
`../CardputerWIDS`. Both can't run at once: one radio, one mode (constraint 2).

## Two detectors

1. **Grid detector** (`elrs_detect`). It works from 2 ms max-hold power spectra:
   narrow bursts on the 2400.4 + k MHz grid, spread evenly. It also catches
   FLRC rates. On the first hardware run it could not tell ELRS from clutter
   (see BUGLOG).
2. **Chirp detector** (`lora_detect`). Every second slice it takes a raw I/Q
   snapshot (~0.46 ms) and dechirps it at each ExpressLRS spreading factor,
   in both directions. LoRa collapses into one FFT peak for one direction
   only; Wi-Fi, Bluetooth and carriers don't. It reports the SF, which gives
   the ELRS rate (SF5 500/333 Hz, SF6 250 Hz, SF7 150/100 Hz, SF8 50 Hz). It
   cannot see FLRC rates (F500, F1000), which aren't LoRa. Presence only: no
   symbols, packets or IDs are recovered.

The screen says DETECTED if either detector does. The `verdict` log line
says which one (`grid`, `chirp`, `chirp+grid`).

**Signal strength.** For each chirp detection the `lora` log carries three
numbers:

- `snr_db`: dB above the noise in the 1.2 MHz baseband. This is shown on
  screen. It hardly depends on the AGC, so it is the "how close" number.
  Host-tested to within 0.1 dB.
- `dbfs`: dB relative to the ADC's full scale.
- `agc`: the gain index stamped in the snapshot.

Turning `dbfs` and `agc` into dBm needs a calibration: esp-sdr has no
gain-index-to-dB table. It could be done against the RSSI the ELRS receiver
itself reports, or by holding the gain fixed with `g`.

## Keys

`d` details · `c` clear · `+`/`-` threshold · `i` invert spectrum ·
`g` gain (AGC → fixed 10…60) · `t` test alert · `r` log the next raw
snapshot (`bb` baseband + `raw` words, to check the dechirp offline).

## Screen

The main view is only the answer: **ELRS DETECTED** (red, with a flash) or
**NOT DETECTED** (green), plus up to three lines. With a chirp detection
these are the SF and ELRS rate, the raw level in dBFS with its gain index,
and the SNR with its change since the detection began (+ = closer). The key
reference is in the details view. The line under it says which kind of "not
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
| Build files | done. Static data ends 9,712 bytes below the RF ring; app image 797 KB |
| `components/lora_detect`: LoRa chirp detector (dechirp, SF5–SF8, both directions) | written, **host-tested** on synthetic SX1280-style LoRa: 0 false alarms in 180 interference cases, detects down to +5 dB in-band SNR and when hard-clipped |
| `main/chirp_watch.c`: raw snapshots between slices, evidence window | in the simulator (12/12 scenarios). **Untested on hardware** |
| `sim/`: whole-firmware simulator on a PC | **12/12 scenarios pass**. Found and fixed 3 bugs (see `../BUGLOG.md`) |
| On hardware | **nothing has run on a Cardputer yet** |

The tests and the simulator show the logic and timing do what they claim on a
model of ELRS, Bluetooth, BLE and Wi-Fi taken from their specs. They do **not**
show that the Cardputer can see a real transmitter. That is milestone M0 in
the plan.

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
sim/             host simulator: run_sim.sh, scenarios, radio model, shims
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

## Simulate the whole firmware on a PC

```sh
./sim/run_sim.sh            # every scenario, about 20 s
./sim/run_sim.sh elrs_50    # one scenario
./sim/run_sim.sh list
```

Needs gcc, g++ and zlib headers. Nothing to flash.

**What runs for real:**

- `app_main.c` and `ui.cpp`, drawn by the real M5GFX code into a 240x135 canvas;
- the detector, the SPC1 decoder, the sink and `wids_log.c`;
- everything compiled with `-Werror` under ASan/UBSan.

**What is modelled:**

- the air: ELRS at four rates, Bluetooth audio, BLE and Wi-Fi (`sim/rf_scene.c`);
- esp-sdr's capture loop, reduced to its frame-emission rules, each cited to
  the upstream line (`sim/sim_engine.c`);
- time: a virtual clock;
- key presses: a script.

Each scenario checks:

- the console log is valid NDJSON;
- the verdict timeline: detection time, no false alarm, release;
- the screen, from its pixels.

It writes `sim/out/<scenario>/`: `log.ndjson`, screenshots (`film_*.png` every
5 s) and `summary.txt`.

**Not modelled:**

- the Cardputer's antenna, gain and AGC;
- strong-signal desense;
- other SX1280 links (TBS Tracer, Ghost);
- microwave ovens.

A pass here is necessary, not sufficient. M0 on real hardware decides.

## Licensing

esp-sdr is GPL-3.0-or-later. `s3_rx.c` and `overlay/ring_io.h` are derived
from it and carry that SPDX tag. Any firmware image that links the engine is
GPL-3.0-or-later when distributed. The licence for the rest of this directory
is the repository owner's decision.
