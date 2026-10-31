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
2. **Chirp detector** (`lora_detect`). Between slices it takes a raw I/Q
   snapshot (~0.46 ms) and dechirps it at each ExpressLRS spreading factor,
   in both directions. LoRa collapses into one FFT peak for one direction
   only; Wi-Fi, Bluetooth and carriers don't. It reports the SF, which gives
   the ELRS rate (SF5 500/333 Hz, SF6 250 Hz, SF7 150/100 Hz, SF8 50 Hz). It
   cannot see FLRC rates (F500, F1000), which aren't LoRa. Presence only: no
   symbols, packets or IDs are recovered.

   How often: after each snapshot it waits twice what that snapshot cost
   (capture + scan), so chirp work takes at most a third of the time. A fast
   scan runs every slice, a slow one every few. The `lora` log's `step_ms`
   is that cost.

   Two tiers per snapshot. A **full** pass: 2 in 5 s give a verdict. A
   **weak** pass (lower score floor, ~1.5 dB more sensitive): it counts only
   when 3 snapshots in 5 s agree on SF and slope, since chance weak passes
   (~1 in 200 in a crowded band, host and simulator) land on any of 8 SF/slope
   pairs. A verdict resting on weak passes only says "weak" on screen and
   `"chirp_tier":"weak"` in the `verdict` log.

The screen says DETECTED if either detector does. The `verdict` log line
says which one (`grid`, `chirp`, `chirp+grid`).

**Hopping or fixed (raw I/Q only).** ELRS hops over 80 channels, so its chirp
hits in 5 s land on many of them. The watcher groups the hits by channel
(whole-MHz buckets, neighbours merged): `hop_clusters` in the `lora` log, and
`ch<N> hop` on the details line. With **5 or more hits all on one channel**
the LoRa is real but does not hop (a mesh node, an IoT sensor, maybe ELRS
binding): the screen says "LoRa on ONE channel (not ELRS)", a `lora_fixed`
event is logged, and the verdict is not ELRS. With fewer hits nothing
changes, so range is not affected. Cost: a fixed-channel LoRa device shows
a false ELRS for about a second at the start (the verdict needs 2 hits, this
test 5), and at the edge of range, with under 5 hits, it still can.
In the grid detector's "other hopping" state, the line says **Bluetooth-like**
when most narrow bursts sit on the whole-MHz grid.

## Other things it recognises (shown under the ELRS answer)

ELRS keeps the big red banner and the alert sound. Everything else seen at
the same time is a line in a list (`main/targets.c`), most important first,
and a `target` on/off event in the log. Presence and kind only, 2.4 GHz only:

| Line | How | From |
|---|---|---|
| **DJI-like OFDM link** (orange) | LTE-like cyclic prefix (15 or 30 kHz subcarriers: 66.7 / 33.3 µs symbols) | raw snapshots |
| **LoRa 1 channel, not ELRS** (orange) | chirps, all on one channel | raw snapshots |
| **Bluetooth-like / other hopper** (yellow) | narrow hops off the ELRS grid | spectrum |
| **wide signal, not Wi-Fi/DJI** (yellow) | steady wide activity, no OFDM prefix found | both |
| **microwave oven** (cyan) | on/off locked to the 50 Hz mains, in 2440–2480 MHz | spectrum |
| **Wi-Fi ch N** (grey) | 3.2 µs prefix (802.11a/g/n/ac, and Wi-Fi 6 beacons) | raw snapshots |

- **Microwave ovens** (`components/band_detect/band_watch.c`): a household
  oven radiates for about half of every mains cycle. Per 100 ms slice, the
  10 MHz window with most of the narrow activity in 2440–2480 MHz must switch
  at exactly 50 (or 60) Hz, be on 20–80 % of the time, show up in nearly
  every cycle, and hold most of the range's activity. Wi-Fi-size runs are
  left out (an access point overlaps the same range). Inverter ovens may not
  pulse at the mains rate and are **not** recognised.
- **DJI-like** (`components/band_detect/ofdm_detect.c`): each snapshot, one
  5 MHz slot is mixed to 0 Hz and decimated to 10 MS/s (3rd-order CIC). It is
  then multiplied by itself delayed by 3.2, 33.3 and 66.7 µs, and the
  products are folded by position in the symbol period. A cyclic prefix
  makes one timing window stand out. DJI's O3/O4 links are reported to use
  LTE-like OFDM at 15 and 30 kHz (one author's measurements, linked in the
  header). Phone networks use the same numerology, but not in this band. So
  the label is **DJI-like**, never "DJI": other drone links could look the
  same. Slots a narrow signal fills, or where the chirp scan saw LoRa in the
  same snapshot, are not scored. A slot needs a running mean of ≥ 5.5 over
  ≥ 6 scans, ~0 dB in the slot (host statistics).
- Positions come from the spectrum: one target per wide-activity run, at its
  centre. Two signals touching edge to edge (a DJI link right beside a Wi-Fi
  channel) are one run and are shown at their shared centre.

Logs: `band` every 2 s (oven rhythm `r`, `duty`, `cover`, passing `slices`,
and wide occupancy per slot), `ofdm` every 2 s (slot means per kind,
`narrow_skips`, `lora_skips`, `scan_ms`). The OFDM scan runs inside the
snapshot step but is not charged to the chirp duty cycle, so ELRS snapshots
are not made rarer.

**Signal strength.** For each chirp detection the `lora` log carries three
numbers:

- `snr_db`: dB above the noise in the 1.2 MHz baseband, the noise taken
  from the bands beside the channel. This is shown on screen. It hardly
  depends on the AGC, so it is the "how close" number. Host-tested to within
  1 dB, also behind a modelled analog filter (BUGLOG: it used to read ~23 dB
  high there).
- `dbfs`: dB relative to the ADC's full scale.
- `agc`: the gain index stamped in the snapshot.

Turning `dbfs` and `agc` into dBm needs a calibration: esp-sdr has no
gain-index-to-dB table. It could be done against the RSSI the ELRS receiver
itself reports, or by holding the gain fixed (`g`, arrows).

## Keys

`d` details · `c` clear · `+`/`-` threshold · `i` invert spectrum ·
`g` hardware AGC ↔ fixed gain · `;` (↑) / `.` (↓) fixed gain ±4 (leaves
AGC; starts from 40, upstream's default index; a PHY index, not dB) ·
`w` analog filter: PHY default ↔ widest (~69 MHz), a field test (see below) ·
`m` sound: normal → quiet → off (speaker: two rising tones when ELRS is
detected, two falling when it is lost, one tick at boot; **not yet tried on
hardware**, and silent on a Cardputer ADV, whose codec needs an I2C init this
build lacks) ·
`t` test alert · `r` log the next raw snapshot (`bb` baseband + `raw` words,
to check the dechirp offline). A fixed gain stays on the main view until `g`
returns to AGC.

**Receiver limits in the log** (from esp-sdr's own measurements,
`docs/rx-controls.md`):
- `diag` carries `rx_filter_code`/`rx_filter_mhz`, the analog filter in
  force. It is read back from the register on each run, and the MHz figure
  comes from upstream's S3 curve.
- `lora` carries `clip_pct`, the share of samples at the ADC rail. The
  Wi-Fi AGC does not protect against a strong continuous signal; if
  `clip_pct` climbs near a transmitter, use a fixed gain.
- `diag` also carries `floor_db`/`peak_db` (grid detector) and
  `empty_frames` (spectrum frames without an FFT, now ignored; see BUGLOG).

The details view shows `flt <MHz>` (`W` = widest) and `clip <%>`.

## Screen

ELRS detected: the red **ELRS DETECTED** banner (with a flash and two rising
tones), why (SF and ELRS rate, or the grid), the SNR with its change since
the detection began (+ = closer), and up to two lines of anything else seen.

No ELRS: a small green **ELRS NOT DETECTED** header, then up to six lines of
other things recognised (table above), then when ELRS was last detected and
the fixed gain if one is set. With nothing recognised it says so: "no
hopping signal seen on 2.4 GHz". It never says "all clear", because the
device sees 2.4 GHz only and misses a link below the noise. `d` toggles a
details view with the channel strip, detector numbers and the key reference.

## Status

| Part | State |
|---|---|
| `components/elrs_detect`: SPC1 frame decoder and ELRS detector | written, **host-tested** (ASan/UBSan, `-Werror`) on a synthetic model |
| `components/cp_keyboard`: GPIO-matrix keyboard port | written. Keymap **host-tested**, GPIO side untested |
| `components/sdr_engine`: esp-sdr engine wrapper, RAM sink, transport overlay | **compiles and links** with the pinned IDF, 0 warnings |
| `main/`: app loop, M5GFX screen, NDJSON log | **compiles and links**, 0 warnings (our code builds with `-Werror`) |
| Build files | done. Static data ends 6,048 bytes below the RF ring; app image 840 KB |
| `components/lora_detect`: LoRa chirp detector (dechirp, SF5–SF8, both directions) | **host-tested** on synthetic SX1280-style LoRa: 0 false alarms in 180 interference cases, 50 % detection per snapshot at about −3 dB in-band SNR (weak tier about −4.5 dB), works when hard-clipped. **Works on hardware** (below) |
| `main/chirp_watch.c`: raw snapshots between slices, evidence window, weak tier | in the simulator (17/17 scenarios). **Works on hardware** since the FFT-alignment fix (BUGLOG): detection returned and the range grew (owner's report, no numbers yet). Which of the weak tier, noise floor and adaptive rate helps, and by how much, is not measured; the speaker alerts are untested |
| `components/band_detect`: microwave ovens, wide occupancy, Wi-Fi vs DJI-like OFDM | **host-tested** (369 checks) on synthetic signals, and in the simulator. **Not tried on real air**: no real oven, DJI link or S3 scan time yet (BUGLOG) |
| `sim/`: whole-firmware simulator on a PC | **17/17 scenarios pass**, including a 10-minute crowded-band soak and core 1 falling behind. Found and fixed 4 bugs (see `../BUGLOG.md`) |
| On hardware | Chirp detector detects a 250 Hz ExpressLRS controller (SF6, down-chirps) and clears when it is off; 100–200 m from inside a closed car. Grid detector cannot separate ELRS from clutter. See `../BUGLOG.md` |

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
  lora_detect/   pure C: the chirp detector
  band_detect/   pure C: band_watch.c (ovens, wide occupancy), ofdm_detect.c
  cp_keyboard/   cp_keymap.c (pure, tested), cp_keyboard.c (driver/gpio)
  sdr_engine/    s3_rx.c (extracted from upstream receiver.c), sdr_sink.c,
                 overlay/ring_io.h (replaces upstream's USB transport)
main/            app_main.c, chirp_watch.c, targets.c (the list), ui.cpp, beep*.c
test/            run_tests.sh, test_elrs_detect.c, test_lora_detect.c,
                 test_band_detect.c, test_keymap.c, lora_synth.c (synthetic I/Q)
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

**No Python? Flash from the browser.** Espressif's esptool-js demo page,
<https://espressif.github.io/esptool-js/>, worked for this image on a
Cardputer (Chrome or Edge, Web Serial). Connect in download mode, set the
flash address to `0x0`, choose `cardputer-elrs-watch.bin`, and press
**Program** (there is no button called "Flash").

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
