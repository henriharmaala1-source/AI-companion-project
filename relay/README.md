# c5relay — ESP32-C5 analog video relay control

Turns the drone into a 5.8 GHz analog video relay: an ESP32-C5 running
[C5VRX-5](https://github.com/Twotoz/C5VRX) receives another drone's video,
the FC overlays its OSD, and the drone's own VTX retransmits it on a
channel far away from the one being received. This package runs on the
Raspberry Pi and keeps the two channels paired.

```
source drone VTX (e.g. R1 5658) )))
        ↓
[C5 antenna: patch, facing forward]
XIAO ESP32-C5 (C5VRX-5) ── 6-bit resistor DAC ── CVBS ──→ FC CAM in (H743, AT7456E OSD)
        │ USB (power + console)                                │ VTX video out
Raspberry Pi 5 ── UART / MSP ──→ FC ── SmartAudio/Tramp ──→ VTX (TX500), e.g. B8 5866
                                                              ↓
                                             [VTX antenna: patch, facing back] ))) → goggles
```

## What it does

1. Refuses to start if the FC is armed: Betaflight only retunes the VTX
   while disarmed (`io/vtx.c`), so pairing happens on the ground.
2. Reads the VTX config and vtxtable from the FC over MSP.
3. Picks the receive channel: the one you give with `--rx`, or the best
   analog video found by stepping the C5 through all channels. The survey
   skips channels within `--min-sep` of the VTX's current frequency, so
   the C5 does not lock onto the relay's own transmitter.
4. Tunes the C5 there and sets the VTX to the vtxtable channel furthest
   from it, within `--vtx-range`. Power and pit mode are left unchanged.
5. With `--watch`, keeps running. If someone moves the C5 with its BOOT
   button onto a channel too close to the VTX, it moves the C5 back (armed)
   or re-pairs the VTX (disarmed).

## Usage

```
pip install pyserial
cd relay
python3 -m c5relay --c5 /dev/ttyACM0 --fc /dev/ttyAMA0 --rx R1 --save
python3 -m c5relay --c5 /dev/ttyACM0 --fc /dev/ttyAMA0 --watch   # survey, then guard
```

| Option | Default | |
|---|---|---|
| `--rx` | survey | source channel, e.g. `R1`, `L3` |
| `--min-sep` | 60 MHz | minimum RX/VTX separation. C5VRX captures ±20 MHz and analog FM video is ~20–30 MHz wide, so adjacent Raceband (37 MHz) is not enough |
| `--vtx-range` | 5725-5875 | EU licence-free band (25 mW). `any` or a wider range needs an amateur radio licence |
| `--save` | off | write the VTX channel to the FC's EEPROM |

Inside 5725–5875 MHz the best pairing is a source at the bottom (A8 5725,
B1 5733) and the relay VTX at the top (A1 5865, B8 5866): ~140 MHz apart.

A survey takes roughly 3–4 minutes (C5VRX prints status every 2 s, and
each channel needs a settle and a fresh reading).
Each C5 channel step is also saved to its flash, so do not run surveys in
a loop; give `--rx` once you know the source channel.

## Wiring

- **C5 → FC video:** build the resistor DAC from the
  [C5VRX README](https://github.com/Twotoz/C5VRX#wiring-the-resistor-dac)
  (D4..D9 with 8.2k / 3.9k / 2k / 1k / 470R / 240R, 200R load) and connect
  its output node to the FC's camera input instead of the camera. A common
  ground is required.
- **C5 → Pi:** USB-C from the Pi. It powers the XIAO and carries the
  console, which shows up as `/dev/ttyACM0`.
- **FC:** Betaflight with a working vtxtable and VTX control
  (SmartAudio/Tramp) and MSP on the UART the Pi uses. Only one process can
  own that UART, so run c5relay before the OSD service starts, or merge
  the two.
- **Antennas:** the XIAO's U.FL connector to a 5.8 GHz antenna (the FPC
  antenna in the box may be 2.4 GHz only). See the isolation notes below.

## Before the first flight

1. Flash C5VRX-5 and let it calibrate **with the VTX unpowered** (up to
   ~8 minutes; the C5 must not hear the relay's own VTX while it measures).
2. On the bench, check the picture through the FC: set C5VRX's OUTPUT level
   (`g` on its console) until the OSD sits correctly and the sync is stable.
   Whether the AT7456E locks reliably to C5VRX's output is **untested**.
3. Measure the isolation: with the VTX on at flight power, run ESPsoup or
   check C5VRX's `signal` on the receive channel with the source drone off.
   It should show no video.

## Limits

- **Self-interference is the hard problem, not this code.** The relay VTX
  arrives 60–75 dB stronger than the source drone. Frequency separation is
  only one layer. Back-to-back patch antennas, opposite polarisation
  (source and C5 LHCP, relay VTX and goggles RHCP), the battery between the
  antennas, and a shielded C5 are still needed. A VTX on another band
  (1.3/3.3 GHz) remains the more robust design.
- **No in-flight VTX changes** (Betaflight). In flight only the C5 can be
  moved, and only to channels compatible with the VTX.
- **C5VRX is experimental** (undocumented radio debug path, latency not
  measured, needs ESP-IDF 6.0.2's exact libphy for its filter).
- **Hardware-tested: no.** The tests run against fakes that copy C5VRX-5's
  console output (commit `24da232`) and Betaflight's MSP handlers. The CLI
  was also run end to end over pseudo-terminals with pyserial.
- C5VRX is GPL-3.0. This package contains none of its code; it only talks
  to it over the serial console.

## Tests

```
cd relay && python3 -m unittest
```
