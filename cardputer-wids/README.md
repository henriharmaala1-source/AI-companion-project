# Cardputer WIDS

A passive 2.4 GHz wireless intrusion-detection tool for the M5Stack Cardputer
(ESP32-S3). It sits at a fixed point, learns what the local RF environment
normally looks like, and flags changes that suggest an attack on the network or
a surveillance device planted nearby.

## Scope — read this first

**This is one layer, not a shield.**

It can see an attacker operating RF near the house: a spoofed AP, an evil twin,
a deauth/MITM rig, a planted 2.4 GHz device.

It **cannot** see the intrusion path a capable adversary actually uses — a
compromised mailbox, an implant on a laptop or phone, a companion device
quietly linked to a messaging account. None of that touches the air near your
house. Run this alongside hardware security keys and canary tokens, not instead
of them.

**2.4 GHz only.** The Cardputer has a single 2.4 GHz radio and no 5 GHz
capability whatsoever. A quiet screen means *"nothing seen on 2.4"* — never
*"all clear"*. The UI and the logs both say so, on purpose.

## What it does and does not do

**Receives only.** No deauth, no beacon spam, no jamming, no evil-portal
credential capture, no transmit-side anything. The monitor runs in
`WIFI_MODE_NULL` — the radio is on, but it never associates and never sends.
Anything that would attack or phish a third party is out of scope by design,
not by oversight.

**EU / ETSI regulatory domain**, channels 1–13, set with
`WIFI_COUNTRY_POLICY_MANUAL`. Not `AUTO` — auto would let a nearby AP's country
information element move our channel set, and that is exactly the sort of thing
an attacker controls.

**Short data retention.** MAC addresses are held only as long as detection
needs them. Nothing is uploaded anywhere; there is no network client in this
firmware at all.

## Two modes, never both at once

One radio cannot hop the whole band and hold a SoftAP on a fixed channel at the
same time. The modes are mutually exclusive and switched from the keyboard.

| Mode | State | What it does |
|---|---|---|
| **Monitor** | working | Promiscuous capture, channel hop 1–13, management-frame parsing, NDJSON out |
| **Honeypot** | roadmap step 6 | WPA2 SoftAP on a fixed channel; logs association attempts and handshake failures |

Pressing `h` today prints "not built yet" rather than doing nothing quietly.

## Status against the roadmap

| # | Step | State |
|---|---|---|
| 1 | Build + serial + PSRAM confirmed | **code complete, NOT yet run on hardware** |
| 2 | RSN AKM parsing (WPA3 SAE vs WPA2 downgrade) | not started |
| 3 | RSSI-variance gate on NEW_AP | not started |
| 4 | LCD + keyboard UI | working (counters, mode keys, alert flash + beep) |
| 5 | SD logging | not started — sink #2 slots into `wids_log.c` |
| 6 | Honeypot as mode 2 | not started |
| 7 | Probe-request client fingerprinting | not started |

Step 1 is written and tested on a host, but **nobody has flashed it to a
Cardputer yet**. Until that happens it is theory. The alert detectors from
steps 2–3 do not exist, so the only things on screen are frame counters.

## Layout

```
CardputerWIDS/
  CardputerWIDS.ino   Arduino/C++: hardware bring-up, LCD, keyboard, serial sink
  wids_monitor.c/.h   promiscuous capture, 802.11 parsing, channel hop
  wids_log.c/.h       NDJSON emitter with pluggable sinks
  wids_platform.c/.h  boot-time chip / PSRAM / flash probe
  wids_config.h       tunables in one place
test/
  run_tests.sh        host build + parser tests, no hardware needed
tools/
  build.sh            arduino-cli build with the warning flags
BUGLOG.md             gotchas, with symptoms and fixes
```

Only the `.ino` is C++. Everything with logic in it is plain ESP-IDF-style C,
compiled as C by Arduino and called across `extern "C"`.

## Building

Arduino IDE works for flashing and the serial monitor. Board settings that
matter:

- **Board:** M5Cardputer
- **PSRAM:** OPI PSRAM — required on an N16R8 module. Check the `{"ev":"boot"}`
  line afterwards; it reports what was actually found.

For a build with the project's warning flags, use `tools/build.sh`
(arduino-cli). The Arduino IDE cannot set per-sketch compiler flags, so the
`-Wall -Wextra` discipline is enforced two ways:

- `test/run_tests.sh` compiles every `.c` with `-Wall -Wextra -Werror` against
  stub headers — strict, and genuinely enforceable.
- `tools/build.sh` passes the same flags to the real build, where the Arduino
  core and M5 libraries add third-party noise you will have to grep past.

## Testing without hardware

```sh
./test/run_tests.sh
```

Compiles each translation unit with `-Wall -Wextra -Werror`, then runs the
frame parser under AddressSanitizer and UBSan against deliberately hostile
input: information elements that lie about their length, runt frames, SSIDs
containing JSON metacharacters, non-ASCII SSIDs, maximum-length SSIDs, and
frames flagged with receive errors. Frames are built in exactly-sized heap
allocations, so a single byte of overrun fails the run.

The parser is the only code here that reads bytes straight off the air, which
makes it the only code an attacker can aim malformed input at directly.

## Log format

NDJSON, one object per line, identical across both modes:

```json
{"t":1234567,"ev":"boot","chip":"ESP32-S3","psram":true,"psram_bytes":8388608,"band":"2.4GHz-only"}
{"t":1234890,"ev":"frame","sub":"beacon","ch":6,"rssi":-42,"sa":"ab:...","bssid":"cd:...","ssid":"TestNet"}
```

`t` is **microseconds since boot**. There is no battery-backed RTC on the base
unit, so there is no wall-clock time to record and none is invented.

SSIDs are escaped to printable ASCII, with anything else as `\u00XX`. That is
lossy for legitimate UTF-8, so when an SSID is not pure printable ASCII the raw
bytes also go out as `ssid_hex`. **The hex is the ground truth; the string is
for reading.** An SSID is attacker-controlled input and is treated that way.
