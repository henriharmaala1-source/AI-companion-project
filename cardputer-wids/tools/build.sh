#!/usr/bin/env bash
#
# Build the sketch with arduino-cli and the warning flags the project expects.
#
# Why arduino-cli and not just the IDE: the IDE has no way to add per-sketch
# compiler flags, so "-Wall -Wextra clean" is unenforceable there. arduino-cli
# takes --build-property, which is how the acceptance criterion survives the
# move to Arduino. The IDE still works fine for flashing and the serial monitor.
#
# NOTE ON WARNINGS: these flags apply to the whole build, and the Arduino core
# plus the M5 libraries are not warning-clean under -Wextra. Expect third-party
# noise. Grep the output for our own files:
#
#     ./tools/build.sh 2>&1 | grep -E 'CardputerWIDS/wids_|CardputerWIDS\.ino'
#
# For the C logic specifically, test/run_tests.sh compiles it with
# -Wall -Wextra -Werror in isolation, which IS enforceable.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$HERE/../CardputerWIDS"

# The exact FQBN depends on the M5Stack board package version you installed, so
# discover it rather than trusting a value copied from anywhere (this file
# included). Override by exporting FQBN before running.
if [[ -z "${FQBN:-}" ]]; then
    echo "== looking up the Cardputer FQBN =="
    arduino-cli board listall 2>/dev/null | grep -i cardputer || {
        echo "No Cardputer board found. Install the M5Stack package first:"
        echo "  arduino-cli config add board_manager.additional_urls \\"
        echo "    https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json"
        echo "  arduino-cli core update-index"
        echo "  arduino-cli core install m5stack:esp32"
        exit 1
    }
    echo
    echo "Set FQBN to the identifier from the right-hand column above, e.g.:"
    echo "  FQBN=m5stack:esp32:m5stack_cardputer ./tools/build.sh"
    exit 1
fi

# PSRAM must be enabled or the AP baseline table has nowhere to live. The menu
# option name also comes from the board package - check it with:
#   arduino-cli board details --fqbn "$FQBN"
BOARD_OPTS="${BOARD_OPTS:-}"

echo "== building $SKETCH for $FQBN =="
arduino-cli compile \
    --fqbn "${FQBN}${BOARD_OPTS}" \
    --build-property "compiler.c.extra_flags=-Wall -Wextra" \
    --build-property "compiler.cpp.extra_flags=-Wall -Wextra" \
    --warnings all \
    "$SKETCH"

echo
echo "Flash with:  arduino-cli upload --fqbn \"$FQBN\" -p /dev/ttyACM0 \"$SKETCH\""
echo "Monitor with: arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200"
