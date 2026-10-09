#!/usr/bin/env bash
# Host tests for the hardware-independent parts of the ELRS watch firmware.
# No ESP-IDF needed. -Wall -Wextra -Werror, ASan + UBSan.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
C="$HERE/../components"
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
FLAGS=(-std=gnu11 -Wall -Wextra -Werror -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer)
gcc "${FLAGS[@]}" -I"$C/elrs_detect/include" "$HERE/test_elrs_detect.c" \
    "$C/elrs_detect/elrs_detect.c" "$C/elrs_detect/spc1.c" -lm -o "$OUT/elrs"
gcc "${FLAGS[@]}" -I"$C/cp_keyboard/include" "$HERE/test_keymap.c" \
    "$C/cp_keyboard/cp_keymap.c" -o "$OUT/keymap"
"$OUT/elrs"
"$OUT/keymap"
