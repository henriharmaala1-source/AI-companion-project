#!/usr/bin/env bash
#
# Host-side tests for the parsing code. No hardware, no ESP-IDF needed.
#
# The frame parser is the only part of this project that reads bytes straight
# off the air, so it is the part worth testing hardest. Frames are built in
# exactly-sized heap allocations and the whole thing runs under
# AddressSanitizer: if a malformed information element ever walks off the end
# of a buffer, this fails loudly instead of corrupting a log at 3am.
#
# The stubs/ directory mirrors the ESP-IDF v5.5.5 signatures and the real
# bitfield widths of wifi_pkt_rx_ctrl_t. It is NOT a substitute for building
# against the actual core - it is a fast correctness check for the C logic.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$HERE/../CardputerWIDS"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

echo "== compiling each translation unit with -Wall -Wextra -Werror =="
for f in wids_log.c wids_platform.c wids_monitor.c; do
    gcc -c -std=gnu99 -Wall -Wextra -Werror \
        -I"$HERE/stubs" -I"$SKETCH" "$SKETCH/$f" -o "$OUT/${f%.c}.o"
    echo "  OK  $f"
done

echo
echo "== parser tests under AddressSanitizer + UBSan =="
gcc -std=gnu99 -Wall -Wextra -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$HERE/stubs" -I"$SKETCH" \
    "$HERE/test_parser.c" "$SKETCH/wids_log.c" -o "$OUT/test_parser"
"$OUT/test_parser"
