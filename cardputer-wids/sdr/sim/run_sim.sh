#!/usr/bin/env bash
# Build and run the ELRS watch simulator on a PC. No ESP-IDF, no Cardputer.
#
#   ./sim/run_sim.sh               every scenario
#   ./sim/run_sim.sh elrs_250      one scenario
#   ./sim/run_sim.sh list
#
# The firmware's own sources are compiled unchanged with -Wall -Wextra -Werror
# under ASan + UBSan. M5GFX (third party) is compiled once, without -Werror.
# Results: sim/out/<scenario>/ (console log, screenshots, summary).
# Needs: gcc, g++, zlib headers (zlib1g-dev).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDR="$HERE/.."
WIDS="$SDR/../CardputerWIDS"
GFX="$SDR/third_party/M5GFX/src"
BUILD="$HERE/build"
OUT="$HERE/out"
mkdir -p "$BUILD/gfx" "$OUT"

if [ ! -f "$GFX/lgfx/v1/LGFXBase.cpp" ]; then
    echo "M5GFX submodule missing: git submodule update --init --recursive" >&2
    exit 2
fi

# --- M5GFX drawing core, host build (LGFX_LINUX_FB picks a platform with no
# SDL; only its in-memory sprite is used). Built once.
GFX_FLAGS=(-O1 -DLGFX_LINUX_FB -I"$GFX" -ffunction-sections -fdata-sections -w)
if [ ! -f "$BUILD/gfx/.done" ]; then
    echo "building M5GFX for the host (once)..."
    for f in lgfx_v1.cpp LGFXBase.cpp lgfx_fonts.cpp lgfx_v1_panel.cpp lgfx_v1_platforms.cpp; do
        g++ -std=c++17 "${GFX_FLAGS[@]}" -c "$GFX/lgfx/v1/$f" -o "$BUILD/gfx/${f%.cpp}.o"
    done
    while IFS= read -r f; do
        gcc "${GFX_FLAGS[@]}" -c "$f" -o "$BUILD/gfx/$(basename "${f%.c}").o"
    done < <(find "$GFX/lgfx/utility" "$GFX/lgfx/Fonts" -name '*.c')
    touch "$BUILD/gfx/.done"
fi

# --- the firmware and the simulator
WARN=(-Wall -Wextra -Werror)
SAN=(-g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
INC=(-I"$HERE/shim" -I"$HERE" -I"$SDR/main" -I"$SDR/components/elrs_detect/include"
     -I"$SDR/components/sdr_engine/include" -I"$SDR/components/sdr_engine"
     -I"$SDR/components/cp_keyboard/include" -I"$WIDS" -isystem "$GFX" -DLGFX_LINUX_FB)

C_SRCS=("$SDR/main/app_main.c" "$WIDS/wids_log.c"
        "$SDR/components/elrs_detect/spc1.c" "$SDR/components/elrs_detect/elrs_detect.c"
        "$SDR/components/sdr_engine/sdr_sink.c"
        "$HERE/rf_scene.c" "$HERE/sim_engine.c" "$HERE/sim_platform.c" "$HERE/sim_main.c")
CXX_SRCS=("$SDR/main/ui.cpp" "$HERE/sim_display.cpp")

OBJS=()
for f in "${C_SRCS[@]}"; do
    o="$BUILD/$(basename "${f%.c}").o"
    gcc -std=gnu11 "${WARN[@]}" "${SAN[@]}" "${INC[@]}" -c "$f" -o "$o"
    OBJS+=("$o")
done
for f in "${CXX_SRCS[@]}"; do
    o="$BUILD/$(basename "${f%.cpp}").o"
    g++ -std=c++17 "${WARN[@]}" "${SAN[@]}" "${INC[@]}" -c "$f" -o "$o"
    OBJS+=("$o")
done
g++ "${SAN[@]}" "${OBJS[@]}" "$BUILD"/gfx/*.o -Wl,--gc-sections -lz -lm -o "$BUILD/elrs_sim"

cd "$HERE"
# The firmware allocates once and never frees (app_main never returns), so
# leak reports at the end of a run are noise; ASan still checks every access.
export ASAN_OPTIONS="detect_leaks=0${ASAN_OPTIONS:+:$ASAN_OPTIONS}"
exec "$BUILD/elrs_sim" "${1:-all}" "$OUT"
