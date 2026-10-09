# sdr_engine: esp-sdr's ESP32-S3 continuous-capture engine, built from the
# pinned submodule WITHOUT editing it, plus our receiver glue and RAM sink.
#
# How the transport is swapped: upstream ring_capture.c does
#   #include "ring_io.h"
# with quotes, and GCC always looks in the including file's own directory
# first. So ring_capture.c is copied into the build tree next to our own
# overlay/ring_io.h. configure_file() re-copies whenever upstream changes.

if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "sdr_engine only supports esp32s3 (the Cardputer)")
endif()

set(up "${CMAKE_CURRENT_LIST_DIR}/../../third_party/esp-sdr/main")
set(gen "${CMAKE_CURRENT_BINARY_DIR}/overlay")
if(NOT EXISTS "${up}/common/ring_capture.c")
    message(FATAL_ERROR "esp-sdr submodule missing: git submodule update --init --recursive")
endif()
configure_file("${up}/common/ring_capture.c" "${gen}/ring_capture.c" COPYONLY)
configure_file("${CMAKE_CURRENT_LIST_DIR}/overlay/ring_io.h" "${gen}/ring_io.h" COPYONLY)

set(upstream_srcs
    "${gen}/ring_capture.c"
    "${up}/common/rx_recalibration.c"
    "${up}/common/rx_calibration_state.c"
    "${up}/common/spectrum_stats.c"
    "${up}/targets/esp32s3/s3_fft_rnd.S"
    "${up}/targets/esp32s3/s3_unpack.S"
    "${up}/targets/esp32s3/s3_core1.S"
    "${up}/targets/esp32s3/s3_firdot.S")
set(our_srcs "s3_rx.c" "sdr_sink.c")

idf_component_register(
    SRCS ${our_srcs} ${upstream_srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "${gen}" "." "${up}/common" "${up}/targets/esp32s3"
    LDFRAGMENTS "linker.lf"
    PRIV_REQUIRES esp_wifi esp_phy esp_event esp_hw_support esp_timer heap esp_rom
                  soc hal freertos esp-dsp
    # Keep every object: the SRAM reservation and the --wrap targets are
    # reached through linker sections, not ordinary calls.
    WHOLE_ARCHIVE)

# Warnings are errors in OUR code. Upstream files keep upstream's flags.
set_source_files_properties(${our_srcs} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")

# Upstream guard: fail the link if static data or code reaches the RF ring.
target_link_options(${COMPONENT_LIB} INTERFACE "-T${up}/targets/esp32s3/sram_guard.ld")
# Upstream S3 PHY hooks (esp-sdr main/CMakeLists.txt, common/rx_calibration_link.cmake).
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=chip_v7_set_chan_ana"
    "-Wl,--wrap=set_rx_gain_cal_dc")
