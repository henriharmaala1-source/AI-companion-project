# ELRS watch for the Cardputer: ESP-IDF project.
#
# MUST be built with the ESP-IDF commit esp-sdr pins for the S3 (its
# firmware-targets.json): the engine depends on private PHY ABIs that change
# between IDF versions. A mismatch stops the build unless you override it.
cmake_minimum_required(VERSION 3.16)

set(ELRS_IDF_PIN "25fe69f946311abdaf9ad56591f25fedbc20ac98")
option(ELRS_ALLOW_IDF_MISMATCH "Build with a different ESP-IDF commit (RF may misbehave)" OFF)
execute_process(COMMAND git -C "$ENV{IDF_PATH}" rev-parse HEAD
                OUTPUT_VARIABLE idf_head OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE idf_git_rc ERROR_QUIET)
if(NOT idf_head STREQUAL ELRS_IDF_PIN)
    if(ELRS_ALLOW_IDF_MISMATCH)
        message(WARNING "ESP-IDF is '${idf_head}', expected ${ELRS_IDF_PIN}. Continuing as asked.")
    else()
        message(FATAL_ERROR "ESP-IDF is '${idf_head}', esp-sdr needs ${ELRS_IDF_PIN} "
                            "(6.2.0-dev). Check out that commit, or pass "
                            "-DELRS_ALLOW_IDF_MISMATCH=ON knowing the RF path may break.")
    endif()
endif()

# No component registry downloads: everything is pinned in third_party/.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/third_party/esp-sdr/components/esp-dsp"
    "${CMAKE_CURRENT_LIST_DIR}/third_party/M5GFX")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(cardputer_elrs_watch)
