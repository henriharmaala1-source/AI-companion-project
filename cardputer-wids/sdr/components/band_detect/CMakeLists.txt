# Microwave ovens and wide signals from spectrum frames (band_watch), and
# Wi-Fi vs DJI-like OFDM from raw snapshots (ofdm_detect). Pure C,
# host-tested in test/run_tests.sh.
idf_component_register(SRCS "band_watch.c" "ofdm_detect.c"
                       INCLUDE_DIRS "include"
                       REQUIRES lora_detect)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
