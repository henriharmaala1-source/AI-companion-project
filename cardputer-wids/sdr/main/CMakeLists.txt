# main: app loop, screen, NDJSON log. wids_log.c is shared with the Arduino
# Wi-Fi monitor so both modes write the same log format.
set(wids "${CMAKE_CURRENT_LIST_DIR}/../../CardputerWIDS")

idf_component_register(
    SRCS "app_main.c" "ui.cpp" "${wids}/wids_log.c"
    INCLUDE_DIRS "." "${wids}"
    PRIV_REQUIRES sdr_engine elrs_detect cp_keyboard M5GFX
                  nvs_flash esp_driver_usb_serial_jtag esp_timer heap log freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
