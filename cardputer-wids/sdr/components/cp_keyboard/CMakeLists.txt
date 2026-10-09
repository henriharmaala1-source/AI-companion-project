idf_component_register(SRCS "cp_keymap.c" "cp_keyboard.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_driver_gpio esp_rom)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
