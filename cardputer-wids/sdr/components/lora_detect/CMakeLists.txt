# Pure C, no ESP-IDF dependency: host-tested in test/run_tests.sh.
idf_component_register(SRCS "lora_detect.c"
                       INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
