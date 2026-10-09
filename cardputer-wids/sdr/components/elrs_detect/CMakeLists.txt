idf_component_register(SRCS "spc1.c" "elrs_detect.c"
                       INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
