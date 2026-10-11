# Pure C: host-tested in test/run_tests.sh with its own FFT. On the S3 it
# uses esp-dsp's optimised FFT (checked against the host FFT, see
# lora_detect.c).
idf_component_register(SRCS "lora_detect.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp-dsp)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
