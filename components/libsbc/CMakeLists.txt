# libsbc - BlueZ SBC codec implementation for ESP32
# Stable alternative to ESP-ADF SBC encoder

idf_component_register(
    SRCS "sbc.c"
         "sbc_primitives.c"
    INCLUDE_DIRS "."
    REQUIRES esp_common
)

# Define ARDUINO to use portable byte-swap macros instead of Linux byteswap.h
target_compile_definitions(${COMPONENT_LIB} PRIVATE ARDUINO=1)

# Compiler flags for optimal performance
target_compile_options(${COMPONENT_LIB} PRIVATE
    -O2
    -ffast-math
)
