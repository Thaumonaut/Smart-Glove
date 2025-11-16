# ESP Audio Codec Component - Prebuilt Library Wrapper
idf_component_register(
    INCLUDE_DIRS
        "include"
        "include/decoder"
        "include/decoder/impl"
        "include/encoder"
        "include/encoder/impl"
        "include/simple_dec"
        "include/simple_dec/impl"
)

# Link prebuilt library
add_prebuilt_library(esp_audio_codec "${CMAKE_CURRENT_SOURCE_DIR}/lib/libesp_audio_codec.a")
target_link_libraries(${COMPONENT_LIB} INTERFACE esp_audio_codec)
