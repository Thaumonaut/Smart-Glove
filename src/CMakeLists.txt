# ESP-IDF Main Component CMakeLists.txt
# This is required for ESP-IDF framework

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)
