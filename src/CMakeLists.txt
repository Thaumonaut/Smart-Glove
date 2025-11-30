# ESP-IDF Main Component CMakeLists.txt
# This is required for ESP-IDF framework

idf_component_register(
    SRCS
        "main.cpp"
        "display.cpp"
        "bluetooth.cpp"
        "sensors/mpu6050.cpp"
        "sensors/max30102.cpp"
        "sensors/apds9960.cpp"
        "sensors/analog_sensors.cpp"
        "sensors/sensor_manager.cpp"
        "gestures/gesture_detector.cpp"
        "audio/audio_manager.cpp"
        "audio/mic_manager.cpp"
        "ui/ui_manager.cpp"
    INCLUDE_DIRS
        "."
        "sensors"
        "gestures"
        "audio"
        "ui"
    REQUIRES
        esp_adc
        driver
        nvs_flash
        bt
        lvgl
        esp_audio_codec
)
