/**
 * Smart Glove - Main Entry Point
 *
 * Assistive wearable for Periodic Paralysis patients
 * - Emergency alerts via gestures
 * - Bluetooth hands-free calling
 * - Health monitoring (HR/O2)
 * - Menu navigation with thumb gestures
 *
 * Hardware: IdeaSpark ESP32 (original, Bluetooth Classic)
 * Framework: ESP-IDF
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/i2c.h"

#include "config.h"

// ============================================================================
// LOGGING TAG
// ============================================================================
static const char *TAG = "SmartGlove";

// ============================================================================
// SYSTEM STATE
// ============================================================================
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_IN_CALL,
    STATE_ALERT,
    STATE_LOW_POWER
} system_state_t;

static system_state_t current_state = STATE_INIT;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
static void init_nvs(void);
static void init_gpio(void);
static void init_i2s(void);
static void init_i2c(void);
static void init_adc(void);
static void init_bluetooth(void);

// Task functions
static void sensor_task(void *pvParameters);
static void gesture_task(void *pvParameters);
static void audio_task(void *pvParameters);
static void ui_task(void *pvParameters);

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * Initialize Non-Volatile Storage (required for Bluetooth and WiFi)
 */
static void init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
}

/**
 * Initialize GPIO pins
 */
static void init_gpio(void) {
    // Configure vibration motor pin as output
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << VIBRATION_MOTOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Ensure vibration motor is off
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);

    ESP_LOGI(TAG, "GPIO initialized");
}

/**
 * Initialize I2S for audio (speaker output and microphone input)
 */
static void init_i2s(void) {
    // I2S configuration for speaker (port 0)
    i2s_config_t i2s_speaker_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t speaker_pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_SPKR_DO_IO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_PORT_SPEAKER, &i2s_speaker_config, 0, NULL);
    i2s_set_pin(I2S_PORT_SPEAKER, &speaker_pin_config);

    // I2S configuration for microphone (port 1)
    i2s_config_t i2s_mic_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t mic_pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_DI_IO
    };

    i2s_driver_install(I2S_PORT_MIC, &i2s_mic_config, 0, NULL);
    i2s_set_pin(I2S_PORT_MIC, &mic_pin_config);

    ESP_LOGI(TAG, "I2S initialized (speaker on port %d, mic on port %d)",
             I2S_PORT_SPEAKER, I2S_PORT_MIC);
}

/**
 * Initialize I2C bus for sensors
 */
static void init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ_HZ,
        },
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", I2C_SDA_IO, I2C_SCL_IO);
}

/**
 * Initialize ADC for flex sensors and pressure pads
 */
static void init_adc(void) {
    // Configure ADC1 (works with WiFi/BT, unlike ADC2)
    adc1_config_width(ADC_WIDTH);

    // Configure each ADC channel
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN);  // GPIO 36 - Index
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN);  // GPIO 39 - Middle
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN);  // GPIO 34 - Pinky
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN);  // GPIO 35 - Pressure
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN);  // GPIO 33 - Pressure 2

    ESP_LOGI(TAG, "ADC initialized for flex sensors and pressure pads");
}

/**
 * Initialize Bluetooth Classic (A2DP/HFP)
 */
static void init_bluetooth(void) {
    // TODO: Implement Bluetooth A2DP and HFP initialization
    // This will be done in separate bluetooth module
    ESP_LOGI(TAG, "Bluetooth initialization placeholder (see test/bt_a2dp_test.cpp)");
}

// ============================================================================
// FREERTOS TASKS
// ============================================================================

/**
 * Sensor reading task
 * Reads flex sensors, pressure pads, gyro/accel, HR/O2 at regular intervals
 */
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");

    while (1) {
        // Read flex sensors (ADC)
        int flex_index = adc1_get_raw(ADC1_CHANNEL_0);
        int flex_middle = adc1_get_raw(ADC1_CHANNEL_3);
        int flex_pinky = adc1_get_raw(ADC1_CHANNEL_6);
        int pressure = adc1_get_raw(ADC1_CHANNEL_7);

        // Log sensor values periodically
        static int log_counter = 0;
        if (log_counter++ % 20 == 0) {  // Every 1 second at 50ms interval
            ESP_LOGI(TAG, "Flex: I=%d M=%d P=%d | Pressure=%d",
                     flex_index, flex_middle, flex_pinky, pressure);
        }

        // TODO: Read I2C sensors (APDS9960, MAX30102, MPU6050)
        // TODO: Process sensor data and update system state

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

/**
 * Gesture detection task
 * Analyzes sensor data to recognize gestures
 */
static void gesture_task(void *pvParameters) {
    ESP_LOGI(TAG, "Gesture task started");

    while (1) {
        // TODO: Implement gesture recognition logic
        // - "Call me" gesture detection
        // - Thumb swipe navigation via APDS9960
        // - Emergency alert gestures

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Audio handling task
 * Manages I2S audio streaming during calls
 */
static void audio_task(void *pvParameters) {
    ESP_LOGI(TAG, "Audio task started");

    while (1) {
        // TODO: Handle audio streaming when in call
        // - Route mic input to Bluetooth
        // - Route Bluetooth audio to speaker

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * UI/Display task
 * Manages LVGL display updates
 */
static void ui_task(void *pvParameters) {
    ESP_LOGI(TAG, "UI task started");

    while (1) {
        // TODO: Initialize and update LVGL display
        // - Home screen
        // - Menu navigation
        // - Call status
        // - Alert confirmation

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Smart Glove - Assistive Wearable Device");
    ESP_LOGI(TAG, "For Periodic Paralysis patients");
    ESP_LOGI(TAG, "===========================================");

    // Initialize hardware
    ESP_LOGI(TAG, "Initializing hardware...");
    init_nvs();
    init_gpio();
    init_i2s();
    init_i2c();
    init_adc();
    init_bluetooth();

    // Test vibration motor
    ESP_LOGI(TAG, "Testing vibration motor...");
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(VIBRATION_DURATION_MS));
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);

    // Create FreeRTOS tasks
    ESP_LOGI(TAG, "Creating tasks...");

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(gesture_task, "gesture_task", 4096, NULL, 4, NULL);
    xTaskCreate(audio_task, "audio_task", 4096, NULL, 6, NULL);
    xTaskCreate(ui_task, "ui_task", 8192, NULL, 3, NULL);

    current_state = STATE_IDLE;
    ESP_LOGI(TAG, "System ready - entering idle state");

    // Main loop (optional - tasks run independently)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
