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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "config.h"
#include "bluetooth.h"

// Module headers
#include "sensors/sensor_manager.h"
#include "gestures/gesture_detector.h"
#include "audio/audio_manager.h"
#include "ui/ui_manager.h"
#include "display.h"

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

// I2C mutex for preventing bus conflicts between display and sensors
SemaphoreHandle_t i2c_mutex = NULL;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
static void init_nvs(void);
static void init_gpio(void);
static void init_i2c(void);
static void on_gesture_detected(gesture_type_t gesture);

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

// ============================================================================
// GESTURE CALLBACK
// ============================================================================

/**
 * Handle detected gestures
 */
static void on_gesture_detected(gesture_type_t gesture) {
    switch (gesture) {
        case GESTURE_CALL_ME:
            ESP_LOGI(TAG, "Gesture detected: CALL ME");
            // TODO: Trigger phone call
            break;
        case GESTURE_EMERGENCY_SHAKE:
            ESP_LOGI(TAG, "Gesture detected: EMERGENCY");
            // TODO: Send emergency alert
            break;
        case GESTURE_SWIPE_UP:
            ESP_LOGI(TAG, "Gesture detected: SWIPE UP");
            // TODO: Navigate menu up
            break;
        case GESTURE_SWIPE_DOWN:
            ESP_LOGI(TAG, "Gesture detected: SWIPE DOWN");
            // TODO: Navigate menu down
            break;
        default:
            break;
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

    // Create I2C mutex before hardware init
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
    }

    // Initialize low-level hardware
    ESP_LOGI(TAG, "Initializing hardware...");
    init_nvs();
    init_gpio();
    init_i2c();

    // Test vibration motor
    ESP_LOGI(TAG, "Testing vibration motor...");
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(VIBRATION_DURATION_MS));
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);

    // Initialize modules
    ESP_LOGI(TAG, "Initializing modules...");
    sensor_manager_init();
    gesture_detector_init(on_gesture_detected);
    
    // Initialize Bluetooth FIRST (before I2S to avoid resource conflicts)
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "##############################################");
    ESP_LOGI(TAG, "### STARTING BLUETOOTH INITIALIZATION ###");
    ESP_LOGI(TAG, "##############################################");
    bluetooth_init();
    ESP_LOGI(TAG, "##############################################");
    ESP_LOGI(TAG, "### BLUETOOTH READY - YOU CAN PAIR NOW ###");
    ESP_LOGI(TAG, "##############################################");
    ESP_LOGI(TAG, "");
    
    // Initialize audio after Bluetooth
    audio_manager_init();
    
    // Initialize microphone for HFP uplink (uses different clock source than speaker)
    bluetooth_init_microphone();
    
    ui_manager_init();
    
    // Update display with initial Bluetooth status (now that display is ready)
    display_set_bluetooth_status(bluetooth_is_connected());

    // Start all tasks
    ESP_LOGI(TAG, "Starting tasks...");
    sensor_manager_start_task();
    gesture_detector_start_task();
    audio_manager_start_task();
    ui_manager_start_task();

    current_state = STATE_IDLE;
    ESP_LOGI(TAG, "System ready - entering idle state");

    // Main loop (optional - tasks run independently)
    while (1) {
        // Get sensor state for display updates
        sensor_state_t sensors;
        sensor_manager_get_state(&sensors);
        
        if (sensors.motion.valid) {
            ui_update_accel(sensors.motion.accel_x, 
                           sensors.motion.accel_y, 
                           sensors.motion.accel_z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
