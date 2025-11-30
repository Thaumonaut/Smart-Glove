/**
 * UI Manager Implementation
 */

#include "ui_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "../display.h"
#include "../bluetooth.h"
#include "../sensors/sensor_manager.h"

static const char *TAG = "UIManager";

// Current accelerometer values for display
static int16_t display_accel_x = 0;
static int16_t display_accel_y = 0;
static int16_t display_accel_z = 0;
static gesture_type_t latest_gesture = GESTURE_NONE;

/**
 * UI update task
 */
static void ui_task(void *pvParameters) {
    ESP_LOGI(TAG, "UI task started");

    while (1) {
        // Handle LVGL timer
        display_lvgl_tick();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t ui_manager_init(void) {
    ESP_LOGI(TAG, "Initializing UI...");
    
    // Initialize display hardware and LVGL
    display_init();
    
    ESP_LOGI(TAG, "UI initialized");
    return ESP_OK;
}

void ui_update_accel(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
    display_accel_x = accel_x;
    display_accel_y = accel_y;
    display_accel_z = accel_z;
}

void ui_update_all(void) {
    // Read latest sensors
    sensor_state_t sensors;
    sensor_manager_get_state(&sensors);

    // Update motion display if available
    if (sensors.motion.valid) {
        display_update_accel(sensors.motion.accel_x, sensors.motion.accel_y, sensors.motion.accel_z);
        display_update_gyro(sensors.motion.gyro_x, sensors.motion.gyro_y, sensors.motion.gyro_z);
    }
    
    // Update health display (HR/SpO2 or raw)
    display_update_health(sensors.health.valid,
                          sensors.health.heart_rate,
                          sensors.health.spo2,
                          sensors.health.red_led,
                          sensors.health.ir_led);
    
    // Update APDS9960 proximity and gesture
    if (sensors.gesture.valid) {
        display_update_apds_proximity(sensors.gesture.proximity);
        const char *dir_str = "";
        switch (sensors.gesture.gesture) {
            case APDS_DIR_UP: dir_str = "UP"; break;
            case APDS_DIR_DOWN: dir_str = "DOWN"; break;
            case APDS_DIR_LEFT: dir_str = "LEFT"; break;
            case APDS_DIR_RIGHT: dir_str = "RIGHT"; break;
            case APDS_DIR_NEAR: dir_str = "NEAR"; break;
            case APDS_DIR_FAR: dir_str = "FAR"; break;
            case APDS_DIR_NONE:
            default: dir_str = "--"; break;
        }
        display_update_apds_gesture(dir_str);
    } else {
        display_update_apds_proximity(0);
        display_update_apds_gesture("--");
    }
    
    // Update Bluetooth status
    char title[128] = {0};
    char artist[128] = {0};
    bool has_metadata = bluetooth_get_track_info(title, sizeof(title), artist, sizeof(artist));
    
    display_update_bluetooth_state(bluetooth_is_connected(),
                                   bluetooth_is_playing_audio(),
                                   bluetooth_is_call_active(),
                                   has_metadata ? title : NULL,
                                   has_metadata ? artist : NULL);

    // Update gesture label
    const char *gtext = "None";
    switch (latest_gesture) {
        case GESTURE_CALL_ME: gtext = "Call Me"; break;
        case GESTURE_EMERGENCY_SHAKE: gtext = "Emergency"; break;
        case GESTURE_SWIPE_UP: gtext = "Swipe Up"; break;
        case GESTURE_SWIPE_DOWN: gtext = "Swipe Down"; break;
        case GESTURE_TAP: gtext = "Tap"; break;
        case GESTURE_NONE:
        default: gtext = "None"; break;
    }
    display_update_gesture(gtext);
}

void ui_manager_start_task(void) {
    xTaskCreate(ui_task, "ui_task", 8192, NULL, 3, NULL);
    ESP_LOGI(TAG, "UI task created");
}

void ui_set_gesture(gesture_type_t gesture) {
    latest_gesture = gesture;
}
