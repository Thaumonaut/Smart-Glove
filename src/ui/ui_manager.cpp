/**
 * UI Manager Implementation
 */

#include "ui_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "../display.h"
#include "../sensors/sensor_manager.h"

static const char *TAG = "UIManager";

// Current accelerometer values for display
static int16_t display_accel_x = 0;
static int16_t display_accel_y = 0;
static int16_t display_accel_z = 0;

/**
 * UI update task
 */
static void ui_task(void *pvParameters) {
    ESP_LOGI(TAG, "UI task started");

    while (1) {
        // Update accelerometer display
        display_update_accel(display_accel_x, display_accel_y, display_accel_z);

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

void ui_manager_start_task(void) {
    xTaskCreate(ui_task, "ui_task", 8192, NULL, 3, NULL);
    ESP_LOGI(TAG, "UI task created");
}
