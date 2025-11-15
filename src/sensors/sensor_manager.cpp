/**
 * Sensor Manager Implementation
 */

#include "sensor_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "SensorManager";

// Global sensor state (thread-safe via FreeRTOS task priorities)
static sensor_state_t g_sensor_state = {0};

// MPU6050 failure tracking
static int mpu_consecutive_failures = 0;
static const int MAX_CONSECUTIVE_FAILURES = 10;

/**
 * Sensor reading task
 */
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");

    // Wait for I2C to settle
    vTaskDelay(pdMS_TO_TICKS(200));
    
    int skip_counter = 0;

    while (1) {
        // Read flex sensors
        read_flex_sensors(&g_sensor_state.flex);
        
        // Read pressure sensors
        read_pressure_sensors(&g_sensor_state.pressure);

        // MPU6050 DISABLED - unreliable hardware connection
        // Skip the gyro/accelerometer readings to clean up serial output
        skip_counter++;
        g_sensor_state.motion.valid = false;

        // Log sensor values periodically
        static int log_counter = 0;
        // if (log_counter++ % 20 == 0) {  // Every ~1 second at 50ms interval
        //     ESP_LOGI(TAG, "Flex: I=%d M=%d P=%d | Pressure: Palm=%d Thumb=%d",
        //              g_sensor_state.flex.index, 
        //              g_sensor_state.flex.middle, 
        //              g_sensor_state.flex.pinky,
        //              g_sensor_state.pressure.palm,
        //              g_sensor_state.pressure.thumb);
        // }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

esp_err_t sensor_manager_init(void) {
    ESP_LOGI(TAG, "Initializing sensors...");
    
    // Initialize analog sensors
    esp_err_t ret = analog_sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize analog sensors");
        return ret;
    }
    
    // MPU6050 disabled - unreliable hardware connection
    // Skip initialization to avoid I2C error messages
    ESP_LOGI(TAG, "MPU6050 disabled (unreliable hardware)");
    g_sensor_state.motion.valid = false;
    
    ESP_LOGI(TAG, "Sensor manager initialized");
    return ESP_OK;
}

void sensor_manager_get_state(sensor_state_t *state) {
    // Simple copy - safe because we're reading from lower priority task
    *state = g_sensor_state;
}

void sensor_manager_start_task(void) {
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Sensor task created");
}
