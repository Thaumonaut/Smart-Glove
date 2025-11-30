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

        // Read MPU6050 (with soft-failure tolerance)
        int16_t ax, ay, az, gx, gy, gz;
        if (mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz)) {
            g_sensor_state.motion.accel_x = ax;
            g_sensor_state.motion.accel_y = ay;
            g_sensor_state.motion.accel_z = az;
            g_sensor_state.motion.gyro_x = gx;
            g_sensor_state.motion.gyro_y = gy;
            g_sensor_state.motion.gyro_z = gz;
            g_sensor_state.motion.valid = true;
            mpu_consecutive_failures = 0;
        } else {
            mpu_consecutive_failures++;
            if (mpu_consecutive_failures > MAX_CONSECUTIVE_FAILURES) {
                g_sensor_state.motion.valid = false;
            }
        }

        // Read MAX30102
        if (max30102_read(&g_sensor_state.health.red_led, &g_sensor_state.health.ir_led)) {
            g_sensor_state.health.valid = true;
            // Simple placeholder for HR/SpO2 calculation
            // In a real implementation, we would need a buffer and algorithm here
            g_sensor_state.health.heart_rate = 0.0f; 
            g_sensor_state.health.spo2 = 0.0f;
            // Log when finger detected (high IR value)
            static uint32_t last_ir = 0;
            if (g_sensor_state.health.ir_led > 50000 && last_ir < 50000) {
                ESP_LOGI(TAG, "MAX30102 - Finger detected! IR=%lu Red=%lu", 
                         g_sensor_state.health.ir_led, g_sensor_state.health.red_led);
            }
            last_ir = g_sensor_state.health.ir_led;
        } else {
            g_sensor_state.health.valid = false;
        }

        // Read APDS9960 proximity
        if (apds9960_read_proximity(&g_sensor_state.gesture.proximity)) {
            g_sensor_state.gesture.valid = true;
            // Log proximity changes
            static uint8_t last_prox = 0;
            if (abs((int)g_sensor_state.gesture.proximity - (int)last_prox) > 10) {
                ESP_LOGI(TAG, "APDS Proximity: %d", g_sensor_state.gesture.proximity);
                last_prox = g_sensor_state.gesture.proximity;
            }
        } else {
            g_sensor_state.gesture.valid = false;
        }
        
        // Check for gesture (non-blocking)
        if (apds9960_gesture_available()) {
            g_sensor_state.gesture.gesture = apds9960_read_gesture();
            if (g_sensor_state.gesture.gesture != APDS_DIR_NONE) {
                const char *dir_name[] = {"NONE", "LEFT", "RIGHT", "UP", "DOWN", "NEAR", "FAR"};
                ESP_LOGI(TAG, "*** APDS GESTURE: %s ***", dir_name[g_sensor_state.gesture.gesture]);
            }
        } else {
            g_sensor_state.gesture.gesture = APDS_DIR_NONE;
        }

        // Log sensor values periodically
        static int log_counter = 0;
        if (log_counter++ % 40 == 0) {  // Every ~2 seconds at 50ms interval
             ESP_LOGI(TAG, "Sensors - Flex: I=%d M=%d P=%d | Pressure: Palm=%d Thumb=%d | HR: Red=%lu IR=%lu | Motion valid=%d",
                      g_sensor_state.flex.index, 
                      g_sensor_state.flex.middle, 
                      g_sensor_state.flex.pinky,
                      g_sensor_state.pressure.palm,
                      g_sensor_state.pressure.thumb,
                      g_sensor_state.health.red_led,
                      g_sensor_state.health.ir_led,
                      g_sensor_state.motion.valid);
        }

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
    
    // Initialize MPU6050 (soft-fail is acceptable)
    ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize MPU6050 (optional): %s", esp_err_to_name(ret));
        g_sensor_state.motion.valid = false;
    } else {
        g_sensor_state.motion.valid = true;
    }

    // Initialize MAX30102
    ret = max30102_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize MAX30102 (optional)");
        // Don't return error, just mark as invalid
        g_sensor_state.health.valid = false;
    } else {
        g_sensor_state.health.valid = true;
    }
    
    // Initialize APDS9960
    ret = apds9960_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize APDS9960 (optional): %s", esp_err_to_name(ret));
        g_sensor_state.gesture.valid = false;
    } else {
        g_sensor_state.gesture.valid = true;
    }
    
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
