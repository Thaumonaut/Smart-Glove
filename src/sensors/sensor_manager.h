/**
 * Sensor Manager
 * 
 * Coordinates all sensor reading:
 * - MPU6050 gyro/accelerometer
 * - Flex sensors
 * - Pressure pads
 * - Heart rate/SpO2 (future)
 * - Proximity sensor (future)
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include "esp_err.h"
#include "mpu6050.h"
#include "max30102.h"
#include "analog_sensors.h"
#include "apds9960.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Motion sensor data
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    bool valid;  // true if data was successfully read
} motion_data_t;

/**
 * Heart rate / SpO2 data
 */
typedef struct {
    uint32_t red_led;
    uint32_t ir_led;
    float heart_rate; // Calculated heart rate (placeholder for now)
    float spo2;       // Calculated SpO2 (placeholder for now)
    bool valid;
} health_data_t;

/**
 * APDS9960 gesture/proximity data
 */
typedef struct {
    uint8_t proximity;           // 0-255, higher = closer
    apds9960_gesture_t gesture;  // Last detected gesture
    bool valid;
} gesture_data_t;

/**
 * Complete sensor state
 */
typedef struct {
    motion_data_t motion;
    flex_sensors_t flex;
    pressure_sensors_t pressure;
    health_data_t health;
    gesture_data_t gesture;
} sensor_state_t;

/**
 * Initialize all sensors
 * 
 * @return ESP_OK if all sensors initialized successfully
 */
esp_err_t sensor_manager_init(void);

/**
 * Get latest sensor readings
 * Non-blocking - returns last known values
 * 
 * @param state Pointer to store sensor state
 */
void sensor_manager_get_state(sensor_state_t *state);

/**
 * Start sensor reading task
 * Creates FreeRTOS task that continuously reads sensors
 */
void sensor_manager_start_task(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_MANAGER_H
