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
#include "analog_sensors.h"

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
 * Complete sensor state
 */
typedef struct {
    motion_data_t motion;
    flex_sensors_t flex;
    pressure_sensors_t pressure;
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
