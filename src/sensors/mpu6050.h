/**
 * MPU6050 Gyroscope/Accelerometer Driver
 * 
 * I2C-based 6-axis motion sensor
 * Address: 0x68 (or 0x69 if AD0 is HIGH)
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize MPU6050 sensor
 * - Detects device on I2C bus
 * - Performs device reset
 * - Configures sample rate, DLPF, ranges
 * 
 * @return ESP_OK on success, ESP_FAIL if device not found
 */
esp_err_t mpu6050_init(void);

/**
 * Read all sensor data from MPU6050
 * 
 * @param accel_x Pointer to store accelerometer X-axis (±2g range, ~16384 = 1g)
 * @param accel_y Pointer to store accelerometer Y-axis
 * @param accel_z Pointer to store accelerometer Z-axis
 * @param gyro_x Pointer to store gyroscope X-axis (±250°/s range)
 * @param gyro_y Pointer to store gyroscope Y-axis
 * @param gyro_z Pointer to store gyroscope Z-axis
 * @return true if read successful, false on I2C error or timeout
 */
bool mpu6050_read(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                  int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
