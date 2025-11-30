/**
 * MAX30102 Heart Rate / SpO2 Sensor Driver
 * 
 * I2C-based pulse oximeter and heart-rate sensor
 * Address: 0x57
 */

#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// MAX30102 I2C Address
#define MAX30102_ADDR 0x57

/**
 * Initialize MAX30102 sensor
 * - Detects device on I2C bus
 * - Performs device reset
 * - Configures sample rate, LED pulse width, and current
 * 
 * @return ESP_OK on success, ESP_FAIL if device not found
 */
esp_err_t max30102_init(void);

/**
 * Read sensor data from MAX30102 FIFO
 * 
 * @param red_led Pointer to store Red LED data
 * @param ir_led Pointer to store IR LED data
 * @return true if read successful, false on I2C error or timeout
 */
bool max30102_read(uint32_t *red_led, uint32_t *ir_led);

/**
 * Shutdown the sensor to save power
 */
esp_err_t max30102_shutdown(void);

/**
 * Wake up the sensor from shutdown
 */
esp_err_t max30102_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif // MAX30102_H
