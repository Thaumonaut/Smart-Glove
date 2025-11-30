/**
 * APDS9960 Gesture/Proximity/Light/Color Sensor Driver
 * I2C Address: 0x39
 */

#ifndef APDS9960_H
#define APDS9960_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APDS9960_ADDR 0x39

// Gesture directions
typedef enum {
    APDS_DIR_NONE = 0,
    APDS_DIR_LEFT = 1,
    APDS_DIR_RIGHT = 2,
    APDS_DIR_UP = 3,
    APDS_DIR_DOWN = 4,
    APDS_DIR_NEAR = 5,
    APDS_DIR_FAR = 6
} apds9960_gesture_t;

/**
 * Initialize APDS9960
 * Enables gesture detection with default settings
 * 
 * @return ESP_OK on success
 */
esp_err_t apds9960_init(void);

/**
 * Read proximity value
 * 
 * @param proximity Pointer to store proximity (0-255, higher = closer)
 * @return true if read successful
 */
bool apds9960_read_proximity(uint8_t *proximity);

/**
 * Read gesture if available
 * Non-blocking - returns APDS_DIR_NONE if no gesture
 * 
 * @return Detected gesture direction
 */
apds9960_gesture_t apds9960_read_gesture(void);

/**
 * Check if gesture is available
 * 
 * @return true if gesture data ready
 */
bool apds9960_gesture_available(void);

/**
 * Enable/disable gesture detection
 * 
 * @param enable true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t apds9960_enable_gesture(bool enable);

#ifdef __cplusplus
}
#endif

#endif // APDS9960_H
