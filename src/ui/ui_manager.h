/**
 * UI Manager
 * 
 * Manages LVGL display and user interface
 */

#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize UI system
 * - Initializes display hardware
 * - Sets up LVGL
 * 
 * @return ESP_OK on success
 */
esp_err_t ui_manager_init(void);

/**
 * Update accelerometer display values
 * 
 * @param accel_x X-axis acceleration
 * @param accel_y Y-axis acceleration
 * @param accel_z Z-axis acceleration
 */
void ui_update_accel(int16_t accel_x, int16_t accel_y, int16_t accel_z);

/**
 * Start UI task
 * Creates FreeRTOS task for display updates
 */
void ui_manager_start_task(void);

#ifdef __cplusplus
}
#endif

#endif // UI_MANAGER_H
