/**
 * UI Manager - Display and User Interface
 */

#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "esp_err.h"
#include <stdint.h>

// Initialize UI system
esp_err_t ui_manager_init(void);

// Start UI task
void ui_manager_start_task(void);

// Update accelerometer display values
void ui_update_accel(int16_t accel_x, int16_t accel_y, int16_t accel_z);

// Update all UI elements (call from main loop)
void ui_update_all(void);

#endif // UI_MANAGER_H
