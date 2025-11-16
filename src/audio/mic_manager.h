/**
 * Microphone Manager - INMP441 I2S Microphone
 * Handles microphone input for A2DP Source and voice commands
 */

#ifndef MIC_MANAGER_H
#define MIC_MANAGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Microphone callback type - called when audio data is available
typedef void (*mic_data_callback_t)(const uint8_t *data, size_t len);

/**
 * Initialize microphone on I2S Port 1
 * Uses separate BCK/WS pins from speaker (GPIO16/17)
 */
// Initialize microphone I2S driver
esp_err_t mic_manager_init(void);

// Deinitialize microphone I2S driver (free resources)
void mic_manager_deinit(void);

// Start/stop the microphone task

/**
 * Start microphone capture task
 * Continuously reads from mic and calls registered callback
 */
void mic_manager_start_task(void);

/**
 * Register callback for microphone data
 * Callback is called from FreeRTOS task context
 */
void mic_manager_register_callback(mic_data_callback_t callback);

/**
 * Enable/disable microphone capture
 */
void mic_manager_enable(bool enable);

/**
 * Get microphone status
 */
bool mic_manager_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // MIC_MANAGER_H
