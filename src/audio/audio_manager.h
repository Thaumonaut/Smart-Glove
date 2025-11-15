/**
 * Audio Manager
 * 
 * Manages I2S audio streaming for:
 * - Bluetooth A2DP sink (music/calls)
 * - Microphone input
 * - Voice commands (future)
 */

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize I2S audio subsystem
 * Configures speaker and microphone I2S interfaces
 * 
 * @return ESP_OK on success
 */
esp_err_t audio_manager_init(void);

/**
 * Start audio processing task
 * Currently handled by Bluetooth A2DP callbacks
 * Can be extended for voice commands or audio effects
 */
void audio_manager_start_task(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_MANAGER_H
