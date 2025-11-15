/**
 * Gesture Detection
 * 
 * Analyzes sensor data to recognize gestures:
 * - "Call me" gesture
 * - Emergency alert gestures
 * - Thumb swipe navigation
 */

#ifndef GESTURE_DETECTOR_H
#define GESTURE_DETECTOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Gesture types
 */
typedef enum {
    GESTURE_NONE,
    GESTURE_CALL_ME,           // Phone gesture to ear
    GESTURE_EMERGENCY_SHAKE,   // Rapid shaking
    GESTURE_SWIPE_UP,          // Thumb swipe up
    GESTURE_SWIPE_DOWN,        // Thumb swipe down
    GESTURE_TAP                // Quick tap
} gesture_type_t;

/**
 * Gesture callback function type
 */
typedef void (*gesture_callback_t)(gesture_type_t gesture);

/**
 * Initialize gesture detection
 * 
 * @param callback Function to call when gesture detected
 * @return ESP_OK on success
 */
esp_err_t gesture_detector_init(gesture_callback_t callback);

/**
 * Start gesture detection task
 */
void gesture_detector_start_task(void);

#ifdef __cplusplus
}
#endif

#endif // GESTURE_DETECTOR_H
