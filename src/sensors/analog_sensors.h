/**
 * Analog Sensors (Flex Sensors and Pressure Pads)
 * 
 * ADC-based sensor reading for:
 * - 3x Flex sensors (index, middle, pinky fingers)
 * - 2x Pressure pads (palm/thumb area)
 */

#ifndef ANALOG_SENSORS_H
#define ANALOG_SENSORS_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Flex sensor readings
 */
typedef struct {
    int index;   // Index finger flex (0-4095)
    int middle;  // Middle finger flex (0-4095)
    int pinky;   // Pinky finger flex (0-4095)
} flex_sensors_t;

/**
 * Pressure sensor readings
 */
typedef struct {
    int palm;    // Palm pressure pad (0-4095)
    int thumb;   // Thumb area pressure (0-4095)
} pressure_sensors_t;

/**
 * Initialize ADC for analog sensors
 * Configures ADC1 channels for flex and pressure sensors
 * 
 * @return ESP_OK on success
 */
esp_err_t analog_sensors_init(void);

/**
 * Read flex sensor values
 * 
 * @param sensors Pointer to store flex sensor readings
 */
void read_flex_sensors(flex_sensors_t *sensors);

/**
 * Read pressure sensor values
 * 
 * @param sensors Pointer to store pressure sensor readings
 */
void read_pressure_sensors(pressure_sensors_t *sensors);

#ifdef __cplusplus
}
#endif

#endif // ANALOG_SENSORS_H
