/**
 * Analog Sensors Implementation
 */

#include "analog_sensors.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "AnalogSensors";

esp_err_t analog_sensors_init(void) {
    // Configure ADC1 (works with WiFi/BT, unlike ADC2)
    adc1_config_width(ADC_WIDTH);

    // Configure each ADC channel
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN);  // GPIO 36 - Index
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN);  // GPIO 39 - Middle
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN);  // GPIO 34 - Pinky
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN);  // GPIO 35 - Pressure
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN);  // GPIO 33 - Pressure 2

    ESP_LOGI(TAG, "ADC initialized for flex sensors and pressure pads");
    return ESP_OK;
}

void read_flex_sensors(flex_sensors_t *sensors) {
    sensors->index = adc1_get_raw(ADC1_CHANNEL_0);
    sensors->middle = adc1_get_raw(ADC1_CHANNEL_3);
    sensors->pinky = adc1_get_raw(ADC1_CHANNEL_6);
}

void read_pressure_sensors(pressure_sensors_t *sensors) {
    sensors->palm = adc1_get_raw(ADC1_CHANNEL_7);
    sensors->thumb = adc1_get_raw(ADC1_CHANNEL_5);
}
