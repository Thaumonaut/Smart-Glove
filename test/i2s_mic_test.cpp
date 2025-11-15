/**
 * I2S Microphone Test
 *
 * Tests INMP441 MEMS microphone by recording and displaying audio levels
 * Useful for verifying microphone is working before Bluetooth integration
 *
 * Hardware connections:
 * - INMP441 VDD → 3.3V
 * - INMP441 GND → GND
 * - INMP441 SD → GPIO 32
 * - INMP441 WS → GPIO 26
 * - INMP441 SCK → GPIO 27
 * - INMP441 L/R → GND (left channel)
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"

#include "../src/config.h"

static const char *TAG = "I2S_MIC_TEST";

#define SAMPLE_BUFFER_SIZE  1024

/**
 * Calculate RMS (Root Mean Square) of audio samples
 * This gives us the audio level/volume
 */
static float calculate_rms(int16_t *samples, size_t num_samples) {
    float sum = 0.0f;
    for (size_t i = 0; i < num_samples; i++) {
        float value = (float)samples[i];
        sum += value * value;
    }
    return sqrtf(sum / num_samples);
}

/**
 * Draw a simple VU meter in the console
 */
static void draw_vu_meter(float rms, float max_rms) {
    const int bar_width = 50;
    float normalized = rms / max_rms;
    if (normalized > 1.0f) normalized = 1.0f;

    int filled = (int)(normalized * bar_width);

    printf("\rLevel: [");
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) {
            printf("=");
        } else {
            printf(" ");
        }
    }
    printf("] %.0f ", rms);
    fflush(stdout);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "I2S Microphone Test");
    ESP_LOGI(TAG, "Speak into the microphone");
    ESP_LOGI(TAG, "===========================================");

    // Configure I2S for microphone input
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_DI_IO
    };

    // Install and configure I2S driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_MIC, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_MIC, &pin_config));

    ESP_LOGI(TAG, "I2S microphone initialized on GPIO %d", I2S_MIC_DI_IO);
    ESP_LOGI(TAG, "Sample rate: %d Hz", I2S_SAMPLE_RATE);
    ESP_LOGI(TAG, "Starting audio capture...\n");

    // Allocate buffer for audio samples
    int16_t *samples = (int16_t*)malloc(SAMPLE_BUFFER_SIZE * sizeof(int16_t));
    if (samples == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sample buffer");
        return;
    }

    // Continuously read and display audio levels
    const float MAX_RMS = 5000.0f;  // Expected maximum RMS value
    size_t bytes_read = 0;

    while (1) {
        // Read samples from microphone
        esp_err_t result = i2s_read(I2S_PORT_MIC, samples,
                                    SAMPLE_BUFFER_SIZE * sizeof(int16_t),
                                    &bytes_read, portMAX_DELAY);

        if (result == ESP_OK && bytes_read > 0) {
            size_t num_samples = bytes_read / sizeof(int16_t);

            // Calculate and display audio level
            float rms = calculate_rms(samples, num_samples);
            draw_vu_meter(rms, MAX_RMS);

            // Also log peak values occasionally for debugging
            static int log_counter = 0;
            if (log_counter++ % 50 == 0) {
                int16_t min_val = 32767, max_val = -32768;
                for (size_t i = 0; i < num_samples; i++) {
                    if (samples[i] < min_val) min_val = samples[i];
                    if (samples[i] > max_val) max_val = samples[i];
                }
                ESP_LOGI(TAG, "\nRMS: %.0f, Min: %d, Max: %d", rms, min_val, max_val);
            }
        } else {
            ESP_LOGE(TAG, "I2S read error: %d", result);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Cleanup (never reached in this test)
    free(samples);
    i2s_driver_uninstall(I2S_PORT_MIC);
}
