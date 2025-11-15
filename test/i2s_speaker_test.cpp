/**
 * I2S Speaker Test
 *
 * Tests MAX98357A speaker amplifier with a simple tone
 * Plays a 440Hz sine wave (musical note A4) for 3 seconds
 *
 * Hardware connections:
 * - MAX98357A VIN → 5V
 * - MAX98357A GND → GND
 * - MAX98357A DIN → GPIO 25
 * - MAX98357A BCLK → GPIO 27
 * - MAX98357A LRC → GPIO 26
 * - 3W Speaker → MAX98357A output
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"

#include "../src/config.h"

static const char *TAG = "I2S_SPEAKER_TEST";

#define TEST_SAMPLE_RATE    16000
#define TEST_FREQUENCY      440     // A4 note
#define TEST_DURATION_MS    3000
#define TEST_AMPLITUDE      8000    // 16-bit PCM amplitude

/**
 * Generate sine wave samples
 */
static void generate_sine_wave(int16_t *samples, size_t num_samples, float frequency, float sample_rate) {
    for (size_t i = 0; i < num_samples; i++) {
        float t = (float)i / sample_rate;
        float value = sinf(2.0f * M_PI * frequency * t);
        samples[i] = (int16_t)(value * TEST_AMPLITUDE);
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "I2S Speaker Test");
    ESP_LOGI(TAG, "Playing 440Hz tone for %d seconds", TEST_DURATION_MS / 1000);
    ESP_LOGI(TAG, "===========================================");

    // Configure I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = TEST_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_SPKR_DO_IO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    // Install and configure I2S driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_SPEAKER, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_SPEAKER, &pin_config));

    ESP_LOGI(TAG, "I2S initialized - generating tone...");

    // Generate one second of audio samples
    size_t num_samples = TEST_SAMPLE_RATE;
    int16_t *samples = (int16_t*)malloc(num_samples * sizeof(int16_t));
    if (samples == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sample buffer");
        return;
    }

    generate_sine_wave(samples, num_samples, TEST_FREQUENCY, TEST_SAMPLE_RATE);
    ESP_LOGI(TAG, "Generated %d samples of %dHz sine wave", num_samples, TEST_FREQUENCY);

    // Play the tone for TEST_DURATION_MS
    ESP_LOGI(TAG, "Playing tone...");
    size_t bytes_written;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) < TEST_DURATION_MS) {
        i2s_write(I2S_PORT_SPEAKER, samples, num_samples * sizeof(int16_t),
                  &bytes_written, portMAX_DELAY);
    }

    ESP_LOGI(TAG, "Tone playback complete");
    ESP_LOGI(TAG, "If you heard a tone, the speaker is working!");

    // Cleanup
    free(samples);
    i2s_driver_uninstall(I2S_PORT_SPEAKER);

    ESP_LOGI(TAG, "Test complete");
}
