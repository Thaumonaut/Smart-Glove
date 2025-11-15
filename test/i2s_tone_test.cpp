/**
 * Simple I2S Tone Test for MAX98357A
 * Generates a 440Hz sine wave directly to speaker
 * Use this to verify I2S hardware is working
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"

#define I2S_BCK_IO      27
#define I2S_WS_IO       26
#define I2S_DOUT_IO     25
#define I2S_GAIN_PIN    5

#define SAMPLE_RATE     44100
#define TONE_FREQ       440  // A4 note
#define AMPLITUDE       8000 // 25% of max volume

static const char *TAG = "I2S_TONE_TEST";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2S Tone Test - 440Hz sine wave");
    ESP_LOGI(TAG, "========================================");
    
    // Configure GAIN pin LOW (9dB)
    gpio_config_t gain_cfg = {
        .pin_bit_mask = (1ULL << I2S_GAIN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gain_cfg);
    gpio_set_level((gpio_num_t)I2S_GAIN_PIN, 0);  // LOW = 9dB
    ESP_LOGI(TAG, "GAIN pin set to LOW (9dB)");
    
    // Configure I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DOUT_IO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    
    ESP_LOGI(TAG, "Installing I2S driver...");
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_LOGI(TAG, "Setting I2S pins: BCK=%d, WS=%d, DOUT=%d", I2S_BCK_IO, I2S_WS_IO, I2S_DOUT_IO);
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config));
    ESP_LOGI(TAG, "I2S initialized successfully");
    
    // Generate sine wave samples
    const int samples_per_cycle = SAMPLE_RATE / TONE_FREQ;
    const int buffer_size = 256;
    int16_t sample_buffer[buffer_size * 2]; // Stereo (L+R)
    
    ESP_LOGI(TAG, "Generating %dHz tone at %d samples/sec", TONE_FREQ, SAMPLE_RATE);
    ESP_LOGI(TAG, "Samples per cycle: %d", samples_per_cycle);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "YOU SHOULD HEAR A TONE NOW!");
    ESP_LOGI(TAG, "If not, check your wiring:");
    ESP_LOGI(TAG, "  GPIO27 -> BCLK");
    ESP_LOGI(TAG, "  GPIO26 -> LRC/WS");
    ESP_LOGI(TAG, "  GPIO25 -> DIN");
    ESP_LOGI(TAG, "  GPIO5  -> SD/GAIN (or disconnect)");
    ESP_LOGI(TAG, "  3.3V   -> VIN");
    ESP_LOGI(TAG, "  GND    -> GND");
    ESP_LOGI(TAG, "========================================");
    
    int sample_count = 0;
    size_t bytes_written;
    
    while (1) {
        // Fill buffer with sine wave
        for (int i = 0; i < buffer_size; i++) {
            float angle = 2.0 * M_PI * sample_count / samples_per_cycle;
            int16_t sample = (int16_t)(AMPLITUDE * sin(angle));
            
            // Stereo: same sample for both channels
            sample_buffer[i * 2] = sample;     // Left
            sample_buffer[i * 2 + 1] = sample; // Right
            
            sample_count++;
            if (sample_count >= samples_per_cycle) {
                sample_count = 0;
            }
        }
        
        // Write to I2S
        esp_err_t ret = i2s_write(I2S_NUM_0, sample_buffer, sizeof(sample_buffer), &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "i2s_write failed: %d", ret);
        }
        
        // Log every second
        static uint32_t last_log = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_log >= 1000) {
            ESP_LOGI(TAG, "Playing tone... bytes_written: %d", bytes_written);
            last_log = now;
        }
    }
}
