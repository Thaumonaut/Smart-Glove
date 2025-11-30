/**
 * Audio Manager Implementation
 */

#include "audio_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "AudioManager";

/**
 * Audio processing task
 */
static void audio_task(void *pvParameters) {
    ESP_LOGI(TAG, "Audio task started");

    // Audio is primarily handled by Bluetooth A2DP callbacks
    // This task can be used for:
    // - Voice command processing
    // - Audio effects/equalization
    // - Recording to SD card
    
    while (1) {
        // TODO: Add voice command processing or audio effects
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t audio_manager_init(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "*** INITIALIZING I2S AUDIO ***");
    ESP_LOGI(TAG, "========================================");
    
    // Configure MAX98357A GAIN pin (GPIO output)
    // HIGH = 15dB gain (loudest), LOW = 9dB gain (quietest)
    gpio_config_t gain_cfg = {
        .pin_bit_mask = (1ULL << I2S_GAIN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gain_cfg);
    // Set GAIN HIGH for maximum output (15dB)
    gpio_set_level((gpio_num_t)I2S_GAIN_PIN, 1);  // HIGH = 15dB (loudest)
    ESP_LOGI(TAG, "*** MAX98357A GAIN pin: GPIO5 = HIGH (15dB gain - MAXIMUM) ***");
    
    // I2S configuration for speaker (port 0)
    // Separate clocks from microphone (no TDM sharing)
    // DMA handles data transfer without CPU intervention
    i2s_config_t i2s_speaker_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),  // Master generates clocks
        .sample_rate = 44100,  // 44.1kHz for music/A2DP
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,    // 8 DMA buffers for smooth playback
        .dma_buf_len = 512,    // 512 samples/buffer (increased) - reduces interrupt frequency & noise
        .use_apll = true,      // Use APLL for cleaner clock (reduces jitter/noise)
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        .bits_per_chan = I2S_BITS_PER_CHAN_16BIT
    };

    i2s_pin_config_t speaker_pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_IO,        // Master clock output (shared)
        .ws_io_num = I2S_WS_IO,          // Master WS output (shared)
        .data_out_num = I2S_SPKR_DO_IO,  // Speaker data only
        .data_in_num = I2S_PIN_NO_CHANGE // No RX on this port
    };

    esp_err_t ret = i2s_driver_install(I2S_PORT_SPEAKER, &i2s_speaker_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S driver: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "*** I2S driver installed successfully ***");
    
    ret = i2s_set_pin(I2S_PORT_SPEAKER, &speaker_pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S pins: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "*** I2S pins: BCK=GPIO%d, WS=GPIO%d, DOUT=GPIO%d ***", I2S_BCK_IO, I2S_WS_IO, I2S_SPKR_DO_IO);
    
    // NOISE REDUCTION: Use moderate drive strength on clock pins
    // Lower drive = less EMI, but signal integrity may suffer with long wires
    // If audio is choppy/distorted, increase to GPIO_DRIVE_CAP_2 or _3
    gpio_set_drive_capability((gpio_num_t)I2S_BCK_IO, GPIO_DRIVE_CAP_1);   // BCK: 10mA (balanced)
    gpio_set_drive_capability((gpio_num_t)I2S_WS_IO, GPIO_DRIVE_CAP_1);    // WS: 10mA (balanced)
    gpio_set_drive_capability((gpio_num_t)I2S_SPKR_DO_IO, GPIO_DRIVE_CAP_2); // DOUT: 20mA (data needs more)
    ESP_LOGI(TAG, "*** Clock pins set to MODERATE drive strength (10mA) to reduce EMI ***");
    ESP_LOGI(TAG, "*** Hardware fix: Add 100-220 ohm series resistors on BCK/WS if noise persists ***");
    
    // Set I2S clock for better audio quality
    i2s_set_clk(I2S_PORT_SPEAKER, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
    ESP_LOGI(TAG, "*** I2S clock: 44.1kHz, 16-bit stereo ***");
    
    // Clear any existing data in DMA buffers
    i2s_zero_dma_buffer(I2S_PORT_SPEAKER);
    
    // CRITICAL: Write silence immediately to start LRC toggling
    // MAX98357A enters shutdown if LRC is LOW for >100ms
    // This keeps the amp active and ready for audio
    int16_t silence[128] = {0};  // 128 samples of silence
    size_t bytes_written;
    i2s_write(I2S_PORT_SPEAKER, silence, sizeof(silence), &bytes_written, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "*** Started I2S clock - LRC should be toggling NOW ***");
    ESP_LOGI(TAG, "*** MAX98357A should be ACTIVE (not in shutdown) ***");

    // NOTE: Microphone initialization skipped
    // ESP32 I2S cannot share BCK/WS pins between two ports
    // For hands-free calling, use Bluetooth HFP (Hands-Free Profile) instead
    // HFP uses built-in Bluetooth audio path, not I2S
    // The INMP441 mic can be used for local recording/voice commands via I2S0
    // when speaker is not active, or use a dedicated microphone solution
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "*** Note: For Bluetooth calls, use HFP profile ***");
    ESP_LOGI(TAG, "*** INMP441 mic available for local voice commands ***");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "*** I2S INITIALIZATION COMPLETE ***");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    return ESP_OK;
}

void audio_manager_start_task(void) {
    xTaskCreate(audio_task, "audio_task", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "Audio task created");
}
