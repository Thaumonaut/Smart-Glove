/**
 * Microphone Manager Implementation
 * INMP441 I2S MEMS Microphone on I2S Port 1
 */

#include "mic_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "MicManager";

// Microphone state
static bool mic_enabled = false;
static mic_data_callback_t data_callback = NULL;
static TaskHandle_t mic_task_handle = NULL;

/**
 * Microphone capture task
 * Reads audio from INMP441 and passes to callback
 * Sample rate switches dynamically: 44.1kHz normal, 16kHz during HFP calls
 */
static void mic_task(void *pvParameters) {
    ESP_LOGI(TAG, "Microphone task started (I2S Port 1, separate clocks)");
    
    // Buffer for mic samples (32-bit from INMP441)
    const size_t buffer_size = 1024;
    uint8_t *mic_buffer = (uint8_t *)malloc(buffer_size);
    
    if (!mic_buffer) {
        ESP_LOGE(TAG, "Failed to allocate mic buffer");
        vTaskDelete(NULL);
        return;
    }
    
    size_t bytes_read = 0;
    
    static uint32_t read_count = 0;
    
    while (1) {
        if (mic_enabled && data_callback) {
            // Read from microphone on I2S Port 1 (separate clocks from speaker)
            esp_err_t ret = i2s_read(I2S_PORT_MIC, mic_buffer, buffer_size, &bytes_read, pdMS_TO_TICKS(100));
            
            read_count++;
            if (read_count % 100 == 0) {  // Log every 100 reads (~10 seconds at 100ms timeout)
                ESP_LOGI(TAG, "Mic task running: read #%lu, enabled=%d, callback=%p, ret=%d, bytes=%d", 
                         read_count, mic_enabled, data_callback, ret, bytes_read);
            }
            
            if (ret == ESP_OK && bytes_read > 0) {
                // Call registered callback with mic data
                data_callback(mic_buffer, bytes_read);
            } else if (ret != ESP_OK) {
                ESP_LOGW(TAG, "I2S read error: %d", ret);
            }
        } else {
            // Microphone disabled or no callback, sleep
            if (read_count % 50 == 0) {
                ESP_LOGI(TAG, "Mic task sleeping: enabled=%d, callback=%p", mic_enabled, data_callback);
            }
            read_count++;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    free(mic_buffer);
    vTaskDelete(NULL);
}

esp_err_t mic_manager_init(void) {
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "*** INITIALIZING MICROPHONE (I2S Port 1) ***");
    ESP_LOGI(TAG, "========================================");
    
    // I2S Port 1 in MASTER mode with SEPARATE clocks from speaker
    // This configuration dynamically switches sample rates (44.1kHz → 16kHz during calls)
    i2s_config_t i2s_mic_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // MASTER mode - generates own clocks
        .sample_rate = 44100,  // Start at 44.1kHz (will switch to 16kHz during calls)
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // INMP441 outputs 24-bit in 32-bit frame
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,   // Mono (L/R pin to GND)
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,    // 4 DMA buffers - low latency for voice
        .dma_buf_len = 256,    // 256 samples/buffer = ~6ms latency per buffer
        .use_apll = false,     // Use standard PLL (will match speaker's clock source)
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        .bits_per_chan = I2S_BITS_PER_CHAN_32BIT
    };

    // Pin configuration - BCK/WS are INPUTS (slave mode), shared with speaker
    i2s_pin_config_t mic_pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,   // No MCLK
        .bck_io_num = I2S_MIC_BCK_IO,      // GPIO27 (SHARED, input in slave mode)
        .ws_io_num = I2S_MIC_WS_IO,        // GPIO26 (SHARED, input in slave mode)
        .data_out_num = I2S_PIN_NO_CHANGE, // Not used for RX
        .data_in_num = I2S_MIC_DI_IO       // GPIO32 (separate data line)
    };

    esp_err_t ret = i2s_driver_install(I2S_PORT_MIC, &i2s_mic_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S mic driver: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "*** I2S mic driver installed (MASTER mode) ***");
    
    ret = i2s_set_pin(I2S_PORT_MIC, &mic_pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S mic pins: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "*** Separate I2S clocks: BCK=GPIO%d, WS=GPIO%d, DIN=GPIO%d ***", 
             I2S_MIC_BCK_IO, I2S_MIC_WS_IO, I2S_MIC_DI_IO);
    
    // CRITICAL FIX: Increase drive strength on clock pins to fix signal integrity
    gpio_set_drive_capability((gpio_num_t)I2S_MIC_BCK_IO, GPIO_DRIVE_CAP_3);  // BCK: Max drive (40mA)
    gpio_set_drive_capability((gpio_num_t)I2S_MIC_WS_IO, GPIO_DRIVE_CAP_3);   // WS: Max drive (40mA)
    ESP_LOGI(TAG, "*** Mic clock pins set to maximum drive strength (40mA) ***");
    
    ESP_LOGI(TAG, "*** Dynamic sample rate: 44.1kHz normal, 16kHz during HFP calls ***");
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "*** MICROPHONE INIT COMPLETE ***");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}

void mic_manager_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing microphone...");
    
    // Stop and delete task if running
    if (mic_task_handle != NULL) {
        vTaskDelete(mic_task_handle);
        mic_task_handle = NULL;
    }
    
    // Uninstall I2S driver to free I2S Port 1
    i2s_driver_uninstall(I2S_PORT_MIC);
    
    mic_enabled = false;
    ESP_LOGI(TAG, "Microphone deinitialized");
}

void mic_manager_start_task(void) {
    if (mic_task_handle == NULL) {
        xTaskCreate(mic_task, "mic_task", 4096, NULL, 5, &mic_task_handle);
        ESP_LOGI(TAG, "Microphone task created");
    }
}

void mic_manager_register_callback(mic_data_callback_t callback) {
    data_callback = callback;
    ESP_LOGI(TAG, "Microphone callback registered");
}

void mic_manager_enable(bool enable) {
    mic_enabled = enable;
    ESP_LOGI(TAG, "Microphone %s", enable ? "ENABLED" : "DISABLED");
}

bool mic_manager_is_enabled(void) {
    return mic_enabled;
}
