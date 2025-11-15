/**
 * Bluetooth A2DP Sink Test
 *
 * Test Bluetooth Classic audio reception (phone → glove)
 * This receives audio from a phone and plays it through the speaker
 *
 * To use this test:
 * 1. Comment out src.cpp in src/CMakeLists.txt
 * 2. Uncomment this file in test/CMakeLists.txt
 * 3. Build and upload
 * 4. Pair phone with "SmartGlove_Test"
 * 5. Play audio on phone
 *
 * Hardware connections:
 * - MAX98357A speaker amp on I2S port 0 (GPIO 25, 26, 27)
 * - 3W speaker connected to amp
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"
#include "driver/gpio.h"

#include "../src/config.h"

// ============================================================================
// LOGGING
// ============================================================================
static const char *TAG = "BT_A2DP_SINK";

// ============================================================================
// BLUETOOTH STATE
// ============================================================================
static bool bt_connected = false;
static bool audio_playing = false;

// ============================================================================
// I2S INITIALIZATION
// ============================================================================
static void init_i2s_speaker(void) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,  // CD quality for music testing
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = true,  // Use APLL for better audio quality
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_SPKR_DO_IO,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_SPEAKER, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_SPEAKER, &pin_config));
    ESP_ERROR_CHECK(i2s_set_clk(I2S_PORT_SPEAKER, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO));

    ESP_LOGI(TAG, "I2S speaker initialized (44.1kHz stereo)");
}

// ============================================================================
// A2DP CALLBACKS
// ============================================================================

/**
 * A2DP data callback - called when audio data is received from phone
 */
static void bt_a2dp_data_cb(const uint8_t *data, uint32_t len) {
    if (data == NULL || len == 0) {
        return;
    }

    // Write audio data to I2S speaker
    size_t bytes_written = 0;
    i2s_write(I2S_PORT_SPEAKER, data, len, &bytes_written, portMAX_DELAY);

    if (!audio_playing) {
        audio_playing = true;
        ESP_LOGI(TAG, "Audio playback started");
    }
}

/**
 * A2DP callback - handles connection state changes
 */
static void bt_a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "A2DP connected to device");
                bt_connected = true;

                // Vibrate to confirm connection
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "A2DP disconnected");
                bt_connected = false;
                audio_playing = false;
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
                ESP_LOGI(TAG, "Audio streaming started");
            } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED) {
                ESP_LOGI(TAG, "Audio streaming stopped");
                audio_playing = false;
            }
            break;

        case ESP_A2D_AUDIO_CFG_EVT:
            ESP_LOGI(TAG, "Audio config: sample_rate=%d, channels=%d",
                     param->audio_cfg.mcc.cie.sbc[0],
                     param->audio_cfg.mcc.cie.sbc[1]);
            break;

        default:
            ESP_LOGW(TAG, "Unhandled A2DP event: %d", event);
            break;
    }
}

// ============================================================================
// GAP CALLBACKS
// ============================================================================

/**
 * GAP callback - handles pairing and discovery
 */
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success: %s",
                         param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d",
                         param->auth_cmpl.stat);
            }
            break;

        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGI(TAG, "GAP mode change: %d", param->mode_chg.mode);
            break;

        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

// ============================================================================
// BLUETOOTH INITIALIZATION
// ============================================================================

static void init_bluetooth(void) {
    ESP_LOGI(TAG, "Initializing Bluetooth...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Release memory used by BLE (we only need Classic)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // Initialize Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Register callbacks
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_a2dp_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_a2dp_data_cb));

    // Initialize A2DP sink
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    // Set device name
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name("SmartGlove_Test"));

    // Set discoverable and connectable mode
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                              ESP_BT_GENERAL_DISCOVERABLE));

    ESP_LOGI(TAG, "Bluetooth initialized - device is discoverable as 'SmartGlove_Test'");
    ESP_LOGI(TAG, "Pair your phone and play audio to test A2DP");
}

// ============================================================================
// MAIN
// ============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Bluetooth A2DP Sink Test");
    ESP_LOGI(TAG, "Smart Glove - Audio Reception Test");
    ESP_LOGI(TAG, "===========================================");

    // Initialize vibration motor for feedback
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << VIBRATION_MOTOR_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);

    // Initialize I2S speaker
    init_i2s_speaker();

    // Initialize Bluetooth
    init_bluetooth();

    ESP_LOGI(TAG, "Test ready - waiting for phone connection...");

    // Status monitoring loop
    uint32_t last_status_log = 0;
    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_status_log > 10000) {  // Every 10 seconds
            ESP_LOGI(TAG, "Status: BT=%s Audio=%s",
                     bt_connected ? "Connected" : "Disconnected",
                     audio_playing ? "Playing" : "Idle");
            last_status_log = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
