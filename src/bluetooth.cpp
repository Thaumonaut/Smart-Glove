/**
 * Bluetooth A2DP Audio Sink and HFP (Hands-Free Profile) Module
 * Handles Bluetooth Classic audio reception and phone calls
 */

#include "bluetooth.h"
#include "config.h"
#include "display.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_hf_client_api.h"  // HFP client API
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Bluetooth";

// Bluetooth state
static bool bt_connected = false;
static bool audio_playing = false;

// HFP state
static bool hfp_connected = false;
static bool call_active = false;
static esp_hf_call_status_t call_status = ESP_HF_CALL_STATUS_NO_CALLS;

// ============================================================================
// BLUETOOTH CALLBACKS
// ============================================================================

/**
 * A2DP data callback - receives audio from phone
 */
static void bt_a2dp_data_cb(const uint8_t *data, uint32_t len) {
    if (data == NULL || len == 0) {
        return;
    }

    // Direct passthrough - 100% volume, let phone control volume
    size_t bytes_written = 0;
    esp_err_t ret = i2s_write(I2S_PORT_SPEAKER, data, len, &bytes_written, pdMS_TO_TICKS(100));
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "*** I2S WRITE FAILED: %d ***", ret);
    } else if (bytes_written < len) {
        // Partial write = potential underrun
        static uint32_t underrun_count = 0;
        underrun_count++;
        if (underrun_count % 10 == 0) {
            ESP_LOGW(TAG, "*** UNDERRUN: Only wrote %d/%d bytes (count: %d) ***", bytes_written, len, underrun_count);
        }
    } else {
        // Diagnostic: Log write success periodically
        static uint32_t packet_count = 0;
        packet_count++;
        if (packet_count % 200 == 0) {
            ESP_LOGI(TAG, "*** Audio stable: packet #%d ***", packet_count);
        }
    }
    
    // Update playing status
    if (!audio_playing) {
        audio_playing = true;
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "*** AUDIO PLAYBACK STARTED ***");
        ESP_LOGI(TAG, "*** Volume: 100%% (phone controls volume) ***");
        ESP_LOGI(TAG, "*** GAIN: 15dB (maximum output) ***");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "");
        display_show_message("Playing audio");
    }
}

/**
 * A2DP callback - handles connection state changes
 */
static void bt_a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "A2DP connected");
                bt_connected = true;
                display_set_bluetooth_status(true);

                // Vibrate to confirm connection
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "A2DP disconnected");
                bt_connected = false;
                audio_playing = false;
                display_set_bluetooth_status(false);
                display_show_message("");
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
                ESP_LOGI(TAG, "Audio streaming started");
            } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED) {
                ESP_LOGI(TAG, "Audio streaming stopped");
                audio_playing = false;
                display_show_message("");
            }
            break;
            
        case ESP_A2D_AUDIO_CFG_EVT:
            // Audio configuration received - log codec details
            {
                ESP_LOGI(TAG, "=== Audio Codec Configuration ===");
                uint8_t codec_type = param->audio_cfg.mcc.type;
                
                if (codec_type == ESP_A2D_MCT_SBC) {
                    ESP_LOGI(TAG, "Codec: SBC (SubBand Codec)");
                    // SBC configuration is in param->audio_cfg.mcc.cie.sbc
                    // Byte 0: sampling frequency and channel mode
                    // Byte 1: block length, subbands, allocation method
                    // Byte 2-3: min/max bitpool
                    uint8_t samp_freq = (param->audio_cfg.mcc.cie.sbc[0] >> 4) & 0x0F;
                    uint8_t ch_mode = param->audio_cfg.mcc.cie.sbc[0] & 0x0F;
                    uint8_t min_bitpool = param->audio_cfg.mcc.cie.sbc[2];
                    uint8_t max_bitpool = param->audio_cfg.mcc.cie.sbc[3];
                    
                    const char* freq_str[] = {"16kHz", "32kHz", "44.1kHz", "48kHz"};
                    const char* mode_str[] = {"Mono", "Dual", "Stereo", "Joint Stereo"};
                    
                    ESP_LOGI(TAG, "Sample rate: %s", freq_str[samp_freq]);
                    ESP_LOGI(TAG, "Channel mode: %s", mode_str[ch_mode]);
                    ESP_LOGI(TAG, "Bitpool: %d-%d (higher = better quality)", min_bitpool, max_bitpool);
                } else if (codec_type == ESP_A2D_MCT_M24) {
                    ESP_LOGI(TAG, "Codec: AAC (Advanced Audio Coding) - EXCELLENT!");
                } else {
                    ESP_LOGI(TAG, "Codec type: 0x%02x", codec_type);
                }
                ESP_LOGI(TAG, "================================");
            }
            break;

        default:
            break;
    }
}

/**
 * GAP callback - handles pairing and authentication
 */
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
                ESP_LOGI(TAG, "Device paired successfully!");
            } else {
                ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
            }
            break;
            
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT - Please confirm pairing: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
            
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %d", param->key_notif.passkey);
            break;
            
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT - Please enter passkey");
            break;
            
        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
            break;
            
        default:
            ESP_LOGI(TAG, "GAP event: %d", event);
            break;
    }
}

/**
 * HFP (Hands-Free Profile) callback
 */
static void bt_hfp_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "HFP connection state: %d", param->conn_stat.state);
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED) {
                hfp_connected = true;
                ESP_LOGI(TAG, "*** HFP CONNECTED - Hands-free calling ready ***");
                display_show_message("HFP Ready");
            } else if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
                hfp_connected = false;
                call_active = false;
                ESP_LOGI(TAG, "HFP disconnected");
            }
            break;
            
        case ESP_HF_CLIENT_AUDIO_STATE_EVT:
            ESP_LOGI(TAG, "HFP audio state: %d", param->audio_stat.state);
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
                ESP_LOGI(TAG, "*** HFP AUDIO ACTIVE - Call in progress ***");
                call_active = true;
            } else {
                ESP_LOGI(TAG, "HFP audio disconnected");
                call_active = false;
            }
            break;
            
        case ESP_HF_CLIENT_BVRA_EVT:
            ESP_LOGI(TAG, "Voice recognition state: %d", param->bvra.value);
            break;
            
        case ESP_HF_CLIENT_CIND_CALL_EVT:
            ESP_LOGI(TAG, "Call indicator: %d", param->call.status);
            if (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS) {
                ESP_LOGI(TAG, "*** CALL IN PROGRESS ***");
                call_active = true;
                display_show_message("Call Active");
            } else if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS) {
                ESP_LOGI(TAG, "No active calls");
                call_active = false;
            }
            break;
            
        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
            ESP_LOGI(TAG, "Call setup: %d", param->call_setup.status);
            if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING) {
                ESP_LOGI(TAG, "*** INCOMING CALL ***");
                display_show_message("Incoming Call");
            } else if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING || 
                       param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING) {
                ESP_LOGI(TAG, "Outgoing call...");
                display_show_message("Calling...");
            }
            break;
            
        case ESP_HF_CLIENT_RING_IND_EVT:
            ESP_LOGI(TAG, "*** PHONE RINGING ***");
            // Optional: Add vibration/LED notification here
            break;
            
        case ESP_HF_CLIENT_CLIP_EVT:
            ESP_LOGI(TAG, "Caller ID: %s", param->clip.number);
            break;
            
        case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
            ESP_LOGI(TAG, "Volume: type=%d, volume=%d", param->volume_control.type, param->volume_control.volume);
            break;
            
        default:
            ESP_LOGI(TAG, "HFP event: %d", event);
            break;
    }
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void bluetooth_init(void) {
    ESP_LOGI(TAG, "=== BLUETOOTH INITIALIZATION STARTING ===");
    ESP_LOGI(TAG, "Step 1: Releasing BLE memory...");

    // Release BLE memory (we only need Classic)
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller release BLE memory failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "=== BLUETOOTH INIT FAILED AT STEP 1 ===");
        return;
    }
    ESP_LOGI(TAG, "Step 1: BLE memory released successfully");

    // Initialize Bluetooth controller
    ESP_LOGI(TAG, "Step 2: Initializing BT controller...");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "=== BLUETOOTH INIT FAILED AT STEP 2 ===");
        return;
    }
    ESP_LOGI(TAG, "Step 2: BT controller initialized successfully");

    ESP_LOGI(TAG, "Step 3: Enabling BT controller (BTDM mode - dual mode)...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);  // Try dual mode instead
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Trying Classic BT only mode...");
        // Deinit and try again with just classic
        esp_bt_controller_deinit();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret == ESP_OK) {
            ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Both BTDM and Classic BT modes failed");
            ESP_LOGE(TAG, "=== BLUETOOTH INIT FAILED AT STEP 3 ===");
            return;
        }
    }
    ESP_LOGI(TAG, "Step 3: BT controller enabled successfully");

    // Initialize Bluedroid stack
    ESP_LOGI(TAG, "Step 4: Initializing Bluedroid stack...");
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_LOGI(TAG, "Step 4: Bluedroid stack initialized");

    // Register callbacks
    ESP_LOGI(TAG, "Step 5: Registering callbacks...");
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_a2dp_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(bt_a2dp_data_cb));
    ESP_LOGI(TAG, "Step 5: Callbacks registered");

    // Initialize A2DP sink
    ESP_LOGI(TAG, "Step 6: Initializing A2DP sink...");
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_LOGI(TAG, "Step 6: A2DP sink initialized");
    
    // Initialize HFP (Hands-Free Profile)
    ESP_LOGI(TAG, "Step 7: Initializing HFP (Hands-Free Profile)...");
    ESP_ERROR_CHECK(esp_hf_client_register_callback(bt_hfp_cb));
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_LOGI(TAG, "Step 7: HFP initialized - hands-free calling enabled");
    
    // Note: AAC codec support
    // ESP32 Bluedroid stack supports AAC (ESP_A2D_MCT_M24) but codec negotiation
    // happens automatically between phone and ESP32. The phone will use AAC if:
    // 1. Phone supports AAC
    // 2. ESP32 has AAC decoder capability enabled in sdkconfig
    // 3. Both devices successfully negotiate AAC during connection
    // If negotiation fails, it falls back to SBC automatically.
    ESP_LOGI(TAG, "Codec support: SBC (always), AAC (if phone negotiates)");

    // Set device name
    ESP_LOGI(TAG, "Step 8: Setting device name to '%s'...", BT_DEVICE_NAME);
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(BT_DEVICE_NAME));
    ESP_LOGI(TAG, "Step 8: Device name set");

    // Set discoverable and connectable
    ESP_LOGI(TAG, "Step 9: Setting discoverable mode...");
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
    ESP_LOGI(TAG, "Step 9: Device is now discoverable");

    // Configure pairing/authentication
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Configuring pairing parameters...");
    
    // Set IO capability to NONE for "Just Works" pairing (no display/keyboard)
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));
    
    // Set authentication requirements - no MITM protection needed for Just Works
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    
    // Set default PIN code (used for legacy devices)
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
    
    ESP_LOGI(TAG, "Pairing mode: Just Works (no PIN required for modern devices)");
    ESP_LOGI(TAG, "Legacy PIN: 0000");

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== BLUETOOTH FULLY INITIALIZED ===");
    ESP_LOGI(TAG, "Device name: %s", BT_DEVICE_NAME);
    ESP_LOGI(TAG, "Status: Discoverable and ready to pair");
    ESP_LOGI(TAG, "Look for 'SmartGlove' in your phone's Bluetooth settings");
}

bool bluetooth_is_connected(void) {
    return bt_connected;
}

bool bluetooth_is_playing_audio(void) {
    return audio_playing;
}

// ============================================================================
// HFP (HANDS-FREE PROFILE) FUNCTIONS
// ============================================================================

bool bluetooth_is_call_active(void) {
    return call_active;
}

void bluetooth_answer_call(void) {
    if (hfp_connected) {
        ESP_LOGI(TAG, "*** ANSWERING CALL ***");
        esp_hf_client_answer_call();
        display_show_message("Answering...");
    } else {
        ESP_LOGW(TAG, "HFP not connected - cannot answer call");
    }
}

void bluetooth_reject_call(void) {
    if (hfp_connected) {
        ESP_LOGI(TAG, "*** REJECTING CALL ***");
        esp_hf_client_reject_call();
        display_show_message("Call Rejected");
    } else {
        ESP_LOGW(TAG, "HFP not connected - cannot reject call");
    }
}

void bluetooth_hangup_call(void) {
    if (hfp_connected) {
        ESP_LOGI(TAG, "*** HANGING UP CALL ***");
        // Use CHLD command to release all active calls
        esp_hf_client_send_chld_cmd(ESP_HF_CHLD_TYPE_REL_ACC, 0);
        display_show_message("Call Ended");
    } else {
        ESP_LOGW(TAG, "HFP not connected - cannot hangup");
    }
}
