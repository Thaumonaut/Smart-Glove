/**
 * Bluetooth A2DP Audio Sink and HFP (Hands-Free Profile) Module
 * Handles Bluetooth Classic audio reception and phone calls
 */

#include "bluetooth.h"
#include "config.h"
#include "display.h"
#include "audio/mic_manager.h"
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

// ESP-ADF SBC decoder/encoder for mSBC
#include "decoder/esp_audio_dec.h"
#include "decoder/impl/esp_sbc_dec.h"
#include "encoder/esp_audio_enc.h"
#include "encoder/impl/esp_sbc_enc.h"
#include "esp_sbc_def.h"

static const char *TAG = "Bluetooth";

// Bluetooth state
static bool bt_connected = false;
static bool audio_playing = false;

// HFP state
static bool hfp_connected = false;
static bool call_active = false;
static esp_hf_call_status_t call_status = ESP_HF_CALL_STATUS_NO_CALLS;

// mSBC decoder/encoder state
static void *msbc_decoder = NULL;
static void *msbc_encoder = NULL;
static esp_hf_sync_conn_hdl_t hfp_sync_conn_hdl = 0;  // Store connection handle for uplink
static uint32_t hfp_rx_packet_count = 0;  // Count packets received from phone
static bool mic_tx_ready = false;  // Only send mic data after receiving packets from phone
static uint16_t hfp_preferred_frame_size = 60;  // Frame size from phone (set in AUDIO_STATE_EVT)
static volatile bool mic_tx_slot_available = false;  // Set to true each time we receive audio from phone

// Microphone data callback - called by mic_manager when audio is available
static void mic_data_ready_cb(const uint8_t *data, size_t len) {
    static uint32_t packet_count = 0;
    packet_count++;
    
    // DEBUG: Log every call for first few, then periodically
    if (packet_count <= 10 || packet_count % 100 == 0) {
        // Also log first few raw samples to debug microphone reading
        const int32_t *samples_32 = (const int32_t *)data;
        ESP_LOGI(TAG, "MIC CALLBACK #%lu: call=%d, hfp=%d, hdl=%d, ready=%d, rx=%lu, slot=%d, len=%d [raw32: %ld, %ld, %ld]",
                 packet_count, call_active, hfp_connected, hfp_sync_conn_hdl,
                 mic_tx_ready, hfp_rx_packet_count, mic_tx_slot_available, len,
                 samples_32[0], samples_32[1], samples_32[2]);
    }
    
    if (!call_active || !hfp_connected || hfp_sync_conn_hdl == 0) {
        if (packet_count % 100 == 0) {  // Log every 100 packets
            ESP_LOGD(TAG, "Mic data dropped: call_active=%d, hfp_connected=%d, hdl=%d", 
                     call_active, hfp_connected, hfp_sync_conn_hdl);
        }
        return;  // Only send mic data during active call
    }
    
    // CRITICAL: Wait until we've received packets from phone before sending
    // This ensures HFP audio buffers are properly initialized
    if (!mic_tx_ready || hfp_rx_packet_count < 50) {
        if (packet_count % 50 == 0) {
            ESP_LOGD(TAG, "Waiting for HFP RX to stabilize... (rx_count=%lu)", hfp_rx_packet_count);
        }
        return;
    }
    
    // CRITICAL RATE LIMITING: Only send when we have a "slot" from receiving phone audio
    // The phone sends packets every ~7.5ms, we use that as our pacing signal
    if (!mic_tx_slot_available) {
        // No slot available, drop this mic data
        return;
    }
    
    // During call: I2S reconfigured to 16kHz (no downsampling needed!)
    // INMP441 outputs 32-bit samples, HFP expects 16-bit mono
    // Just convert bit depth: 32-bit → 16-bit
    
    // CRITICAL: mSBC encoder requires exactly 240 bytes PCM input (120 samples @ 16-bit)
    // The "preferred frame size" (57 bytes) is the ENCODED OUTPUT size, not input!
    // Encoder expects: 120 samples × 2 bytes/sample = 240 bytes PCM input
    #define MSBC_PCM_INPUT_SIZE 240  // 120 samples × 2 bytes per sample = 240 bytes
    
    // Use accumulator for samples until we have enough for one mSBC frame
    static uint8_t sample_buffer[MSBC_PCM_INPUT_SIZE];
    static size_t sample_buf_pos = 0;
    
    // Process 32-bit mono samples from INMP441
    const int32_t *samples_32 = (const int32_t *)data;
    size_t num_samples = len / 4;  // 32-bit samples
    
    for (size_t i = 0; i < num_samples; i++) {
        // Convert 32-bit to 16-bit (take upper 16 bits where INMP441 audio data is)
        // INMP441 outputs 24-bit left-justified: bits[31:8] = audio, bits[7:0] = 0
        int32_t sample_32 = samples_32[i];
        
        // Remove DC offset (the repeating 256 pattern suggests ~256 offset)
        // This helps center the audio around zero
        int16_t sample_16_raw = (int16_t)(sample_32 >> 16);
        int16_t sample_16 = sample_16_raw - 256;  // Remove DC offset
        
        // Apply 4x gain to boost microphone volume (was 2x, now more aggressive)
        int32_t gained = (int32_t)sample_16 * 4;
        
        // Clamp to prevent overflow
        if (gained > 32767) gained = 32767;
        if (gained < -32768) gained = -32768;
        
        sample_16 = (int16_t)gained;
        
        // Store in temp buffer
        sample_buffer[sample_buf_pos++] = (uint8_t)(sample_16 & 0xFF);
        sample_buffer[sample_buf_pos++] = (uint8_t)(sample_16 >> 8);
        
        // When buffer is full, encode to mSBC and send to phone
        if (sample_buf_pos >= MSBC_PCM_INPUT_SIZE) {
            // CRITICAL: Consume the slot BEFORE sending
            // This prevents sending multiple packets in one callback
            mic_tx_slot_available = false;
            
            // Encode PCM to mSBC using ESP-ADF encoder
            // Input: 240 bytes PCM (120 samples @ 16-bit mono 16kHz)
            // Output: ~57-60 bytes mSBC encoded (varies slightly by encoder)
            uint8_t msbc_encoded[64];  // Slightly larger buffer for safety
            
            if (msbc_encoder) {
                // Prepare input PCM frame (MUST be exactly 240 bytes for mSBC)
                esp_audio_enc_in_frame_t in_frame = {
                    .buffer = sample_buffer,
                    .len = MSBC_PCM_INPUT_SIZE
                };
                
                // Prepare output encoded frame
                esp_audio_enc_out_frame_t out_frame = {
                    .buffer = msbc_encoded,
                    .len = sizeof(msbc_encoded)
                };
                
                esp_audio_err_t ret = esp_sbc_enc_process(msbc_encoder, &in_frame, &out_frame);
                
                if (ret == ESP_AUDIO_ERR_OK && out_frame.encoded_bytes > 0) {
                    // CRITICAL: Use esp_hf_client_audio_buff_alloc instead of static buffer
                    esp_hf_audio_buff_t *hfp_buf = esp_hf_client_audio_buff_alloc(out_frame.encoded_bytes);
                    if (hfp_buf == NULL) {
                        ESP_LOGW(TAG, "Failed to allocate HFP audio buffer");
                        sample_buf_pos = 0;
                        return;
                    }
                    
                    // Copy encoded mSBC data into the allocated buffer
                    memcpy(hfp_buf->data, msbc_encoded, out_frame.encoded_bytes);
                    hfp_buf->data_len = out_frame.encoded_bytes;
                    hfp_buf->buff_size = out_frame.encoded_bytes;
                    
                    // Double-check we still have valid connection before sending
                    if (hfp_sync_conn_hdl == 0 || !call_active) {
                        ESP_LOGW(TAG, "Lost HFP connection, stopping mic TX");
                        esp_hf_client_audio_buff_free(hfp_buf);
                        sample_buf_pos = 0;
                        return;
                    }
                    
                    // Send encoded mSBC to phone
                    esp_err_t send_ret = esp_hf_client_audio_data_send(hfp_sync_conn_hdl, hfp_buf);
                    if (send_ret != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to send mic data: %d", send_ret);
                        esp_hf_client_audio_buff_free(hfp_buf);  // Free on error
                    } else {
                        static uint32_t send_count = 0;
                        send_count++;
                        if (send_count % 20 == 0) {
                            ESP_LOGI(TAG, "MIC TX #%lu: %d bytes PCM → %d bytes mSBC sent", 
                                     send_count, MSBC_PCM_INPUT_SIZE, out_frame.encoded_bytes);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "mSBC encode failed: ret=%d, encoded_bytes=%d", ret, out_frame.encoded_bytes);
                }
            } else {
                ESP_LOGW(TAG, "mSBC encoder not initialized!");
            }
            
            sample_buf_pos = 0;  // Reset buffer
            
            // CRITICAL: Return immediately after sending ONE packet
            return;
        }
    }
}

// HFP audio callback for voice call audio routing (downlink: phone → speaker)
static void bt_hfp_audio_cb(esp_hf_sync_conn_hdl_t sync_conn_hdl, esp_hf_audio_buff_t *audio_buf, bool is_bad_frame) {
    static uint32_t callback_count = 0;
    callback_count++;
    
    // Always log first 10 callbacks to confirm it's working
    if (callback_count <= 10 || callback_count % 100 == 0) {
        ESP_LOGI(TAG, "HFP audio callback #%lu: hdl=%d, buf=%p, len=%d, bad=%d", 
                 callback_count, sync_conn_hdl, audio_buf, 
                 audio_buf ? audio_buf->data_len : 0, is_bad_frame);
    }
    
    // CRITICAL: Bad frames must be freed immediately
    if (is_bad_frame) {
        esp_hf_client_audio_buff_free(audio_buf);
        return;
    }
    
    // Store connection handle for uplink
    hfp_sync_conn_hdl = sync_conn_hdl;
    
    // Count received packets to know when audio path is stable
    hfp_rx_packet_count++;
    
    // CRITICAL: Use incoming packet as pacing signal for mic TX (natural rate limiting)
    // Phone sends packets every ~7.5ms, we send one mic packet for each incoming packet
    mic_tx_slot_available = true;
    
    // Enable mic transmission after receiving enough packets (audio buffers are initialized)
    // CRITICAL: Wait for more packets to ensure BT stack internal buffers are fully ready
    if (hfp_rx_packet_count == 50) {
        mic_tx_ready = true;
        ESP_LOGI(TAG, "*** HFP RX stable - enabling mic uplink transmission ***");
    }
    
    // Log what packet size the phone is using (helps us match uplink)
    static bool packet_size_logged = false;
    if (!packet_size_logged && audio_buf && audio_buf->data_len > 0) {
        ESP_LOGI(TAG, "Phone audio packet size: %d bytes (use same for uplink)", audio_buf->data_len);
        packet_size_logged = true;
    }
    
    // Incoming voice data from phone is mSBC ENCODED (60 bytes)
    // Need to DECODE to PCM before playing on speaker
    if (audio_buf && audio_buf->data && audio_buf->data_len > 0) {
        // Decode mSBC to PCM using ESP-ADF decoder
        // mSBC: 60 bytes encoded → 240 bytes PCM (16-bit mono @ 16kHz, 120 samples)
        int16_t pcm_buffer[240];  // Output buffer for decoded PCM
        
        if (msbc_decoder) {
            // Prepare input raw data
            esp_audio_dec_in_raw_t in_raw = {
                .buffer = audio_buf->data,
                .len = audio_buf->data_len,
                .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE
            };
            
            // Prepare output frame
            esp_audio_dec_out_frame_t out_frame = {
                .buffer = (uint8_t*)pcm_buffer,
                .len = sizeof(pcm_buffer)
            };
            
            // Decode info (optional, can be NULL)
            esp_audio_dec_info_t dec_info = {0};
            
            esp_audio_err_t ret = esp_sbc_dec_decode(msbc_decoder, &in_raw, &out_frame, &dec_info);
            
            if (ret == ESP_AUDIO_ERR_OK && out_frame.decoded_size > 0) {
                // Successfully decoded - write PCM to speaker
                size_t bytes_written = 0;
                i2s_write(I2S_NUM_0, pcm_buffer, out_frame.decoded_size, &bytes_written, pdMS_TO_TICKS(10));
                
                if (callback_count <= 5) {
                    ESP_LOGI(TAG, "HFP RX: %d bytes mSBC → %d bytes PCM → speaker (wrote %d bytes) [samples: %d, %d, %d]", 
                             audio_buf->data_len, out_frame.decoded_size, bytes_written, 
                             pcm_buffer[0], pcm_buffer[1], pcm_buffer[2]);
                } else if (callback_count % 100 == 0) {
                    ESP_LOGI(TAG, "HFP RX #%lu: [decoded samples: %d, %d, %d]", 
                             callback_count, pcm_buffer[0], pcm_buffer[1], pcm_buffer[2]);
                }
            } else {
                ESP_LOGW(TAG, "mSBC decode failed: ret=%d, decoded_size=%d", ret, out_frame.decoded_size);
            }
        } else {
            ESP_LOGW(TAG, "mSBC decoder not initialized!");
        }
    }
    
    // CRITICAL: MUST free the buffer after use (BT stack allocated it for us)
    // Failing to do this causes memory leak and crashes after ~800 packets
    esp_hf_client_audio_buff_free(audio_buf);
}

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
            ESP_LOGI(TAG, "HFP audio state: %d (0=disconn, 1=connecting, 2=conn, 3=conn_msbc)", 
                     param->audio_stat.state);
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                ESP_LOGI(TAG, "*** HFP AUDIO ACTIVE - Voice connection established ***");
                call_active = true;
                
                // CRITICAL: Read preferred frame size from phone
                hfp_preferred_frame_size = param->audio_stat.preferred_frame_size;
                ESP_LOGI(TAG, "Phone preferred frame size: %d bytes", hfp_preferred_frame_size);
                
                // Reconfigure I2S to 16kHz for HFP voice (more efficient than 44.1kHz + downsampling)
                ESP_LOGI(TAG, "Reconfiguring I2S to 16kHz for voice call...");
                
                // CRITICAL: Stop I2S and clear FIFOs before reconfiguration
                i2s_stop(I2S_NUM_0);
                i2s_zero_dma_buffer(I2S_NUM_0);
                i2s_stop(I2S_PORT_MIC);
                i2s_zero_dma_buffer(I2S_PORT_MIC);
                
                // Reconfigure both I2S ports for voice
                i2s_set_clk(I2S_NUM_0, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
                i2s_set_clk(I2S_PORT_MIC, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
                
                // Restart I2S
                i2s_start(I2S_NUM_0);
                i2s_start(I2S_PORT_MIC);
                
                ESP_LOGI(TAG, "I2S reconfigured: 16kHz mono for voice");
                
                // CRITICAL: Flush mic DMA buffers by reading and discarding initial samples
                // INMP441 may have stale data after clock change - discard first few reads
                ESP_LOGI(TAG, "Flushing mic buffers after reconfiguration...");
                uint8_t flush_buf[1024];
                size_t bytes_read;
                for (int i = 0; i < 5; i++) {
                    i2s_read(I2S_PORT_MIC, flush_buf, sizeof(flush_buf), &bytes_read, pdMS_TO_TICKS(50));
                    ESP_LOGI(TAG, "Mic flush #%d: read %d bytes", i+1, bytes_read);
                }
                ESP_LOGI(TAG, "Mic buffers flushed");
                
                // Initialize mSBC decoder for incoming voice
                ESP_LOGI(TAG, "Initializing mSBC decoder...");
                if (msbc_decoder == NULL) {
                    esp_sbc_dec_cfg_t dec_cfg = {
                        .sbc_mode = ESP_SBC_MODE_MSBC,  // mSBC mode for HFP
                        .ch_num = 1,                     // Mono (ignored for mSBC)
                        .enable_plc = 0                  // Disable packet loss concealment for now
                    };
                    
                    esp_audio_err_t ret = esp_sbc_dec_open(&dec_cfg, sizeof(dec_cfg), &msbc_decoder);
                    if (ret == ESP_AUDIO_ERR_OK && msbc_decoder) {
                        ESP_LOGI(TAG, "mSBC decoder initialized successfully");
                    } else {
                        ESP_LOGE(TAG, "Failed to create mSBC decoder! Error: %d", ret);
                    }
                }
                
                // Initialize mSBC encoder for outgoing voice (microphone)
                ESP_LOGI(TAG, "Initializing mSBC encoder...");
                if (msbc_encoder == NULL) {
                    // Use the official mSBC default configuration macro
                    esp_sbc_enc_config_t enc_cfg = ESP_SBC_MSBC_ENC_CONFIG_DEFAULT();
                    
                    esp_audio_err_t ret = esp_sbc_enc_open(&enc_cfg, sizeof(enc_cfg), &msbc_encoder);
                    if (ret == ESP_AUDIO_ERR_OK && msbc_encoder) {
                        // Query actual frame size expected by encoder
                        int in_size = 0, out_size = 0;
                        esp_sbc_enc_get_frame_size(msbc_encoder, &in_size, &out_size);
                        ESP_LOGI(TAG, "mSBC encoder initialized: in_size=%d, out_size=%d", in_size, out_size);
                    } else {
                        ESP_LOGE(TAG, "Failed to create mSBC encoder! Error: %d", ret);
                    }
                }
                
                // Register audio callback NOW (required for HCI + external codec mode)
                ESP_LOGI(TAG, "Registering HFP audio data callback...");
                esp_hf_client_register_audio_data_callback(bt_hfp_audio_cb);
                
                // IMPORTANT: Wait for HFP audio path to stabilize before enabling mic
                // Give the phone time to set up its end of the audio connection
                ESP_LOGI(TAG, "Waiting 500ms for HFP audio path to stabilize...");
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Enable microphone for call/recording
                mic_manager_enable(true);
                ESP_LOGI(TAG, "Microphone enabled for voice connection");
                
                // Request packet status reporting (tells phone we're sending audio)
                esp_err_t pkt_ret = esp_hf_client_pkt_stat_nums_get(0);
                if (pkt_ret == ESP_OK) {
                    ESP_LOGI(TAG, "PKT_STAT requested - phone should use glove mic");
                } else {
                    ESP_LOGW(TAG, "PKT_STAT request failed: %d", pkt_ret);
                }
            } else {
                ESP_LOGI(TAG, "HFP audio disconnected");
                call_active = false;
                
                // Destroy mSBC decoder
                if (msbc_decoder) {
                    esp_sbc_dec_close(msbc_decoder);
                    msbc_decoder = NULL;
                    ESP_LOGI(TAG, "mSBC decoder destroyed");
                }
                
                // Destroy mSBC encoder
                if (msbc_encoder) {
                    esp_sbc_enc_close(msbc_encoder);
                    msbc_encoder = NULL;
                    ESP_LOGI(TAG, "mSBC encoder destroyed");
                }
                
                // Disable microphone when voice ends
                mic_manager_enable(false);
                hfp_sync_conn_hdl = 0;
                hfp_rx_packet_count = 0;  // Reset packet counter
                mic_tx_ready = false;      // Reset mic TX flag
                mic_tx_slot_available = false;  // Reset rate limiting flag
                ESP_LOGI(TAG, "Microphone disabled");
                
                // Restore I2S to 44.1kHz for A2DP music playback
                ESP_LOGI(TAG, "Restoring I2S to 44.1kHz for music...");
                
                // Stop and clear FIFOs
                i2s_stop(I2S_NUM_0);
                i2s_zero_dma_buffer(I2S_NUM_0);
                i2s_stop(I2S_PORT_MIC);
                i2s_zero_dma_buffer(I2S_PORT_MIC);
                
                // Reconfigure back to music mode
                i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
                i2s_set_clk(I2S_PORT_MIC, 44100, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
                
                // Restart I2S
                i2s_start(I2S_NUM_0);
                i2s_start(I2S_PORT_MIC);
                
                ESP_LOGI(TAG, "I2S restored: 44.1kHz stereo for music");
            }
            break;
            
        case ESP_HF_CLIENT_BVRA_EVT:
            ESP_LOGI(TAG, "Voice recognition state: %d", param->bvra.value);
            break;
            
        case ESP_HF_CLIENT_CIND_CALL_EVT:
            ESP_LOGI(TAG, "Call indicator: %d", param->call.status);
            if (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS) {
                ESP_LOGI(TAG, "*** CALL IN PROGRESS ***");
                display_show_message("Call Active");
                // Note: call_active and mic are managed by AUDIO_STATE_EVT
            } else if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS) {
                ESP_LOGI(TAG, "No active calls");
                display_show_message("Call Ended");
                // Note: call_active and mic are managed by AUDIO_STATE_EVT
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
    
    // Note: HFP audio callback is registered dynamically when audio connects
    // (required for HCI mode with external codec)
    
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_LOGI(TAG, "Step 7: HFP initialized - hands-free calling ready");
    
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

void bluetooth_init_microphone(void) {
    ESP_LOGI(TAG, "Initializing microphone for HFP (I2S Port 1)...");
    esp_err_t mic_ret = mic_manager_init();
    if (mic_ret == ESP_OK) {
        mic_manager_register_callback(mic_data_ready_cb);
        mic_manager_start_task();
        ESP_LOGI(TAG, "*** Microphone initialized - ready for calls ***");
    } else {
        ESP_LOGW(TAG, "Microphone init failed, continuing without mic: %d", mic_ret);
    }
}
