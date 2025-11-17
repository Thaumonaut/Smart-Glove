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
#include "esp_avrc_api.h"  // AVRC for metadata
#include "esp_hf_client_api.h"  // HFP client API
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"

// ESP-ADF SBC decoder for mSBC (decoder works fine, keep it)
#include "decoder/esp_audio_dec.h"
#include "decoder/impl/esp_sbc_dec.h"

// BlueZ libsbc encoder (stable replacement for ESP-ADF encoder)
#include "sbc.h"

static const char *TAG = "Bluetooth";
// Trace toggles
static bool hfp_trace_full = true; // default to true per user's request to enable tracing unless disabled

// Small helper to checksum a buffer for quick corruption detection (first/last bytes)
static uint32_t buf_checksum(const uint8_t *buf, size_t len, size_t max_sample_bytes) {
    if (!buf || len == 0) return 0;
    uint32_t sum = 0;
    size_t n = max_sample_bytes;
    if (n > len) n = len;
    // Sum first n bytes
    for (size_t i = 0; i < n; ++i) sum += buf[i];
    // Sum last n bytes
    if (len > n) {
        size_t start = (len > n) ? (len - n) : 0;
        for (size_t i = start; i < len; ++i) sum += buf[i];
    }
    return sum;
}

// Log details of an HFP audio buffer (safely)
static void hfp_log_buffer_info(const char *where, esp_hf_audio_buff_t *b) {
    if (!b) {
        ESP_LOGD(TAG, "%s: HFP buf=null", where);
        return;
    }
    bool data_dma = esp_ptr_dma_capable(b->data);
    uintptr_t aligned = ((uintptr_t)b->data & 0x3) == 0;
    uint32_t chks = 0;
    if (b->data && b->data_len > 0) chks = buf_checksum(b->data, b->data_len, 8);
    ESP_LOGD(TAG, "%s: hfp_buf=%p data=%p buff_size=%d data_len=%d dma=%d align=%d checksum=0x%08x", where,
             b, b->data, (int)b->buff_size, (int)b->data_len, (int)data_dma, (int)aligned, (unsigned)chks);
}

// Bluetooth state
static bool bt_connected = false;
static bool audio_playing = false;

// A2DP metadata state (AVRC - Audio/Video Remote Control)
static char current_track_title[128] = {0};
static char current_track_artist[128] = {0};
static bool metadata_available = false;

// HFP state
// CRITICAL: Use volatile to prevent multi-core race conditions
static volatile bool hfp_connected = false;
static volatile bool call_active = false;
static esp_hf_call_status_t call_status = ESP_HF_CALL_STATUS_NO_CALLS;

// mSBC decoder/encoder state
// CRITICAL: Use volatile to prevent compiler optimization issues in multi-core access
static volatile void *msbc_decoder = NULL;
static volatile void *msbc_encoder = NULL;
static volatile esp_hf_sync_conn_hdl_t hfp_sync_conn_hdl = 0;  // Store connection handle for uplink
static uint32_t hfp_rx_packet_count = 0;  // Count packets received from phone
static bool mic_tx_ready = false;  // Only send mic data after receiving packets from phone
static uint16_t hfp_preferred_frame_size = 60;  // Frame size from phone (set in AUDIO_STATE_EVT)
static volatile bool mic_tx_slot_available = false;  // Set to true each time we receive audio from phone

// Mic TX worker queue/task for encoding and sending uplink data from a single safe context
static QueueHandle_t mic_tx_queue = NULL;
static RingbufHandle_t hfp_ringbuf = NULL; // ring buffer carrying pointers to esp_hf_audio_buff_t
static QueueHandle_t hfp_send_queue = NULL; // optional queue fallback for debugging
static bool hfp_use_queue = false; // if true, use hfp_send_queue instead of hfp_ringbuf
static TaskHandle_t hfp_sender_task_handle = NULL;
static uint32_t mic_tx_drops = 0;
static uint32_t hfp_send_drops = 0;
static uint32_t hfp_encode_fallback_count = 0;
static bool hfp_force_safe_encode = false;
static bool hfp_disable_encoder = false; // If true, skip calling esp_sbc_enc_process (debug)
// debug telemetry
static uint32_t telemetry_tick = 0;
static void hfp_telemetry_task(void *pv);
static TaskHandle_t mic_tx_task_handle = NULL;
typedef struct {
    uint8_t *pcm;     // PCM data (16-bit mono) length = msbc_in_size
    size_t len;       // Length in bytes
} mic_tx_item_t;

static void hfp_sender_task(void *pv);

static void mic_tx_task(void *pv) {
    (void)pv;
    ESP_LOGI(TAG, "Mic TX task started");

    // CRITICAL FIX: Use BlueZ libsbc encoder (stable, battle-tested)
    // Create encoder locally on THIS core (Core 1)
    sbc_t *sbc_encoder = (sbc_t *)heap_caps_malloc(sizeof(sbc_t), MALLOC_CAP_8BIT);
    if (!sbc_encoder) {
        ESP_LOGE(TAG, ">>>MIC_TX_TASK: Failed to allocate sbc_t structure!");
        ESP_LOGE(TAG, ">>>MIC_TX_TASK: CANNOT PROCEED - Task will exit");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, ">>>MIC_TX_TASK: Initializing libsbc mSBC encoder on Core 1...");
    
    // Initialize SBC encoder
    int ret = sbc_init(sbc_encoder, 0);
    if (ret < 0) {
        ESP_LOGE(TAG, ">>>MIC_TX_TASK: sbc_init failed: %d", ret);
        heap_caps_free(sbc_encoder);
        vTaskDelete(NULL);
        return;
    }
    
    // Configure for mSBC (HFP wideband)
    // mSBC parameters: 16kHz, 15 blocks, 8 subbands, mono, bitpool 26
    sbc_encoder->frequency = SBC_FREQ_16000;     // 16kHz for HFP wideband
    sbc_encoder->blocks = SBC_BLK_16;            // 15 blocks (closest is 16)
    sbc_encoder->subbands = SBC_SB_8;            // 8 subbands
    sbc_encoder->mode = SBC_MODE_MONO;           // Mono for HFP
    sbc_encoder->allocation = SBC_AM_LOUDNESS;   // Loudness allocation
    sbc_encoder->bitpool = 26;                   // mSBC standard bitpool
    sbc_encoder->endian = SBC_LE;                // Little-endian for ESP32
    
    // Get frame sizes
    size_t in_size = sbc_get_codesize(sbc_encoder);      // PCM input size
    size_t out_size = sbc_get_frame_length(sbc_encoder); // Encoded output size
    
    ESP_LOGI(TAG, ">>>MIC_TX_TASK: libsbc encoder initialized successfully");
    ESP_LOGI(TAG, ">>>MIC_TX_TASK: Frame sizes - in=%d bytes, out=%d bytes", in_size, out_size);

    while (1) {
        ESP_LOGI(TAG, ">>>MIC_TX_TASK: TOP OF LOOP - about to wait for queue");
        mic_tx_item_t item;
        if (xQueueReceive(mic_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: GOT ITEM ptr=%p len=%d", item.pcm, (int)item.len);
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: About to check DMA capability");
            bool is_dma = esp_ptr_dma_capable(item.pcm);
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: DMA check done: %d", (int)is_dma);
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: About to calculate checksum");
            uint32_t chk = buf_checksum(item.pcm, item.len, 8);
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: Checksum done: 0x%08x", (unsigned)chk);
            
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: About to check call state variables");
            // CRITICAL: Load volatile variables into local copies to prevent multi-core race
            // Core 0 (HFP callback) can modify these while Core 1 (this task) reads them
            bool call_active_local = call_active;
            bool hfp_connected_local = hfp_connected;
            esp_hf_sync_conn_hdl_t hdl_local = hfp_sync_conn_hdl;
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: State snapshot: call=%d hfp=%d enc=%p hdl=%d", 
                     call_active_local, hfp_connected_local, (void*)sbc_encoder, hdl_local);
            
            // If call is not active, skip (encoder is task-local and always valid)
            if (!call_active_local || !hfp_connected_local || hdl_local == 0) {
                ESP_LOGI(TAG, ">>>MIC_TX_TASK: Skipping - call not ready");
                if (item.pcm) heap_caps_free(item.pcm);
                continue;
            }
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: Call state OK, proceeding with encode");

            // Encode PCM to mSBC using task-local libsbc encoder
            sbc_t *encoder = sbc_encoder;  // Use our task-local libsbc encoder
            
            // Validate encoder
            if (encoder == NULL) {
                ESP_LOGE(TAG, ">>>MIC_TX_TASK: ENCODER IS NULL! Skipping frame");
                if (item.pcm) heap_caps_free(item.pcm);
                continue;
            }
            
            // Get frame sizes from libsbc encoder
            size_t msbc_out_size = sbc_get_frame_length(encoder);  // Output encoded size
            ESP_LOGI(TAG, ">>>MIC_TX_TASK: libsbc frame size=%zu bytes", msbc_out_size);

            ESP_LOGI(TAG, ">>>MIC_TX_TASK: msbc_out_size=%zu, force_safe_encode=%d", msbc_out_size, (int)hfp_force_safe_encode);
            
            // CRITICAL FIX: Only allocate HFP buffer if NOT using forced safe encode mode
            // The zero-copy path requires calling esp_hf_client_audio_buff_alloc() early which can crash
            // if the HFP connection state changes on Core 0 while we're on Core 1
            esp_hf_audio_buff_t *hfp_buf = NULL;
            
            if (!hfp_force_safe_encode) {
                ESP_LOGI(TAG, ">>>MIC_TX_TASK: About to allocate HFP buffer (%zu bytes)", msbc_out_size);
                
                // Re-check call state before allocating HFP buffer
                if (!call_active_local || !hfp_connected_local) {
                    ESP_LOGE(TAG, ">>>MIC_TX_TASK: Call/HFP state changed! call=%d hfp=%d - aborting frame", 
                             call_active_local, hfp_connected_local);
                    if (item.pcm) heap_caps_free(item.pcm);
                    continue;
                }
                
                // Allocate HFP buffer for potential zero-copy path
                hfp_buf = esp_hf_client_audio_buff_alloc((uint16_t)msbc_out_size);
                ESP_LOGI(TAG, ">>>MIC_TX_TASK: HFP buffer allocation returned: buf=%p", hfp_buf);
                if (!hfp_buf || !hfp_buf->data) {
                    ESP_LOGW(TAG, "Failed to allocate HFP buffer for mSBC output (%d bytes)", (int)msbc_out_size);
                    if (hfp_buf) esp_hf_client_audio_buff_free(hfp_buf);
                    if (item.pcm) heap_caps_free(item.pcm);
                    continue;
                }
            } else {
                ESP_LOGI(TAG, ">>>MIC_TX_TASK: Safe encode mode - HFP buffer will be allocated later in fallback path");
            }

            // Input PCM data from queue item
            const void *in_buffer = item.pcm;
            size_t in_len = item.len;

            // CRITICAL: Only check DMA capability if we have an HFP buffer
            // When hfp_force_safe_encode=true, hfp_buf is NULL and we skip to fallback path
            bool hfp_buf_dma = false;
            if (hfp_buf != NULL) {
                // Prefer zero-copy into the HFP buffer ONLY if that buffer is DMA-capable.
                // Otherwise encode into a DMA-capable temp buffer and copy into the HFP buffer
                // Ensure HFP buffer is DMA-capable and word-aligned for encoder
                hfp_buf_dma = esp_ptr_dma_capable(hfp_buf->data) && (((uintptr_t)hfp_buf->data & 0x3) == 0);
                // Also make sure the HFP buffer is large enough for an encoded frame;
                // `esp_hf_client_audio_buff_alloc` may return a smaller `buff_size` than requested.
                if ((size_t)hfp_buf->buff_size < msbc_out_size) {
                    ESP_LOGD(TAG, "HFP buffer too small for zero-copy (%d < %d), will fallback", hfp_buf->buff_size, (int)msbc_out_size);
                    hfp_buf_dma = false;
                }
                // Ensure PCM buffer is also DMA-aligned for encoder
                if (((uintptr_t)item.pcm & 0x3) != 0) {
                    ESP_LOGD(TAG, "PCM buffer not 4-byte aligned, will fallback to safe encode\n");
                    hfp_buf_dma = false;
                }
            }
            
            // Encode using libsbc
            ssize_t encoded_bytes = 0;
            int ret = 0;  // libsbc returns negative on error, positive = bytes written
            
            if (hfp_buf_dma && !hfp_force_safe_encode) {
                // Zero-copy path: encode directly into the HFP buffer
                ESP_LOGD(TAG, "Using zero-copy mSBC encode into HFP buffer (DMA)");
                ESP_LOGD(TAG, "ENC-ZERO-COPY: hfp_buf=%p size=%d hfp_buf_dma=%d force_safe=%d in_len=%zu enc=%p", 
                         hfp_buf->data, (int)hfp_buf->buff_size, (int)hfp_buf_dma, 
                         (int)hfp_force_safe_encode, in_len, encoder);
                
                if (hfp_disable_encoder) {
                    ESP_LOGW(TAG, "ENC-DEBUG: hfp_disable_encoder active — skipping encoder (zero-copy path)");
                    encoded_bytes = 0;
                    ret = 0;  // Success but no encoding
                } else {
                    // Use libsbc to encode directly into HFP buffer
                    ret = sbc_encode(sbc_encoder, 
                                    in_buffer, in_len,
                                    hfp_buf->data, hfp_buf->buff_size,
                                    &encoded_bytes);
                }
                
                ESP_LOGD(TAG, "ENC-ZERO-COPY result=%d enc_bytes=%zd checksum=0x%08x", 
                         ret, encoded_bytes,
                         (unsigned)buf_checksum(hfp_buf->data, encoded_bytes, 8));
            } else {
                ESP_LOGD(TAG, "HFP buffer not DMA-capable; using DMA temp buffer for mSBC encode");
                hfp_encode_fallback_count++;
            }
            
            // Check encoding success (libsbc returns positive bytes or negative error)
            if (ret >= 0 && encoded_bytes > 0) {
                // Verify encoded bytes fit the allocated HFP buffer
                if ((size_t)encoded_bytes > hfp_buf->buff_size) {
                    ESP_LOGW(TAG, "Encoder produced %zd bytes > allocated %d, dropping frame",
                             encoded_bytes, (int)hfp_buf->buff_size);
                    esp_hf_client_audio_buff_free(hfp_buf);
                    if (item.pcm) heap_caps_free(item.pcm);
                    continue;
                }

                // Set final encoded length and use zero-copy by passing the buffer pointer
                hfp_buf->data_len = encoded_bytes;
                hfp_buf->buff_size = encoded_bytes;
                ESP_LOGD(TAG, "ENC-ZERO-COPY: encoded_bytes=%zd checksum=0x%08x", encoded_bytes,
                         (unsigned)buf_checksum(hfp_buf->data, encoded_bytes, 8));

                if (hfp_ringbuf) {
                    esp_hf_audio_buff_t *buf_ptr = hfp_buf;
                    // Choose queue vs ringbuffer depending on debug flag
                    if (hfp_use_queue && hfp_send_queue) {
                        ESP_LOGD(TAG, "HFP push: queued via queue hfp_buf=%p len=%d (zero_copy=%d) checksum=0x%08x", hfp_buf, hfp_buf->data_len, hfp_buf_dma && !hfp_force_safe_encode,
                                 (unsigned)buf_checksum(hfp_buf->data, hfp_buf->data_len, 8));
                        if (xQueueSend(hfp_send_queue, &buf_ptr, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "HFP send queue full, dropping encoded frame");
                            hfp_send_drops++;
                            esp_hf_client_audio_buff_free(hfp_buf);
                        }
                    } else {
                        ESP_LOGD(TAG, "HFP push: queued hfp_buf=%p len=%d (zero_copy=%d) checksum=0x%08x", hfp_buf, hfp_buf->data_len, hfp_buf_dma && !hfp_force_safe_encode,
                                 (unsigned)buf_checksum(hfp_buf->data, hfp_buf->data_len, 8));
                        BaseType_t rb_ret = xRingbufferSend(hfp_ringbuf, &buf_ptr, sizeof(buf_ptr), 0);
                        if (rb_ret != pdTRUE) {
                            ESP_LOGW(TAG, "HFP ringbuffer full, dropping encoded frame");
                            hfp_send_drops++;
                            esp_hf_client_audio_buff_free(hfp_buf);
                        }
                    }
                } else {
                    // Fallback: send immediately if ringbuffer isn't available
                    esp_err_t send_ret = esp_hf_client_audio_data_send(hfp_sync_conn_hdl, hfp_buf);
                    if (send_ret != ESP_OK) {
                        ESP_LOGW(TAG, "Mic TX send failed (fallback): %d", send_ret);
                        esp_hf_client_audio_buff_free(hfp_buf);
                    }
                }
            } else {
                // Fallback path: encode into DMA temp buffer, then copy to HFP buffer
                ESP_LOGI(TAG, ">>>MIC_TX_TASK: Entering safe encode fallback path");
                
                // Allocate HFP buffer if we haven't already (happens when hfp_force_safe_encode is set)
                esp_hf_audio_buff_t *hfp_buf_fallback = hfp_buf;
                if (!hfp_buf_fallback) {
                    ESP_LOGI(TAG, ">>>MIC_TX_TASK: Allocating HFP buffer in fallback path (%zu bytes)", msbc_out_size);
                    hfp_buf_fallback = esp_hf_client_audio_buff_alloc((uint16_t)msbc_out_size);
                    ESP_LOGI(TAG, ">>>MIC_TX_TASK: Fallback HFP buffer: %p", hfp_buf_fallback);
                    if (!hfp_buf_fallback || !hfp_buf_fallback->data) {
                        ESP_LOGW(TAG, "Failed to allocate HFP buffer in fallback path");
                        if (hfp_buf_fallback) esp_hf_client_audio_buff_free(hfp_buf_fallback);
                        if (item.pcm) heap_caps_free(item.pcm);
                        continue;
                    }
                }
                
                // allocate tmp buffer with slight headroom to avoid encoder writing past end
                size_t msbc_tmp_alloc = msbc_out_size + 64;
                uint8_t *msbc_tmp = (uint8_t *)heap_caps_malloc(msbc_tmp_alloc, MALLOC_CAP_DMA);
                if (!msbc_tmp) {
                    ESP_LOGW(TAG, "mSBC fallback allocation failed (%d bytes)", (int)msbc_out_size);
                    esp_hf_client_audio_buff_free(hfp_buf);
                    if (item.pcm) heap_caps_free(item.pcm);
                    continue;
                }

                // Fill sentinel guard bytes before/after the usable region so we can detect overflow
                const uint8_t GUARD = 0xA5;
                size_t GUARD_SZ = 16;
                if (msbc_tmp_alloc > GUARD_SZ * 2) {
                    memset(msbc_tmp, GUARD, GUARD_SZ);
                    memset(msbc_tmp + msbc_tmp_alloc - GUARD_SZ, GUARD, GUARD_SZ);
                }
                
                ESP_LOGD(TAG, "ENC-FALLBACK: msbc_tmp=%p alloc=%u in_len=%zu enc_handle=%p", 
                         msbc_tmp, (unsigned)msbc_tmp_alloc, in_len, encoder);
                
                ssize_t fallback_encoded_bytes = 0;
                int ret2;
                
                if (hfp_disable_encoder) {
                    ESP_LOGW(TAG, "ENC-DEBUG: hfp_disable_encoder active — skipping encoder (fallback path)");
                    fallback_encoded_bytes = 0;
                    ret2 = 0;  // Success but no encoding
                } else {
                    // Use libsbc to encode into temporary buffer
                    ESP_LOGI(TAG, ">>>MIC_TX_TASK: PRE-ENCODE: encoder=%p, in.buffer=%p, in.len=%zu, out.buffer=%p, out.len=%u",
                             encoder, in_buffer, in_len, msbc_tmp, msbc_tmp_alloc);
                    
                    ret2 = sbc_encode(sbc_encoder,
                                     in_buffer, in_len,
                                     msbc_tmp, msbc_tmp_alloc,
                                     &fallback_encoded_bytes);
                    
                    ESP_LOGI(TAG, ">>>MIC_TX_TASK: POST-ENCODE: ret=%d, encoded_bytes=%zd", 
                             ret2, fallback_encoded_bytes);
                }
                
                ESP_LOGD(TAG, "sbc_encode ret=%d encoded_bytes=%zd checksum=0x%08x", 
                         ret2, fallback_encoded_bytes,
                         (unsigned)buf_checksum(msbc_tmp, fallback_encoded_bytes, 8));
                // Check sentinel guards for overflow
                if (msbc_tmp_alloc > GUARD_SZ * 2) {
                    bool corrupted = false;
                    for (size_t i = 0; i < GUARD_SZ; ++i) {
                        if (msbc_tmp[i] != GUARD) { corrupted = true; break; }
                    }
                    for (size_t i = 0; i < GUARD_SZ && !corrupted; ++i) {
                        if (msbc_tmp[msbc_tmp_alloc - GUARD_SZ + i] != GUARD) { corrupted = true; break; }
                    }
                    if (corrupted) {
                        ESP_LOGW(TAG, "ENC-FALLBACK: buffer overflow detected for msbc_tmp=%p alloc=%u", msbc_tmp, (unsigned)msbc_tmp_alloc);
                    }
                }
                
                // Check encoding success and copy to HFP buffer
                if (ret2 >= 0 && fallback_encoded_bytes > 0 && 
                    (size_t)fallback_encoded_bytes <= hfp_buf_fallback->buff_size) {
                    memcpy(hfp_buf_fallback->data, msbc_tmp, fallback_encoded_bytes);
                    hfp_buf_fallback->data_len = fallback_encoded_bytes;
                    hfp_buf_fallback->buff_size = fallback_encoded_bytes;
                    if (hfp_ringbuf) {
                        ESP_LOGD(TAG, "ENC-FALLBACK: copied into hfp_buf=%p data_len=%d checksum=0x%08x", hfp_buf_fallback, hfp_buf_fallback->data_len,
                                 (unsigned)buf_checksum(hfp_buf_fallback->data, hfp_buf_fallback->data_len, 8));
                        esp_hf_audio_buff_t *buf_ptr = hfp_buf_fallback;
                        if (hfp_use_queue && hfp_send_queue) {
                            if (xQueueSend(hfp_send_queue, &buf_ptr, 0) != pdTRUE) {
                                ESP_LOGW(TAG, "HFP send queue full, dropping encoded frame (fallback)");
                                hfp_send_drops++;
                                esp_hf_client_audio_buff_free(hfp_buf_fallback);
                            }
                        } else {
                            BaseType_t rb_ret = xRingbufferSend(hfp_ringbuf, &buf_ptr, sizeof(buf_ptr), 0);
                            if (rb_ret != pdTRUE) {
                                ESP_LOGW(TAG, "HFP ringbuffer full, dropping encoded frame (fallback)");
                                hfp_send_drops++;
                                esp_hf_client_audio_buff_free(hfp_buf_fallback);
                            }
                        }
                    } else {
                        esp_err_t send_ret = esp_hf_client_audio_data_send(hfp_sync_conn_hdl, hfp_buf_fallback);
                        if (send_ret != ESP_OK) {
                            ESP_LOGW(TAG, "Mic TX send failed (fallback send): %d", send_ret);
                            esp_hf_client_audio_buff_free(hfp_buf_fallback);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "mSBC fallback failed: ret=%d encoded=%zd", ret2, fallback_encoded_bytes);
                    esp_hf_client_audio_buff_free(hfp_buf_fallback);
                }

                heap_caps_free(msbc_tmp);
            }
            if (item.pcm) heap_caps_free(item.pcm);
        }
    }
    
    // Cleanup encoder when task exits
    if (sbc_encoder) {
        sbc_finish(sbc_encoder);
        heap_caps_free(sbc_encoder);
        ESP_LOGI(TAG, "libsbc encoder destroyed");
    }
}

// Dedicated HFP sender: consume prepared esp_hf_audio_buff_t* and send to the phone
static void hfp_sender_task(void *pv) {
    (void)pv;
    ESP_LOGI(TAG, "HFP sender task started");

    while (1) {
        ESP_LOGI(TAG, ">>>HFP_SENDER: TOP OF LOOP - about to wait for data");
        esp_hf_audio_buff_t *send_buf = NULL;
        // Optional queue path for debugging: read from queue instead of ringbuffer
        if (hfp_use_queue && hfp_send_queue) {
            ESP_LOGI(TAG, ">>>HFP_SENDER: waiting on QUEUE");
            if (xQueueReceive(hfp_send_queue, &send_buf, portMAX_DELAY) != pdTRUE) {
                continue;
            }
            ESP_LOGI(TAG, ">>>HFP_SENDER: got buf from QUEUE: %p", send_buf);
        } else {
            ESP_LOGI(TAG, ">>>HFP_SENDER: waiting on RINGBUFFER");
            size_t item_size;
            void *item = xRingbufferReceive(hfp_ringbuf, &item_size, portMAX_DELAY);
            if (item != NULL && item_size == sizeof(esp_hf_audio_buff_t*)) {
                send_buf = *((esp_hf_audio_buff_t**)item);
                ESP_LOGI(TAG, ">>>HFP_SENDER: got buf from RINGBUF: %p (returning ringbuf item)", send_buf);
                vRingbufferReturnItem(hfp_ringbuf, item);
                ESP_LOGI(TAG, ">>>HFP_SENDER: ringbuf item returned successfully");
            }
        }
        // Log details of the HFP buffer before we wait for a slot
        ESP_LOGI(TAG, ">>>HFP_SENDER: about to log buffer info");
        hfp_log_buffer_info("HFP-sender-pre-slot", send_buf);
        ESP_LOGI(TAG, ">>>HFP_SENDER: buffer info logged, now waiting for TX slot");
            // Wait for a TX slot from the incoming HFP audio callback
            while (!mic_tx_slot_available) {
                vTaskDelay(pdMS_TO_TICKS(1));
                // If call went away, free and break
                if (!call_active || !hfp_connected || hfp_sync_conn_hdl == 0) {
                    if (send_buf) esp_hf_client_audio_buff_free(send_buf);
                    send_buf = NULL;
                    break;
                }
            }
            if (!send_buf) continue;

            // Consume the available slot
            mic_tx_slot_available = false;

            esp_err_t send_ret = esp_hf_client_audio_data_send(hfp_sync_conn_hdl, send_buf);
            ESP_LOGD(TAG, "HFP sender: esp_hf_client_audio_data_send returned %d for buf=%p", send_ret, send_buf);
            if (send_ret != ESP_OK) {
                ESP_LOGW(TAG, "HFP sender task: failed to send (ret=%d)", send_ret);
                esp_hf_client_audio_buff_free(send_buf);
            }
        
    }
}

// Microphone data callback - called by mic_manager when audio is available
static void mic_data_ready_cb(const uint8_t *data, size_t len) {
    static uint32_t packet_count = 0;
    packet_count++;
    
    // DEBUG: Log every call for first few, then periodically
    if (packet_count <= 10 || packet_count % 100 == 0) {
        // Also log first few raw samples to debug microphone reading
        const int32_t *samples_32 = (const int32_t *)data;
        int32_t s0 = 0, s1 = 0, s2 = 0;
        size_t num_samples_dbg = (len / 4);
        if (num_samples_dbg >= 1) s0 = samples_32[0];
        if (num_samples_dbg >= 2) s1 = samples_32[1];
        if (num_samples_dbg >= 3) s2 = samples_32[2];
        ESP_LOGI(TAG, "MIC CALLBACK #%lu: call=%d, hfp=%d, hdl=%d, ready=%d, rx=%lu, slot=%d, len=%d [raw32: %ld, %ld, %ld]",
                 packet_count, call_active, hfp_connected, hfp_sync_conn_hdl,
                 mic_tx_ready, hfp_rx_packet_count, mic_tx_slot_available, len,
                 s0, s1, s2);
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
    // Where possible, query the encoder frame size dynamically, but default to 240.
    #define MSBC_PCM_INPUT_SIZE_DEFAULT 240  // 120 samples × 2 bytes per sample = 240 bytes
    
    // Use accumulator for samples until we have enough for one mSBC frame
    static uint8_t sample_buffer[MSBC_PCM_INPUT_SIZE_DEFAULT];
    static size_t sample_buf_pos = 0;
    
    // Process 32-bit mono samples from INMP441
    const int32_t *samples_32 = (const int32_t *)data;
    size_t num_samples = len / 4;  // 32-bit samples

    // Debug: log first few PCM samples as 16-bit (after conversion) to trace data inputs
    if (num_samples > 0) {
        int16_t debug_first = (int16_t)(samples_32[0] >> 16) - 256;
        ESP_LOGD(TAG, "MIC DEBUG: first sample(32->16)=%d num_samples=%d", debug_first, (int)num_samples);
    }
    
    // Determine target PCM input size for one mSBC frame
    size_t msbc_in_size = MSBC_PCM_INPUT_SIZE_DEFAULT;
    // NOTE: Encoder frame size query disabled - using task-local encoder with fixed size
    // The task-local encoder in mic_tx_task will handle frame sizing
    /*
    void *encoder_local = (void*)msbc_encoder;  // Local non-volatile copy
    if (encoder_local) {
        int tmp_in = 0, tmp_out = 0;
    if (esp_sbc_enc_get_frame_size(encoder_local, &tmp_in, &tmp_out) == ESP_AUDIO_ERR_OK) {
            // tmp_in is number of PCM bytes expected
            if (tmp_in > 0 && tmp_in <= MSBC_PCM_INPUT_SIZE_DEFAULT) {
                msbc_in_size = (size_t)tmp_in;
            }
        }
    }
    */

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
    if (sample_buf_pos >= (size_t)msbc_in_size) {
            // CRITICAL: Consume the slot BEFORE sending
            // This prevents sending multiple packets in one callback
            mic_tx_slot_available = false;
            
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: sample buffer FULL (%d bytes), about to queue for encoder", (int)msbc_in_size);
            
            // Instead of encoding and sending on the mic task, enqueue the PCM frame for
            // processing by the mic_tx_task. This ensures the encoder + HFP send happen
            // from a single task (safer with the BT stack and encoder).
            if (mic_tx_queue == NULL) {
                ESP_LOGW(TAG, "Mic TX queue not available, dropping frame");
                sample_buf_pos = 0;
                return;
            }

            mic_tx_item_t item;
            item.len = msbc_in_size;
            // Use DMA-capable allocation for PCM to ensure encoder can access it safely
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: allocating DMA buffer for PCM (%d bytes)", (int)item.len);
            item.pcm = (uint8_t *)heap_caps_malloc(item.len, MALLOC_CAP_DMA);
            if (!item.pcm) {
                ESP_LOGW(TAG, "Failed to alloc PCM copy for mic TX");
                sample_buf_pos = 0;
                return;
            }
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: DMA buffer allocated at %p", item.pcm);
            // Debug: checksum the prepared PCM frame for corruption tracing
            uint32_t pcm_sum = buf_checksum(sample_buffer, item.len, 8);
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: PCM checksum=0x%08x, about to memcpy", (unsigned)pcm_sum);
            memcpy(item.pcm, sample_buffer, item.len);
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: memcpy done, about to send to queue");

            // Non-blocking send to queue to avoid delaying mic task — if queue is full we drop frame
            if (xQueueSend(mic_tx_queue, &item, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Mic TX queue full, dropping frame");
                mic_tx_drops++;
                if (item.pcm) heap_caps_free(item.pcm);
                sample_buf_pos = 0;
                return;
            }
            ESP_LOGI(TAG, ">>>MIC_CALLBACK: queued PCM frame successfully, returning");

            sample_buf_pos = 0;  // Reset buffer for next accumulation

            // Return immediately after queuing ONE packet
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
        ESP_LOGI(TAG, "HFP: force_safe_encode=%d, use_hfp_queue=%d, hfp_ringbuf=%p, hfp_send_queue=%p, msbc_encoder=%p", 
                 (int)hfp_force_safe_encode, (int)hfp_use_queue, hfp_ringbuf, hfp_send_queue, msbc_encoder);
        // TEMPORARILY DISABLED: Don't enable log filtering until we identify the crash point
        // bluetooth_enable_mic_trace_only(true);
        ESP_LOGI(TAG, ">>> TRACE FILTERING DISABLED - ALL LOGS VISIBLE FOR CRASH DEBUG");
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
        
        void *decoder_local = (void*)msbc_decoder;  // Local non-volatile copy
        if (decoder_local) {
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
            
            esp_audio_err_t ret = esp_sbc_dec_decode(decoder_local, &in_raw, &out_frame, &dec_info);
            
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
        ESP_LOGI(TAG, "*** audio_playing flag set to TRUE ***");
        ESP_LOGI(TAG, "*** Volume: 100%% (phone controls volume) ***");
        ESP_LOGI(TAG, "*** GAIN: 15dB (maximum output) ***");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "");
    }
}

/**
 * AVRC callback - handles metadata (track title, artist, etc.)
 */
static void bt_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    switch (event) {
        case ESP_AVRC_CT_METADATA_RSP_EVT: {
            // ESP-IDF delivers metadata one attribute at a time
            uint8_t attr_id = param->meta_rsp.attr_id;
            const char *attr_text = (const char *)param->meta_rsp.attr_text;
            int attr_len = param->meta_rsp.attr_length;
            
            if (!attr_text || attr_len == 0) {
                break;
            }
            
            if (attr_id == ESP_AVRC_MD_ATTR_TITLE) {
                snprintf(current_track_title, sizeof(current_track_title), "%.*s", 
                         attr_len, attr_text);
                ESP_LOGI(TAG, "Track: %s", current_track_title);
                metadata_available = true;
            } else if (attr_id == ESP_AVRC_MD_ATTR_ARTIST) {
                snprintf(current_track_artist, sizeof(current_track_artist), "%.*s", 
                         attr_len, attr_text);
                ESP_LOGI(TAG, "Artist: %s", current_track_artist);
                metadata_available = true;
            }
            break;
        }
        
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "AVRC connection state: %s", 
                     param->conn_stat.connected ? "connected" : "disconnected");
            break;
            
        case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
            ESP_LOGD(TAG, "AVRC passthrough response: key_code 0x%x, key_state %d",
                     param->psth_rsp.key_code, param->psth_rsp.key_state);
            break;
            
        default:
            ESP_LOGD(TAG, "AVRC event: %d", event);
            break;
    }
}

/**
 * A2DP callback - handles connection state changes
 */
static void bt_a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "*** A2DP CONNECTED ***");
                ESP_LOGI(TAG, "*** bt_connected flag set to TRUE ***");
                ESP_LOGI(TAG, "========================================");
                bt_connected = true;

                // Vibrate to confirm connection
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level((gpio_num_t)VIBRATION_MOTOR_PIN, 0);
            } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "A2DP disconnected");
                bt_connected = false;
                audio_playing = false;
                metadata_available = false;  // Clear metadata on disconnect
                current_track_title[0] = '\0';
                current_track_artist[0] = '\0';
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "*** AUDIO STATE: STARTED ***");
                ESP_LOGI(TAG, "========================================");
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
                    
                    void *dec_temp = NULL;
                    esp_audio_err_t ret = esp_sbc_dec_open(&dec_cfg, sizeof(dec_cfg), &dec_temp);
                    if (ret == ESP_AUDIO_ERR_OK && dec_temp) {
                        msbc_decoder = dec_temp;
                        ESP_LOGI(TAG, "mSBC decoder initialized successfully");
                    } else {
                        ESP_LOGE(TAG, "Failed to create mSBC decoder! Error: %d", ret);
                    }
                }
                
                // NOTE: Global encoder disabled - using task-local libsbc encoder instead
                // The task-local encoder is created in mic_tx_task on Core 1
                /*
                // Initialize mSBC encoder for outgoing voice (microphone)
                ESP_LOGI(TAG, "Initializing mSBC encoder...");
                if (msbc_encoder == NULL) {
                    // Use the official mSBC default configuration macro
                    esp_sbc_enc_config_t enc_cfg = ESP_SBC_MSBC_ENC_CONFIG_DEFAULT();
                    
                    void *enc_temp = NULL;
                    esp_audio_err_t ret = esp_sbc_enc_open(&enc_cfg, sizeof(enc_cfg), &enc_temp);
                    if (ret == ESP_AUDIO_ERR_OK && enc_temp) {
                        msbc_encoder = enc_temp;
                        // Query actual frame size expected by encoder
                        int in_size = 0, out_size = 0;
                        esp_sbc_enc_get_frame_size((void*)msbc_encoder, &in_size, &out_size);
                        ESP_LOGI(TAG, "mSBC encoder initialized: in_size=%d, out_size=%d", in_size, out_size);
                        // Force safe encoding by default to avoid zero-copy crashes
                        // (zero-copy relies on HFP buffer being DMA-capable; many phones do not
                        // guarantee this). We provide a runtime toggle to enable zero-copy
                        // if testing shows it is safe for your device.
                        hfp_force_safe_encode = true;
                        ESP_LOGW(TAG, "HFP mSBC: forcing safe encode by default to prevent crashes");
                    } else {
                        ESP_LOGE(TAG, "Failed to create mSBC encoder! Error: %d", ret);
                    }
                }
                */
                
                // Register audio callback NOW (required for HCI + external codec mode)
                ESP_LOGI(TAG, "Registering HFP audio data callback...");
                esp_hf_client_register_audio_data_callback(bt_hfp_audio_cb);
                
                // IMPORTANT: Wait for HFP audio path to stabilize before enabling mic
                // Give the phone time to set up its end of the audio connection
                ESP_LOGI(TAG, "Waiting 500ms for HFP audio path to stabilize...");
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Create Mic TX queue & task (single consumer for encoding + send)
                if (mic_tx_queue == NULL) {
                    mic_tx_queue = xQueueCreate(8, sizeof(mic_tx_item_t));
                }
                if (hfp_ringbuf == NULL) {
                    // Create a ring buffer to hold pointers to esp_hf_audio_buff_t
                    // Size set to hold approx 64 encoded frames (64 * 64 bytes ~ 4KB)
                    hfp_ringbuf = xRingbufferCreate(4096, RINGBUF_TYPE_NOSPLIT);
                }
                // Always create an optional queue for debugging/alternate path
                if (hfp_send_queue == NULL) {
                    hfp_send_queue = xQueueCreate(16, sizeof(esp_hf_audio_buff_t*));
                }
                if (mic_tx_task_handle == NULL && mic_tx_queue != NULL) {
                    xTaskCreate(mic_tx_task, "mic_tx_task", 4096, NULL, 6, &mic_tx_task_handle);
                }
                if ((hfp_ringbuf && hfp_ringbuf != NULL) || (hfp_send_queue != NULL)) {
                    if (hfp_sender_task_handle == NULL) {
                        xTaskCreate(hfp_sender_task, "hfp_sender", 4096, NULL, 6, &hfp_sender_task_handle);
                    }
                }
                // Start telemetry task
                static TaskHandle_t telemetry_handle = NULL;
                if (telemetry_handle == NULL) {
                    xTaskCreate(hfp_telemetry_task, "hfp_telemetry", 2048, NULL, 2, &telemetry_handle);
                }

                // Enable microphone for call/recording (mic_data_ready_cb will queue PCM)
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
                    esp_sbc_dec_close((void*)msbc_decoder);
                    msbc_decoder = NULL;
                    ESP_LOGI(TAG, "mSBC decoder destroyed");
                }
                
                // NOTE: Global encoder cleanup disabled - using task-local encoder
                // The task-local encoder will be cleaned up when mic_tx_task exits
                /*
                // Destroy mSBC encoder
                if (msbc_encoder) {
                    esp_sbc_enc_close((void*)msbc_encoder);
                    msbc_encoder = NULL;
                    ESP_LOGI(TAG, "mSBC encoder destroyed");
                }
                */

                // Disable microphone when voice ends
                mic_manager_enable(false);
                
                // Reconfigure I2S back to 44.1kHz for A2DP music playback
                ESP_LOGI(TAG, "Reconfiguring I2S back to 44.1kHz for music...");
                
                // Stop I2S and clear FIFOs before reconfiguration
                i2s_stop(I2S_NUM_0);
                i2s_zero_dma_buffer(I2S_NUM_0);
                i2s_stop(I2S_PORT_MIC);
                i2s_zero_dma_buffer(I2S_PORT_MIC);
                
                // Restore original A2DP configuration (44.1kHz stereo)
                i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
                i2s_set_clk(I2S_PORT_MIC, 44100, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
                
                // Restart I2S
                i2s_start(I2S_NUM_0);
                i2s_start(I2S_PORT_MIC);
                
                ESP_LOGI(TAG, "I2S reconfigured: 44.1kHz stereo for music");
                
                hfp_sync_conn_hdl = 0;
                hfp_rx_packet_count = 0;  // Reset packet counter
                mic_tx_ready = false;      // Reset mic TX flag
                mic_tx_slot_available = false;  // Reset rate limiting flag

                // Destroy mic TX task & queue
                if (mic_tx_task_handle) {
                    vTaskDelete(mic_tx_task_handle);
                    mic_tx_task_handle = NULL;
                }
                if (mic_tx_queue) {
                    vQueueDelete(mic_tx_queue);
                    mic_tx_queue = NULL;
                }
                // Restore serial logs to normal so console isn't filtered permanently
                bluetooth_enable_mic_trace_only(false);
                if (hfp_ringbuf) {
                    // Drain any remaining HFP buffers
                    size_t item_size;
                    void *item = NULL;
                    while ((item = xRingbufferReceive(hfp_ringbuf, &item_size, 0)) != NULL) {
                        if (item_size == sizeof(esp_hf_audio_buff_t*)) {
                            esp_hf_audio_buff_t *buf_to_free = *((esp_hf_audio_buff_t**)item);
                            esp_hf_client_audio_buff_free(buf_to_free);
                        }
                        vRingbufferReturnItem(hfp_ringbuf, item);
                    }
                    vRingbufferDelete(hfp_ringbuf);
                    hfp_ringbuf = NULL;
                }
                if (hfp_send_queue) {
                    // Drain any queued items and free buffers
                    esp_hf_audio_buff_t *qbuf = NULL;
                    while (xQueueReceive(hfp_send_queue, &qbuf, 0) == pdTRUE) {
                        if (qbuf) esp_hf_client_audio_buff_free(qbuf);
                    }
                    vQueueDelete(hfp_send_queue);
                    hfp_send_queue = NULL;
                }
                if (hfp_sender_task_handle) {
                    vTaskDelete(hfp_sender_task_handle);
                    hfp_sender_task_handle = NULL;
                }
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
    } // end bt_hfp_cb

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
    // Enable HFP debug trace by default to help catch HFP issues during development
    bluetooth_set_hfp_debug_logs(true);
    // TEMPORARILY DISABLED: Reduce console noise by default: show only Mic/Bluetooth traces
    // bluetooth_enable_mic_trace_only(true);  // This was hiding UIManager logs!
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
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_avrc_ct_cb));  // Register AVRC callback
    ESP_LOGI(TAG, "Step 5: Callbacks registered");

    // Initialize A2DP sink
    ESP_LOGI(TAG, "Step 6: Initializing A2DP sink...");
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_LOGI(TAG, "Step 6: A2DP sink initialized");
    
    // Initialize AVRC controller (for metadata)
    ESP_LOGI(TAG, "Step 6b: Initializing AVRC controller...");
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_LOGI(TAG, "Step 6b: AVRC controller initialized - metadata ready");
    
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

// Periodic telemetry for drop counts and fallback usage
static void hfp_telemetry_task(void *pv) {
    (void)pv;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "HFP telemetry: mic_tx_drops=%u hfp_send_drops=%u fallback_count=%u", mic_tx_drops, hfp_send_drops, hfp_encode_fallback_count);
    }
}

void bluetooth_force_hfp_safe_encode(bool enable) {
    hfp_force_safe_encode = enable;
    ESP_LOGI(TAG, "HFP force-safe-encode set to %d", (int)enable);
}

void bluetooth_disable_hfp_encoder(bool disable) {
    hfp_disable_encoder = disable;
    ESP_LOGI(TAG, "HFP disable-encoder set to %d", (int)disable);
}

void bluetooth_set_hfp_debug_logs(bool enable) {
    esp_log_level_set(TAG, enable ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    ESP_LOGI(TAG, "HFP logs set to %s", enable ? "DEBUG" : "INFO");
    hfp_trace_full = enable;
}

void bluetooth_enable_mic_trace_only(bool enable) {
    if (enable) {
        // Quiet all logs except core mic path tags - set global to WARN
        esp_log_level_set("*", ESP_LOG_WARN);
        // Keep mic and bluetooth trace logs enabled
        esp_log_level_set("MicManager", ESP_LOG_INFO);
        esp_log_level_set("Bluetooth", ESP_LOG_DEBUG); // our mic traces use DEBUG

        // Silence other noisy local tags that are unrelated to mic tracing
        esp_log_level_set("SmartGlove", ESP_LOG_WARN);
        esp_log_level_set("AudioManager", ESP_LOG_WARN);
        esp_log_level_set("Display", ESP_LOG_WARN);
        esp_log_level_set("UIManager", ESP_LOG_WARN);
        esp_log_level_set("SensorManager", ESP_LOG_WARN);
        esp_log_level_set("GestureDetector", ESP_LOG_WARN);
        esp_log_level_set("AnalogSensors", ESP_LOG_WARN);
        esp_log_level_set("MPU6050", ESP_LOG_WARN);

        // Silence some BT stack tags that are usually very chatty
        esp_log_level_set("BT_HCI", ESP_LOG_ERROR);
        esp_log_level_set("BT_APPL", ESP_LOG_ERROR);
        esp_log_level_set("BT_BTC", ESP_LOG_ERROR);
        esp_log_level_set("BT_LOG", ESP_LOG_ERROR);
        esp_log_level_set("BT_BTM", ESP_LOG_ERROR);
        esp_log_level_set("BTIF", ESP_LOG_ERROR);

        // Tests / misc tags
        esp_log_level_set("I2S_SPEAKER_TEST", ESP_LOG_WARN);
        esp_log_level_set("I2S_MIC_TEST", ESP_LOG_WARN);
        esp_log_level_set("I2C_SCANNER", ESP_LOG_WARN);
        esp_log_level_set("BT_A2DP_SINK", ESP_LOG_WARN);
        ESP_LOGI(TAG, "Serial filtering enabled - only Mic/Bluetooth TRACE will be shown");
    } else {
        // Restore default verbosity for all tags
        esp_log_level_set("*", ESP_LOG_INFO);
        esp_log_level_set("MicManager", ESP_LOG_INFO);
        esp_log_level_set("Bluetooth", ESP_LOG_INFO);
        // Restore our usual defaults for local tags
        esp_log_level_set("SmartGlove", ESP_LOG_INFO);
        esp_log_level_set("AudioManager", ESP_LOG_INFO);
        esp_log_level_set("Display", ESP_LOG_INFO);
        esp_log_level_set("UIManager", ESP_LOG_INFO);
        esp_log_level_set("SensorManager", ESP_LOG_INFO);
        esp_log_level_set("GestureDetector", ESP_LOG_INFO);
        esp_log_level_set("AnalogSensors", ESP_LOG_INFO);
        esp_log_level_set("MPU6050", ESP_LOG_INFO);

        // Restore BT stack levels to defaults (INFO)
        esp_log_level_set("BT_HCI", ESP_LOG_INFO);
        esp_log_level_set("BT_APPL", ESP_LOG_INFO);
        esp_log_level_set("BT_BTC", ESP_LOG_INFO);
        esp_log_level_set("BT_LOG", ESP_LOG_INFO);
        esp_log_level_set("BT_BTM", ESP_LOG_INFO);
        esp_log_level_set("BTIF", ESP_LOG_INFO);

        esp_log_level_set("I2S_SPEAKER_TEST", ESP_LOG_INFO);
        esp_log_level_set("I2S_MIC_TEST", ESP_LOG_INFO);
        esp_log_level_set("I2C_SCANNER", ESP_LOG_INFO);
        esp_log_level_set("BT_A2DP_SINK", ESP_LOG_INFO);
        ESP_LOGI(TAG, "Serial filtering disabled - all tags restored to INFO");
    }
}

void bluetooth_use_hfp_queue(bool enable) {
    hfp_use_queue = enable;
    ESP_LOGI(TAG, "HFP send mode: using queue=%d (ringbuf=%p queue=%p)", (int)enable, hfp_ringbuf, hfp_send_queue);
}

bool bluetooth_is_connected(void) {
    return bt_connected;
}

bool bluetooth_is_playing_audio(void) {
    return audio_playing;
}

bool bluetooth_get_track_info(char *title, size_t title_size, char *artist, size_t artist_size) {
    if (!metadata_available) {
        return false;
    }
    
    if (title && title_size > 0) {
        snprintf(title, title_size, "%s", current_track_title);
    }
    if (artist && artist_size > 0) {
        snprintf(artist, artist_size, "%s", current_track_artist);
    }
    
    return true;
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
