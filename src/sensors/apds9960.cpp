/**
 * APDS9960 Gesture/Proximity Sensor Driver Implementation
 */

#include "apds9960.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "APDS9960";

// External I2C mutex (defined in main.cpp)
extern SemaphoreHandle_t i2c_mutex;

// APDS9960 Registers
#define APDS9960_ENABLE     0x80
#define APDS9960_ATIME      0x81
#define APDS9960_WTIME      0x83
#define APDS9960_PPULSE     0x8E
#define APDS9960_CONTROL    0x8F
#define APDS9960_CONFIG2    0x90
#define APDS9960_ID         0x92
#define APDS9960_STATUS     0x93
#define APDS9960_PDATA      0x9C
#define APDS9960_GCONF1     0xA2
#define APDS9960_GCONF2     0xA3
#define APDS9960_GOFFSET_U  0xA4
#define APDS9960_GPULSE     0xA6
#define APDS9960_GCONF3     0xAA
#define APDS9960_GCONF4     0xAB
#define APDS9960_GFLVL      0xAE
#define APDS9960_GSTATUS    0xAF
#define APDS9960_GFIFO_U    0xFC

// Enable register bits
#define APDS9960_PON        0x01  // Power ON
#define APDS9960_AEN        0x02  // ALS enable
#define APDS9960_PEN        0x04  // Proximity enable
#define APDS9960_GEN        0x40  // Gesture enable

/**
 * Write a single byte to APDS9960 register
 */
static esp_err_t apds9960_write_reg(uint8_t reg, uint8_t value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

/**
 * Read a single byte from APDS9960 register
 */
static esp_err_t apds9960_read_reg(uint8_t reg, uint8_t *value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        if (i2c_mutex) {
            xSemaphoreGive(i2c_mutex);
        }
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

/**
 * Read multiple bytes from APDS9960
 */
static esp_err_t apds9960_read_block(uint8_t reg, uint8_t *data, size_t len) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        if (i2c_mutex) {
            xSemaphoreGive(i2c_mutex);
        }
        return ret;
    }
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (APDS9960_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t apds9960_init(void) {
    esp_err_t ret;
    uint8_t id;
    
    ESP_LOGI(TAG, "Initializing APDS9960...");
    
    // Check device ID
    ret = apds9960_read_reg(APDS9960_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Accept multiple compatible IDs: 0xAB (APDS9960), 0x9C (APDS9930), 0x9E (variant/clone)
    if (id != 0xAB && id != 0x9C && id != 0x9E) {
        ESP_LOGE(TAG, "Wrong ID: 0x%02X (expected 0xAB, 0x9C, or 0x9E)", id);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Found device with ID: 0x%02X", id);
    ESP_LOGI(TAG, "Found APDS9960 (ID: 0x%02X)", id);
    
    // Disable everything during configuration
    ret = apds9960_write_reg(APDS9960_ENABLE, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set ADC integration time (affects all ALS/color)
    ret = apds9960_write_reg(APDS9960_ATIME, 0xDB);  // ~100ms
    if (ret != ESP_OK) return ret;
    
    // Set wait time
    ret = apds9960_write_reg(APDS9960_WTIME, 0xF6);  // 27ms
    if (ret != ESP_OK) return ret;
    
    // Set proximity pulse count and length
    ret = apds9960_write_reg(APDS9960_PPULSE, 0x87);  // 8 pulses, 8us each
    if (ret != ESP_OK) return ret;
    
    // Set LED drive strength and proximity gain
    ret = apds9960_write_reg(APDS9960_CONTROL, 0x05);  // 100mA LED, 4x gain
    if (ret != ESP_OK) return ret;
    
    // Configure gesture sensor for better consistency
    ret = apds9960_write_reg(APDS9960_GCONF1, 0x40);  // 4 gesture events for FIFO entry
    if (ret != ESP_OK) return ret;
    
    ret = apds9960_write_reg(APDS9960_GCONF2, 0x65);  // Gain 4x, LED 100mA, wait 1.4ms (faster response)
    if (ret != ESP_OK) return ret;
    
    ret = apds9960_write_reg(APDS9960_GPULSE, 0x89);  // 10 pulses, 16us each (more sensitive)
    if (ret != ESP_OK) return ret;
    
    ret = apds9960_write_reg(APDS9960_GCONF3, 0x00);  // All photodiodes active
    if (ret != ESP_OK) return ret;
    
    // Set gesture entry/exit thresholds to filter noise
    ret = apds9960_write_reg(APDS9960_GCONF4, 0x03);  // GIEN=0 (no interrupt), GMODE=1 (gesture mode)
    if (ret != ESP_OK) return ret;
    
    // Enable power, proximity, and gesture
    ret = apds9960_write_reg(APDS9960_ENABLE, 
                             APDS9960_PON | APDS9960_PEN | APDS9960_GEN);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Test proximity reading immediately after init
    uint8_t test_prox = 0;
    if (apds9960_read_reg(APDS9960_PDATA, &test_prox) == ESP_OK) {
        ESP_LOGI(TAG, "APDS9960 initialized - Initial proximity: %d", test_prox);
    }
    
    ESP_LOGI(TAG, "APDS9960 initialized successfully");
    return ESP_OK;
}

bool apds9960_read_proximity(uint8_t *proximity) {
    return (apds9960_read_reg(APDS9960_PDATA, proximity) == ESP_OK);
}

bool apds9960_gesture_available(void) {
    uint8_t status;
    if (apds9960_read_reg(APDS9960_GSTATUS, &status) != ESP_OK) {
        return false;
    }
    return (status & 0x01) != 0;  // GVALID bit
}

apds9960_gesture_t apds9960_read_gesture(void) {
    uint8_t gstatus;
    
    // Check if gesture data is available
    if (apds9960_read_reg(APDS9960_GSTATUS, &gstatus) != ESP_OK) {
        return APDS_DIR_NONE;
    }
    
    if ((gstatus & 0x01) == 0) {  // GVALID bit not set
        return APDS_DIR_NONE;
    }
    
    // Read FIFO level
    uint8_t fifo_level;
    if (apds9960_read_reg(APDS9960_GFLVL, &fifo_level) != ESP_OK) {
        return APDS_DIR_NONE;
    }
    
    if (fifo_level == 0) {
        return APDS_DIR_NONE;
    }
    
    // Read gesture data (4 bytes per dataset: UP, DOWN, LEFT, RIGHT)
    uint8_t fifo_data[128];
    size_t bytes_to_read = fifo_level * 4;
    if (bytes_to_read > sizeof(fifo_data)) {
        bytes_to_read = sizeof(fifo_data);
    }
    
    if (apds9960_read_block(APDS9960_GFIFO_U, fifo_data, bytes_to_read) != ESP_OK) {
        return APDS_DIR_NONE;
    }
    
    // Filter out low-signal samples (zeros) at beginning and end
    // Find first valid sample (sum > threshold)
    int first_valid = -1;
    int last_valid = -1;
    const int MIN_SIGNAL = 10;  // Minimum sum to consider valid
    
    for (int i = 0; i < fifo_level; i++) {
        int idx = i * 4;
        int sum = fifo_data[idx] + fifo_data[idx+1] + fifo_data[idx+2] + fifo_data[idx+3];
        if (sum > MIN_SIGNAL) {
            if (first_valid == -1) first_valid = i;
            last_valid = i;
        }
    }
    
    // Need at least 3 valid samples for reliable detection
    if (first_valid == -1 || last_valid == -1 || (last_valid - first_valid) < 2) {
        return APDS_DIR_NONE;
    }
    
    // Use filtered first and last samples
    int first_idx = first_valid * 4;
    int last_idx = last_valid * 4;
    
    int first_up = fifo_data[first_idx];
    int first_down = fifo_data[first_idx + 1];
    int first_left = fifo_data[first_idx + 2];
    int first_right = fifo_data[first_idx + 3];
    
    int last_up = fifo_data[last_idx];
    int last_down = fifo_data[last_idx + 1];
    int last_left = fifo_data[last_idx + 2];
    int last_right = fifo_data[last_idx + 3];
    
    // Calculate deltas
    int ud_delta = (last_up - first_up) - (last_down - first_down);
    int lr_delta = (last_left - first_left) - (last_right - first_right);
    
    int ud_ratio = abs(ud_delta) * 100 / (abs(ud_delta) + abs(lr_delta) + 1);
    
    // Determine gesture with higher threshold for better consistency
    const int THRESHOLD = 20;
    
    if (abs(ud_delta) > THRESHOLD || abs(lr_delta) > THRESHOLD) {
        if (ud_ratio > 60) {
            return (ud_delta > 0) ? APDS_DIR_UP : APDS_DIR_DOWN;
        } else {
            return (lr_delta > 0) ? APDS_DIR_LEFT : APDS_DIR_RIGHT;
        }
    }
    
    return APDS_DIR_NONE;
}

esp_err_t apds9960_enable_gesture(bool enable) {
    uint8_t val;
    esp_err_t ret = apds9960_read_reg(APDS9960_ENABLE, &val);
    if (ret != ESP_OK) return ret;
    
    if (enable) {
        val |= APDS9960_GEN;
    } else {
        val &= ~APDS9960_GEN;
    }
    
    return apds9960_write_reg(APDS9960_ENABLE, val);
}
