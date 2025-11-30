/**
 * MAX30102 Heart Rate / SpO2 Sensor Driver Implementation
 */

#include "max30102.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "MAX30102";

// External I2C mutex (defined in main.cpp)
extern SemaphoreHandle_t i2c_mutex;

// Registers
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D
#define REG_PILOT_PA      0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR     0x1F
#define REG_TEMP_FRAC     0x20
#define REG_TEMP_CONFIG   0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID        0xFE
#define REG_PART_ID       0xFF

/**
 * Write a single byte to MAX30102 register
 */
static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
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
 * Read a single byte from MAX30102 register
 */
static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
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
 * Read multiple bytes from MAX30102 register
 */
static esp_err_t max30102_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t max30102_init(void) {
    esp_err_t ret;
    uint8_t part_id;
    
    ESP_LOGI(TAG, "Initializing MAX30102...");
    
    // Check Part ID
    ret = max30102_read_reg(REG_PART_ID, &part_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Part ID: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (part_id != 0x15) { // MAX30102 Part ID is 0x15
        ESP_LOGE(TAG, "Wrong Part ID: 0x%02X (expected 0x15)", part_id);
        // return ESP_FAIL; // Proceed anyway for now in case of compatible variants
    }
    ESP_LOGI(TAG, "Found MAX30102 (Part ID: 0x%02X)", part_id);
    
    // Reset
    ret = max30102_write_reg(REG_MODE_CONFIG, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // FIFO Configuration
    // SMP_AVE=010 (4 samples averaged), FIFO_ROLLOVER_EN=1, FIFO_A_FULL=1111 (15 empty samples)
    ret = max30102_write_reg(REG_FIFO_CONFIG, 0x4F);
    if (ret != ESP_OK) return ret;
    
    // Mode Configuration
    // Mode = 011 (SpO2 mode, Red and IR LEDs)
    ret = max30102_write_reg(REG_MODE_CONFIG, 0x03);
    if (ret != ESP_OK) return ret;
    
    // SpO2 Configuration
    // SPO2_ADC_RGE=01 (4096nA), SPO2_SR=001 (100 samples/sec), LED_PW=11 (411us, 18 bits)
    ret = max30102_write_reg(REG_SPO2_CONFIG, 0x27);
    if (ret != ESP_OK) return ret;
    
    // LED Pulse Amplitude
    // 0x3F = ~12.5mA (increased for visibility)
    ret = max30102_write_reg(REG_LED1_PA, 0x3F); // Red
    if (ret != ESP_OK) return ret;
    ret = max30102_write_reg(REG_LED2_PA, 0x3F); // IR
    if (ret != ESP_OK) return ret;
    
    // Clear FIFO pointers
    max30102_write_reg(REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(REG_OVF_COUNTER, 0x00);
    max30102_write_reg(REG_FIFO_RD_PTR, 0x00);
    
    ESP_LOGI(TAG, "MAX30102 initialized successfully");
    return ESP_OK;
}

bool max30102_read(uint32_t *red_led, uint32_t *ir_led) {
    uint8_t wr_ptr, rd_ptr;
    esp_err_t ret;
    
    // Read pointers
    ret = max30102_read_reg(REG_FIFO_WR_PTR, &wr_ptr);
    if (ret != ESP_OK) return false;
    
    ret = max30102_read_reg(REG_FIFO_RD_PTR, &rd_ptr);
    if (ret != ESP_OK) return false;
    
    // Check if there is data
    if (wr_ptr == rd_ptr) {
        return false; // FIFO empty
    }
    
    // Read one sample (6 bytes: 3 bytes Red, 3 bytes IR)
    uint8_t data[6];
    ret = max30102_read_regs(REG_FIFO_DATA, data, 6);
    if (ret != ESP_OK) return false;
    
    // Parse data (18-bit data, left justified in 3 bytes)
    *red_led = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2]) & 0x03FFFF;
    *ir_led = ((uint32_t)data[3] << 16 | (uint32_t)data[4] << 8 | (uint32_t)data[5]) & 0x03FFFF;
    
    return true;
}

esp_err_t max30102_shutdown(void) {
    uint8_t mode;
    esp_err_t ret = max30102_read_reg(REG_MODE_CONFIG, &mode);
    if (ret != ESP_OK) return ret;
    
    return max30102_write_reg(REG_MODE_CONFIG, mode | 0x80); // Set SHDN bit
}

esp_err_t max30102_wakeup(void) {
    uint8_t mode;
    esp_err_t ret = max30102_read_reg(REG_MODE_CONFIG, &mode);
    if (ret != ESP_OK) return ret;
    
    return max30102_write_reg(REG_MODE_CONFIG, mode & ~0x80); // Clear SHDN bit
}
