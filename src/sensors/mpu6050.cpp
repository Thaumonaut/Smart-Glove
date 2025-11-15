/**
 * MPU6050 Gyroscope/Accelerometer Driver Implementation
 */

#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../config.h"

static const char *TAG = "MPU6050";

// External I2C mutex (defined in main.cpp)
extern SemaphoreHandle_t i2c_mutex;

/**
 * Write a single byte to MPU6050 register
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
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
 * Read a single byte from MPU6050 register
 */
static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *value) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
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
    
    vTaskDelay(pdMS_TO_TICKS(2));
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
    return ret;
}

esp_err_t mpu6050_init(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing MPU6050...");
    
    // Try to detect MPU6050 at address 0x68 or 0x69
    uint8_t who_am_i = 0;
    bool found = false;
    
    // Try primary address 0x68
    ret = mpu6050_read_reg(0x75, &who_am_i);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device found at 0x68, WHO_AM_I: 0x%02X", who_am_i);
        // Accept various WHO_AM_I values (MPU6050=0x68, variants=0x70/0x72, MPU9250=0x71)
        if (who_am_i == 0x68 || who_am_i == 0x70 || who_am_i == 0x71 || who_am_i == 0x72 || who_am_i == 0x73) {
            found = true;
            ESP_LOGI(TAG, "Recognized MPU6xxx variant");
        } else {
            ESP_LOGW(TAG, "Unusual WHO_AM_I, but will attempt initialization anyway");
            found = true; // Try to initialize anyway
        }
    } else {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Reset device
    ESP_LOGI(TAG, "Resetting MPU6050...");
    ret = mpu6050_write_reg(0x6B, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 reset failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(150)); // Wait for reset to complete
    
    // Wake up MPU6050 (PWR_MGMT_1: disable sleep, use internal 8MHz oscillator)
    ESP_LOGI(TAG, "Waking up MPU6050...");
    ret = mpu6050_write_reg(0x6B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake-up failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Set sample rate divider (SMPLRT_DIV): 1kHz / (1 + 9) = 100Hz
    ret = mpu6050_write_reg(0x19, 0x09);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 sample rate config failed");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure DLPF (Digital Low Pass Filter) and gyro range
    // CONFIG register: DLPF_CFG = 6 (5Hz bandwidth, reduces noise)
    ret = mpu6050_write_reg(0x1A, 0x06);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 DLPF config failed");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure gyro range (GYRO_CONFIG): ±250°/s (most sensitive)
    ret = mpu6050_write_reg(0x1B, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 gyro range config failed");
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure accelerometer range (ACCEL_CONFIG): ±2g (most sensitive)
    ret = mpu6050_write_reg(0x1C, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 accel range config failed");
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Verify configuration by reading back
    uint8_t config_val = 0;
    if (mpu6050_read_reg(0x6B, &config_val) == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 PWR_MGMT_1: 0x%02X (should be 0x00)", config_val);
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

bool mpu6050_read(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z,
                  int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGD(TAG, "Read failed: mutex timeout");
        return false;
    }
    
    uint8_t data[14];
    
    // Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // ACCEL_XOUT_H register
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Write failed: %s", esp_err_to_name(ret));
        if (i2c_mutex) {
            xSemaphoreGive(i2c_mutex);
        }
        return false;
    }
    
    // Small delay between write and read
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // Read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }

    if (ret == ESP_OK) {
        *accel_x = (int16_t)((data[0] << 8) | data[1]);
        *accel_y = (int16_t)((data[2] << 8) | data[3]);
        *accel_z = (int16_t)((data[4] << 8) | data[5]);
        *gyro_x = (int16_t)((data[8] << 8) | data[9]);
        *gyro_y = (int16_t)((data[10] << 8) | data[11]);
        *gyro_z = (int16_t)((data[12] << 8) | data[13]);
        
        // Log raw data occasionally for debugging
        static int read_count = 0;
        if (read_count++ % 100 == 0) {
            ESP_LOGI(TAG, "Raw data: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d",
                     *accel_x, *accel_y, *accel_z, *gyro_x, *gyro_y, *gyro_z);
        }
        return true;
    } else {
        ESP_LOGD(TAG, "Read failed: %s", esp_err_to_name(ret));
    }
    return false;
}
