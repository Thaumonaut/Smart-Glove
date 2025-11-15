/**
 * Detailed I2C Scanner Test
 * Scans all I2C addresses and reports what's found
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "../src/config.h"

static const char *TAG = "I2C_SCANNER";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,  // 100kHz - slow and reliable
        },
    };
    
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C initialized - SDA: GPIO%d, SCL: GPIO%d, Speed: 100kHz", 
             I2C_SDA_IO, I2C_SCL_IO);
    return ESP_OK;
}

static esp_err_t i2c_probe_address(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static const char* get_device_name(uint8_t addr)
{
    switch(addr) {
        case 0x68: return "MPU6050 (AD0=LOW) or MPU9250";
        case 0x69: return "MPU6050 (AD0=HIGH) or MPU9250";
        case 0x57: return "MAX30102 (Heart Rate Sensor)";
        case 0x39: return "APDS9960 (Gesture Sensor)";
        case 0x76: return "BMP280 or BME280";
        case 0x77: return "BMP280 or BME280 (Alt)";
        case 0x3C: return "SSD1306 OLED Display";
        case 0x3D: return "SSD1306 OLED Display (Alt)";
        default: return "Unknown Device";
    }
}

static void scan_i2c_bus(void)
{
    ESP_LOGI(TAG, "\n");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "    I2C Bus Scanner - Detailed");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "Scanning addresses 0x03 to 0x77...\n");
    
    int devices_found = 0;
    
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        esp_err_t ret = i2c_probe_address(addr);
        
        if (ret == ESP_OK) {
            devices_found++;
            ESP_LOGI(TAG, "✓ Device found at 0x%02X (%s)", addr, get_device_name(addr));
            
            // Try to read a byte to confirm communication
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
            uint8_t data;
            i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            esp_err_t read_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            
            if (read_ret == ESP_OK) {
                ESP_LOGI(TAG, "  → Read test successful (got 0x%02X)", data);
            } else {
                ESP_LOGW(TAG, "  → Read test failed: %s", esp_err_to_name(read_ret));
            }
            
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Only log timeouts for MPU6050 addresses
            if (addr == 0x68 || addr == 0x69) {
                ESP_LOGW(TAG, "✗ Timeout at 0x%02X (%s)", addr, get_device_name(addr));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between probes
    }
    
    ESP_LOGI(TAG, "\n====================================");
    ESP_LOGI(TAG, "Scan complete. Devices found: %d", devices_found);
    ESP_LOGI(TAG, "====================================\n");
}

static void test_mpu6050_who_am_i(uint8_t addr)
{
    ESP_LOGI(TAG, "Testing MPU6050 WHO_AM_I at address 0x%02X...", addr);
    
    // Try to read WHO_AM_I register (0x75)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x75, true);  // WHO_AM_I register
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    uint8_t who_am_i = 0;
    i2c_master_read_byte(cmd, &who_am_i, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ WHO_AM_I = 0x%02X (Expected: 0x68)", who_am_i);
        if (who_am_i == 0x68) {
            ESP_LOGI(TAG, "  → MPU6050 confirmed!");
        } else {
            ESP_LOGW(TAG, "  → Unexpected WHO_AM_I value");
        }
    } else {
        ESP_LOGE(TAG, "✗ Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Scanner Test...");
    ESP_LOGI(TAG, "GPIO Configuration:");
    ESP_LOGI(TAG, "  SDA: GPIO%d", I2C_SDA_IO);
    ESP_LOGI(TAG, "  SCL: GPIO%d", I2C_SCL_IO);
    
    // Initialize I2C
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C!");
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Scan the bus multiple times
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "\n>>> Scan #%d <<<", i + 1);
        scan_i2c_bus();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Test both possible MPU6050 addresses
    ESP_LOGI(TAG, "\n>>> MPU6050 WHO_AM_I Tests <<<\n");
    test_mpu6050_who_am_i(0x68);
    vTaskDelay(pdMS_TO_TICKS(100));
    test_mpu6050_who_am_i(0x69);
    
    ESP_LOGI(TAG, "\nTest complete. Looping forever...");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
