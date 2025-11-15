/**
 * I2C Scanner Test
 *
 * Scans the I2C bus for connected devices
 * Useful for:
 * - Verifying I2C wiring is correct
 * - Finding device addresses
 * - Detecting address conflicts
 *
 * Expected devices on Smart Glove:
 * - 0x39: APDS9960 (gesture/proximity)
 * - 0x57: MAX30102 (heart rate/O2)
 * - 0x68: MPU6050 (gyro/accelerometer on HW-123)
 *
 * Hardware connections:
 * - All I2C devices share SDA (GPIO 21) and SCL (GPIO 22)
 * - Pull-up resistors should be on the bus (usually built into modules)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "../src/config.h"

static const char *TAG = "I2C_SCANNER";

/**
 * Probe I2C address to see if device responds
 */
static bool i2c_probe_address(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return (result == ESP_OK);
}

/**
 * Get device name for known addresses
 */
static const char* get_device_name(uint8_t address) {
    switch (address) {
        case 0x39: return "APDS9960 (Gesture/Proximity)";
        case 0x57: return "MAX30102 (Heart Rate/O2)";
        case 0x68: return "MPU6050 (Gyro/Accel on HW-123)";
        case 0x69: return "MPU6050 (Alt address)";
        default: return "Unknown device";
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "I2C Bus Scanner");
    ESP_LOGI(TAG, "Smart Glove I2C Device Detection");
    ESP_LOGI(TAG, "===========================================");

    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ_HZ,
        },
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "I2C initialized: SDA=GPIO%d, SCL=GPIO%d, Freq=%dHz",
             I2C_SDA_IO, I2C_SCL_IO, I2C_FREQ_HZ);
    ESP_LOGI(TAG, "Scanning I2C bus...\n");

    // Scan all possible I2C addresses (0x03 to 0x77)
    int devices_found = 0;
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    printf("00:         ");

    for (uint8_t address = 0x03; address < 0x78; address++) {
        // Print row header
        if (address % 16 == 0) {
            printf("\n%02X: ", address);
        }

        // Probe address
        if (i2c_probe_address(address)) {
            printf("%02X ", address);
            devices_found++;
        } else {
            printf("-- ");
        }

        // Small delay between probes
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("\n\n");
    ESP_LOGI(TAG, "Scan complete. Found %d device(s)", devices_found);

    // List found devices with names
    if (devices_found > 0) {
        ESP_LOGI(TAG, "\nDetected devices:");
        for (uint8_t address = 0x03; address < 0x78; address++) {
            if (i2c_probe_address(address)) {
                ESP_LOGI(TAG, "  0x%02X: %s", address, get_device_name(address));
            }
        }
    } else {
        ESP_LOGW(TAG, "\nNo I2C devices found!");
        ESP_LOGW(TAG, "Check wiring and pull-up resistors");
    }

    ESP_LOGI(TAG, "\nExpected devices for Smart Glove:");
    ESP_LOGI(TAG, "  0x39: APDS9960 (when arrives from Amazon)");
    ESP_LOGI(TAG, "  0x57: MAX30102 (when arrives from AliExpress)");
    ESP_LOGI(TAG, "  0x68: MPU6050 on HW-123 module (should be present now)");

    // Keep scanning periodically to detect hot-plugged devices
    ESP_LOGI(TAG, "\nContinuous scanning mode (every 5 seconds)...");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        devices_found = 0;
        for (uint8_t address = 0x03; address < 0x78; address++) {
            if (i2c_probe_address(address)) {
                devices_found++;
            }
        }
        ESP_LOGI(TAG, "Devices on bus: %d", devices_found);
    }
}
