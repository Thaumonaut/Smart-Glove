#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "I2C_SCANNER";

static bool i2c_probe_address(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    return (result == ESP_OK);
}

static const char* get_device_name(uint8_t address) {
    switch (address) {
        case 0x39: return "APDS9960 (Gesture/Proximity)";
        case 0x57: return "MAX30102 (Heart Rate/O2)";
        case 0x68: return "MPU6050 (Gyro/Accel)";
        case 0x69: return "MPU6050 (Alt address)";
        default: return "Unknown device";
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "I2C Bus Scanner (Quick App)");
    ESP_LOGI(TAG, "===========================================");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_FREQ_HZ},
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "I2C: SDA=%d SCL=%d Freq=%d", I2C_SDA_IO, I2C_SCL_IO, I2C_FREQ_HZ);

    while (1) {
        printf("\nScan (0x03..0x77)\n");
        printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
        printf("00:         ");
        int found = 0;
        for (uint8_t address = 0x03; address < 0x78; address++) {
            if ((address & 0x0F) == 0) printf("\n%02X: ", address);
            if (i2c_probe_address(address)) {
                printf("%02X ", address); found++;
            } else {
                printf("-- ");
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        printf("\n\n");
        ESP_LOGI(TAG, "Found %d device(s)", found);
        if (found) {
            for (uint8_t address = 0x03; address < 0x78; address++) {
                if (i2c_probe_address(address)) {
                    ESP_LOGI(TAG, "  0x%02X: %s", address, get_device_name(address));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
