// main/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag.h"
#include "sensors/gps.h"
#include "utils/protocol_init.h"

static const char *TAG = "MAIN";

    // Start with bare minimum hardware validation
    void test_i2c_bus(void) {
        // Scan I2C bus to find your devices
        for (uint8_t addr = 1; addr < 127; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret == ESP_OK) {
                ESP_LOGI("I2C_SCAN", "Found device at address 0x%02X", addr);
            }
        }
    }

void app_main(void)
{
    ESP_LOGI(TAG, "Rowing computer starting...");
    // Initialize I2C for IMU/mag
    i2c_master_init();
    test_i2c_bus();

    // Initialize SPI for GPS (if needed)

    accel_init();
    float ax, ay, az;
    gyro_init();
    float gx, gy, gz;
    mag_init();
    float mx, my, mz;
    gps_init();
    float lat, lon, spd;

    //test_i2c_bus();

    while (1) {
        // Read and print sensor data
        if (accel_read(&ax, &ay, &az) == ESP_OK) {
            //ESP_LOGI(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f", ax, ay, az);
        }

        if (gyro_read(&gx, &gy, &gz) == ESP_OK) {
            //ESP_LOGI(TAG, "Gyro:  X=%.2f Y=%.2f Z=%.2f", gx, gy, gz);
        }

        if (mag_read(&mx, &my, &mz) == ESP_OK) {
            ESP_LOGI(TAG, "Mag: X=%.2f Y=%.2f Z=%.2f", mx, my, mz);
        }

        // if (gps_read(&lat, &lon, &spd) == ESP_OK) {
        //     ESP_LOGI(TAG, "GPS: X=%.2f Y=%.2f Z=%.2f", lat, lon, spd);
        // }
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}