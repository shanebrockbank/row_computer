#include "esp_log.h"
#include "driver/i2c.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3
#define MPU6050_ACCEL_CONFIG_REG 0x1C

static const char *TAG_ACCEL = "ACCEL";

esp_err_t accel_init(void) {
    // Initialize your accelerometer (e.g., MPU6050, ADXL345, etc.)
    ESP_LOGI(TAG_ACCEL, "Accelerometer initialized");
    return ESP_OK;
}

esp_err_t accel_read(float *x, float *y, float *z) {
    // Read raw values from accelerometer
    // Convert to m/s²
    *x = 1.0f; // Replace with actual reading
    *y = 1.1f;
    *z = 1.2f; // Should read ~9.8 when still
    return ESP_OK;
}