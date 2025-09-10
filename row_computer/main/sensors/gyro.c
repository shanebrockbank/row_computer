#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG_ACCEL = "GYRO";

esp_err_t gyro_init(void) {
    // Initialize your gyrometer (e.g., MPU6050, ADXL345, etc.)
    ESP_LOGI(TAG_ACCEL, "Gyrometer initialized");
    return ESP_OK;
}

esp_err_t gyro_read(float *x, float *y, float *z) {
    // Read raw values from gyrometer
    // Convert to rads/s
    *x = 2.0f; // Replace with actual reading
    *y = 2.1f;
    *z = 2.2f; // 0 when still
    return ESP_OK;
}