#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "GPS";

esp_err_t gps_init(void) {
    // Initialize your GPS (e.g., MPU6050, ADXL345, etc.)
    ESP_LOGI(TAG, "GPS initialized");
    return ESP_OK;
}

esp_err_t gps_read(float *lat, float *lon, float *spd) {
    // Read raw values from gyrometer
    // Convert to rads/s
    *lat = 4.0f; // Replace with actual reading
    *lon = 4.1f;
    *spd = 4.2f; // 0 when still
    return ESP_OK;
}