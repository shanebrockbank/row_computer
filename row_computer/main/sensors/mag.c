#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "MAG";

esp_err_t mag_init(void) {
    // Initialize your magnetometer
    ESP_LOGI(TAG, "Magnetometer initialized");
    return ESP_OK;
}

esp_err_t mag_read(float *x, float *y, float *z) {
    // Read raw values from magnetometer
    // Convert to Tesla
    *x = 3.0f; // Replace with actual reading
    *y = 3.1f;
    *z = 3.2f; // 0 when still
    return ESP_OK;
}