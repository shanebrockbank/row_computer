#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"
#include "sensors_common.h"

static const char *TAG = "ACCEL";

esp_err_t accel_init(void) {
    // Initialize accelerometer 
    esp_err_t err = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00); // Wake up sensor
    if (err == ESP_OK){
        ESP_LOGI(TAG, "Accelerometer initialized");
    }else{
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

esp_err_t accel_read(float *x, float *y, float *z) {
    uint8_t data[6];
    esp_err_t err = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Failed to read MPU6050: %s", esp_err_to_name(err));
        return err;
    } 
    // Read raw values from accelerometer
    int16_t raw_x = combine_bytes(data[0], data[1]);
    int16_t raw_y = combine_bytes(data[2], data[3]);
    int16_t raw_z = combine_bytes(data[4], data[5]);

    // Convert raw values to g's
    *x = (float)raw_x / LSB_SENSITIVITY_2g;
    *y = (float)raw_y / LSB_SENSITIVITY_2g;
    *z = (float)raw_z / LSB_SENSITIVITY_2g;

    return ESP_OK;
}