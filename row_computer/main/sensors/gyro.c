#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"
#include "sensors_common.h"


static const char *TAG = "GYRO";

esp_err_t gyro_init(void) {
    // Initialize your gyrometer (e.g., MPU6050, ADXL345, etc.)
    esp_err_t err = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00); // Wake up sensor
    if (err == ESP_OK){
        ESP_LOGI(TAG, "Gyrometer initialized");
    }else{
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

esp_err_t gyro_read(float *x, float *y, float *z) {
    uint8_t data[6];
    esp_err_t err = mpu6050_read_bytes(MPU6050_GYRO_XOUT_H, data, 6);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Failed to read MPU6050: %s", esp_err_to_name(err));
        return err;
    } 
    // Read raw values from accelerometer
    int16_t raw_x = combine_bytes(data[0], data[1]);
    int16_t raw_y = combine_bytes(data[2], data[3]);
    int16_t raw_z = combine_bytes(data[4], data[5]);

    // Convert raw values to g's
    *x = (float)raw_x / LSB_SENSITIVITY_250_deg;
    *y = (float)raw_y / LSB_SENSITIVITY_250_deg;
    *z = (float)raw_z / LSB_SENSITIVITY_250_deg;

    return ESP_OK;
}