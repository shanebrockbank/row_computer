#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"

static const char *TAG = "MPU6050";
static bool mpu6050_initialized = false;

esp_err_t mpu6050_init(void) {
    if (mpu6050_initialized) {
        ESP_LOGD(TAG, "MPU6050 already initialized");
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Initializing MPU6050 (accelerometer + gyroscope)");

    // Wake up the sensor
    esp_err_t err = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    // Small delay for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZE_DELAY_MS));

    // Verify communication by reading WHO_AM_I register
    uint8_t who_am_i;
    err = mpu6050_read_bytes(0x75, &who_am_i, 1); // WHO_AM_I register
    if (err == ESP_OK && who_am_i == 0x68) {
        ESP_LOGD(TAG, "MPU6050 initialized successfully (WHO_AM_I: 0x%02X)", who_am_i);
        mpu6050_initialized = true;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "MPU6050 communication failed or wrong chip ID: 0x%02X", who_am_i);
        return ESP_FAIL;
    }
}

esp_err_t mpu6050_read_all(mpu6050_data_t *data) {
    if (!mpu6050_initialized) {
        ESP_LOGE(TAG, "MPU6050 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL) {
        ESP_LOGE(TAG, "Null data pointer");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[14]; // 6 bytes accel + 2 bytes temp + 6 bytes gyro
    esp_err_t err = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(err));
        return err;
    }

    // Parse accelerometer data (first 6 bytes)
    int16_t raw_accel_x = combine_bytes(raw_data[0], raw_data[1]);
    int16_t raw_accel_y = combine_bytes(raw_data[2], raw_data[3]);
    int16_t raw_accel_z = combine_bytes(raw_data[4], raw_data[5]);

    // Parse gyroscope data (bytes 8-13, skip temperature bytes 6-7)
    int16_t raw_gyro_x = combine_bytes(raw_data[8], raw_data[9]);
    int16_t raw_gyro_y = combine_bytes(raw_data[10], raw_data[11]);
    int16_t raw_gyro_z = combine_bytes(raw_data[12], raw_data[13]);

    // Convert to physical units
    data->accel_x = (float)raw_accel_x / LSB_SENSITIVITY_2g;
    data->accel_y = (float)raw_accel_y / LSB_SENSITIVITY_2g;
    data->accel_z = (float)raw_accel_z / LSB_SENSITIVITY_2g;

    data->gyro_x = (float)raw_gyro_x / LSB_SENSITIVITY_250_deg;
    data->gyro_y = (float)raw_gyro_y / LSB_SENSITIVITY_250_deg;
    data->gyro_z = (float)raw_gyro_z / LSB_SENSITIVITY_250_deg;

    return ESP_OK;
}

esp_err_t mpu6050_read_accel(float *x, float *y, float *z) {
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_all(&data);
    if (err == ESP_OK) {
        *x = data.accel_x;
        *y = data.accel_y;
        *z = data.accel_z;
    }
    return err;
}

esp_err_t mpu6050_read_gyro(float *x, float *y, float *z) {
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_all(&data);
    if (err == ESP_OK) {
        *x = data.gyro_x;
        *y = data.gyro_y;
        *z = data.gyro_z;
    }
    return err;
}