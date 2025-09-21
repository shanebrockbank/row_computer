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
        ESP_LOGI(TAG, "MPU6050 already initialized - skipping");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "=== INITIALIZING MPU6050 ===");
    ESP_LOGI(TAG, "Step 1: Waking up MPU6050 sensor...");

    // Wake up the sensor
    esp_err_t err = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ FAILED to wake up MPU6050: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "❌ MPU6050 INITIALIZATION FAILED");
        return err;
    }
    ESP_LOGI(TAG, "✅ MPU6050 wake-up successful");

    ESP_LOGI(TAG, "Step 2: Sensor stabilization delay (%dms)...", SENSOR_STABILIZE_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZE_DELAY_MS));

    ESP_LOGI(TAG, "Step 3: Configuring accelerometer range (±2g)...");
    err = mpu6050_write_byte(MPU6050_ACCEL_CONFIG_REG, 0x00); // ±2g range
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ FAILED to configure accelerometer range: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "❌ MPU6050 INITIALIZATION FAILED");
        return err;
    }
    ESP_LOGI(TAG, "✅ Accelerometer configured for ±2g range");

    ESP_LOGI(TAG, "Step 4: Configuring gyroscope range (±250°/s)...");
    err = mpu6050_write_byte(0x1B, 0x00); // GYRO_CONFIG register, ±250°/s range
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ FAILED to configure gyroscope range: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "❌ MPU6050 INITIALIZATION FAILED");
        return err;
    }
    ESP_LOGI(TAG, "✅ Gyroscope configured for ±250°/s range");

    ESP_LOGI(TAG, "Step 5: Verifying communication (WHO_AM_I register)...");
    uint8_t who_am_i;
    err = mpu6050_read_bytes(0x75, &who_am_i, 1); // WHO_AM_I register
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ FAILED to read WHO_AM_I register: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "❌ MPU6050 INITIALIZATION FAILED - I2C communication error");
        return err;
    }

    ESP_LOGI(TAG, "WHO_AM_I register read: 0x%02X (expected: 0x68)", who_am_i);
    if (who_am_i == 0x68) {
        ESP_LOGI(TAG, "✅ MPU6050 chip ID verified successfully");
        mpu6050_initialized = true;
        ESP_LOGI(TAG, "🎉 MPU6050 INITIALIZATION COMPLETE");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "❌ Wrong chip ID - Expected: 0x68, Got: 0x%02X", who_am_i);
        ESP_LOGE(TAG, "❌ MPU6050 INITIALIZATION FAILED - Wrong device");
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

    // FORCE DEBUG LOGGING - temporary to diagnose zero values
    static int debug_counter = 0;
    debug_counter++;

    // Always log first 5 reads, then every 100 samples
    if (debug_counter <= 5 || debug_counter % 100 == 0) {
        ESP_LOGE(TAG, "🔍 MPU6050 RAW DATA [sample %d]: AX=%d AY=%d AZ=%d | GX=%d GY=%d GZ=%d",
                debug_counter, raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z);
        ESP_LOGE(TAG, "🔍 MPU6050 CONVERTED [sample %d]: AX=%.3f AY=%.3f AZ=%.3f | GX=%.2f GY=%.2f GZ=%.2f",
                debug_counter, data->accel_x, data->accel_y, data->accel_z, data->gyro_x, data->gyro_y, data->gyro_z);
    }

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