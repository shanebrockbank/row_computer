#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"

static const char *TAG = "ACCEL";

// Write register
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, write_buf, 2, pdMS_TO_TICKS(1000));
}

// Read registers
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}

// Combine bytes
int16_t combine_bytes(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

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
    // Read raw values from accelerometer
    uint8_t data[6];
    esp_err_t err = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, data, 6);
    if (err == ESP_OK){
        int16_t raw_x = combine_bytes(data[0], data[1]);
        int16_t raw_y = combine_bytes(data[2], data[3]);
        int16_t raw_z = combine_bytes(data[4], data[5]);

        *x = (float)raw_x / LSB_SENSITIVITY_2g;
        *y = (float)raw_y / LSB_SENSITIVITY_2g;
        *z = (float)raw_z / LSB_SENSITIVITY_2g;
    }else{
        ESP_LOGE(TAG, "Failed to read MPU6050: %s", esp_err_to_name(err));
    }
    
    return ESP_OK;
}