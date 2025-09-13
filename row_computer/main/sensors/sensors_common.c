#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"
#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag.h"
#include "sensors/gps.h"

static const char *TAG = "SENSORS_COMMON";

void sensors_init(void){
    accel_init();  // MPU6050 accelerometer
    gyro_init();   // MPU6050 gyroscope (same device)
    mag_init();    // HMC5883L magnetometer
    gps_init();    // NEO-6M GPS module
}

// Write register
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    // 1000ms timeout prevents system hang if sensor disconnected
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

// Write single register
esp_err_t mag_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, HMC5883L_ADDR, write_buf, 2, pdMS_TO_TICKS(1000));
}

// Read multiple registers
esp_err_t mag_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, HMC5883L_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}