#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H

// MPU6050 Write register
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data);

// MPU6050  Read registers
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

// Combine bytes
int16_t combine_bytes(uint8_t high, uint8_t low);

// HMC5883L  Write register
esp_err_t mag_write_byte(uint8_t reg_addr, uint8_t data);

// HMC5883L  Read registers
esp_err_t mag_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

#endif