#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

// MPU6050 sensor data structure
typedef struct {
    float accel_x, accel_y, accel_z;    // Acceleration in g-forces
    float gyro_x, gyro_y, gyro_z;       // Angular velocity in deg/s
} mpu6050_data_t;

// Initialize the MPU6050 sensor (both accelerometer and gyroscope)
esp_err_t mpu6050_init(void);

// Read both accelerometer and gyroscope data in one operation
esp_err_t mpu6050_read_all(mpu6050_data_t *data);

// Individual read functions for backward compatibility
esp_err_t mpu6050_read_accel(float *x, float *y, float *z);
esp_err_t mpu6050_read_gyro(float *x, float *y, float *z);

#endif // MPU6050_H