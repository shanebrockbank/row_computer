#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H

#include "sensors/mpu6050.h"
#include "sensors/mag.h"
#include "sensors/gps.h"

typedef struct {
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;    // g-forces
    float gyro_x, gyro_y, gyro_z;       // deg/s
    float mag_x, mag_y, mag_z;          // gauss
} imu_data_t;

// Processed IMU data after calibration and filtering
typedef struct {
    uint32_t timestamp_ms;
    float accel_x, accel_y, accel_z;    // calibrated g-forces
    float gyro_x, gyro_y, gyro_z;       // calibrated deg/s
    float mag_x, mag_y, mag_z;          // calibrated gauss
    // TODO: Add filtered/derived values like:
    // float accel_magnitude;
    // float attitude_roll, attitude_pitch, attitude_yaw;
} processed_imu_data_t;

// Fused motion state combining IMU and GPS data
typedef struct {
    uint32_t timestamp_ms;

    // IMU-derived motion data
    float accel_x, accel_y, accel_z;    // g-forces
    float gyro_x, gyro_y, gyro_z;       // deg/s
    float total_acceleration;           // magnitude

    // GPS position and velocity data
    double latitude, longitude;         // degrees
    float gps_speed_knots;             // knots
    bool gps_valid;                    // GPS fix status
    uint8_t satellites;                // satellite count

    // TODO: Add fused state estimates:
    // float velocity_x, velocity_y;      // m/s in body frame
    // float position_x, position_y;      // m relative to start
    // float heading;                     // degrees
    // float stroke_rate;                 // strokes per minute
    // bool in_stroke;                    // stroke detection
} motion_state_t;

// Initialize all sensors
void sensors_init(void);

// MPU6050 I2C communication functions
esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

// Utility functions
int16_t combine_bytes(uint8_t high, uint8_t low);

// HMC5883L I2C communication functions
esp_err_t mag_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t mag_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

#endif