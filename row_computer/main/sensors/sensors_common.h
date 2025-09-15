#ifndef SENSORS_COMMON_H
#define SENSORS_COMMON_H

#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag.h"
#include "sensors/gps.h"
#include "utils/timing.h"

// =============================================================================
// ENHANCED DATA STRUCTURES WITH TIMING
// =============================================================================

typedef struct {
    sensor_timestamp_t timestamp;           // Timing info + validity flag
    float accel_x, accel_y, accel_z;        // Acceleration in g-forces
    float gyro_x, gyro_y, gyro_z;           // Angular velocity in deg/s
    float mag_x, mag_y, mag_z;              // Magnetic field in gauss
    float temperature;                      // Temperature in celsius (if available)
    // Total size: ~36 bytes (4*10 floats + timestamp struct)
} imu_data_t;

typedef struct {
    sensor_timestamp_t timestamp;           // Timing info + validity flag
    char time_string[16];                   // GPS time "HH:MM:SS" 
    double latitude;                        // Latitude in decimal degrees
    double longitude;                       // Longitude in decimal degrees
    float speed_knots;                      // Speed in knots
    float heading;                          // Heading in degrees
    int satellites;                         // Number of satellites
    bool valid_fix;                         // GPS fix validity
    // Total size: ~64 bytes
} gps_data_t;

// Future: Output data structure for processed/fused data
typedef struct {
    sensor_timestamp_t timestamp;           // Timing info + validity flag
    double position_lat, position_lon;      // Fused position
    float velocity_ms;                      // Speed in m/s
    float heading_deg;                      // Heading in degrees
    float stroke_rate;                      // Strokes per minute
    uint32_t stroke_count;                  // Total strokes
    // Additional rowing metrics can be added here
    // Total size: ~64 bytes
} output_data_t;

// =============================================================================
// SENSOR HEALTH STRUCTURES
// =============================================================================

typedef struct {
    sensor_status_t accel_status;
    sensor_status_t gyro_status;
    sensor_status_t mag_status;
    sensor_status_t gps_status;
    uint32_t last_health_report_ms;
} system_health_t;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

/**
 * @brief Initialize all sensors and their health monitoring
 * @return ESP_OK if critical sensors initialized successfully
 */
esp_err_t sensors_init_with_health(system_health_t *health);

/**
 * @brief Read IMU data with timing and health updates
 * @param imu_data Pointer to IMU data structure to fill
 * @param health Pointer to system health structure
 * @return ESP_OK if at least accelerometer read successfully
 */
esp_err_t read_imu_with_timing(imu_data_t *imu_data, system_health_t *health);

/**
 * @brief Read GPS data with timing and health updates
 * @param gps_data Pointer to GPS data structure to fill
 * @param health Pointer to system health structure
 * @return ESP_OK if GPS read successfully
 */
esp_err_t read_gps_with_timing(gps_data_t *gps_data, system_health_t *health);

/**
 * @brief Log system health status
 * @param health Pointer to system health structure
 */
void log_system_health(system_health_t *health);

// =============================================================================
// EXISTING FUNCTION DECLARATIONS (maintained for compatibility)
// =============================================================================

// Initialize all sensors
void sensors_init(void);

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