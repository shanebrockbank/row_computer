#include "esp_log.h"
#include "driver/i2c.h"
#include "config/pin_definitions.h"
#include "sensors/accel.h"
#include "sensors/gyro.h"
#include "sensors/mag.h"
#include "sensors/gps.h"
#include "sensors/sensors_common.h"
#include "utils/timing.h"

static const char *TAG = "SENSORS_COMMON";

// =============================================================================
// ENHANCED SENSOR INITIALIZATION WITH HEALTH MONITORING
// =============================================================================

esp_err_t sensors_init_with_health(system_health_t *health) {
    ESP_LOGI(TAG, "Initializing sensors with health monitoring...");
    
    if (!health) {
        ESP_LOGE(TAG, "Health structure is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize health structures
    init_sensor_status(&health->accel_status);
    init_sensor_status(&health->gyro_status);
    init_sensor_status(&health->mag_status);
    init_sensor_status(&health->gps_status);
    
    // Initialize sensors and update health status
    esp_err_t accel_err = accel_init();
    esp_err_t gyro_err = gyro_init();
    esp_err_t mag_err = mag_init();
    esp_err_t gps_err = gps_init();
    
    // Update initial health status
    health->accel_status.sensor_available = (accel_err == ESP_OK);
    health->gyro_status.sensor_available = (gyro_err == ESP_OK);
    health->mag_status.sensor_available = (mag_err == ESP_OK);
    health->gps_status.sensor_available = (gps_err == ESP_OK);
    
    if (accel_err == ESP_OK) {
        health->accel_status.last_successful_read_ms = get_system_time_ms();
    }
    if (gyro_err == ESP_OK) {
        health->gyro_status.last_successful_read_ms = get_system_time_ms();
    }
    if (mag_err == ESP_OK) {
        health->mag_status.last_successful_read_ms = get_system_time_ms();
    }
    if (gps_err == ESP_OK) {
        health->gps_status.last_successful_read_ms = get_system_time_ms();
    }
    
    // Log initialization results
    ESP_LOGI(TAG, "Sensor initialization results:");
    ESP_LOGI(TAG, "  Accelerometer: %s", (accel_err == ESP_OK) ? "✅ OK" : "❌ FAILED");
    ESP_LOGI(TAG, "  Gyroscope:     %s", (gyro_err == ESP_OK) ? "✅ OK" : "❌ FAILED");
    ESP_LOGI(TAG, "  Magnetometer:  %s", (mag_err == ESP_OK) ? "✅ OK" : "❌ FAILED");
    ESP_LOGI(TAG, "  GPS:           %s", (gps_err == ESP_OK) ? "✅ OK" : "❌ FAILED");
    
    // Return ESP_OK if critical sensor (accelerometer) is working
    return (accel_err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

// =============================================================================
// ENHANCED SENSOR READING WITH TIMING AND HEALTH UPDATES
// =============================================================================

esp_err_t read_imu_with_timing(imu_data_t *imu_data, system_health_t *health) {
    if (!imu_data) {
        ESP_LOGE(TAG, "IMU data structure is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create timestamp at start of reading
    create_timestamp(&imu_data->timestamp, false);
    
    // Initialize to safe values
    imu_data->accel_x = imu_data->accel_y = imu_data->accel_z = 0.0f;
    imu_data->gyro_x = imu_data->gyro_y = imu_data->gyro_z = 0.0f;
    imu_data->mag_x = imu_data->mag_y = imu_data->mag_z = 0.0f;
    imu_data->temperature = 25.0f; // Default room temperature
    
    // Read sensors
    esp_err_t accel_err = accel_read(&imu_data->accel_x, &imu_data->accel_y, &imu_data->accel_z);
    esp_err_t gyro_err = gyro_read(&imu_data->gyro_x, &imu_data->gyro_y, &imu_data->gyro_z);
    esp_err_t mag_err = mag_read(&imu_data->mag_x, &imu_data->mag_y, &imu_data->mag_z);
    
    // Update health tracking if provided
    if (health) {
        update_sensor_status(&health->accel_status, accel_err == ESP_OK);
        update_sensor_status(&health->gyro_status, gyro_err == ESP_OK);
        update_sensor_status(&health->mag_status, mag_err == ESP_OK);
    }
    
    // Data is valid if accelerometer worked (critical sensor for rowing)
    bool data_valid = (accel_err == ESP_OK);
    imu_data->timestamp.data_valid = data_valid;
    
    // Log detailed errors if in verbose mode
    if (verbose_mode) {
        if (accel_err != ESP_OK) {
            ESP_LOGD(TAG, "Accelerometer read failed: %s", esp_err_to_name(accel_err));
        }
        if (gyro_err != ESP_OK) {
            ESP_LOGD(TAG, "Gyroscope read failed: %s", esp_err_to_name(gyro_err));
        }
        if (mag_err != ESP_OK) {
            ESP_LOGD(TAG, "Magnetometer read failed: %s", esp_err_to_name(mag_err));
        }
    }
    
    return data_valid ? ESP_OK : ESP_FAIL;
}

esp_err_t read_gps_with_timing(gps_data_t *gps_data, system_health_t *health) {
    if (!gps_data) {
        ESP_LOGE(TAG, "GPS data structure is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create timestamp at start of reading
    create_timestamp(&gps_data->timestamp, false);
    
    // Initialize GPS data to safe values
    memset(gps_data->time_string, 0, sizeof(gps_data->time_string));
    gps_data->latitude = 0.0;
    gps_data->longitude = 0.0;
    gps_data->speed_knots = 0.0f;
    gps_data->heading = 0.0f;
    gps_data->satellites = 0;
    gps_data->valid_fix = false;
    
    // Read GPS data using existing function
    esp_err_t gps_err = gps_read(gps_data);
    
    // Update health tracking if provided
    if (health) {
        update_sensor_status(&health->gps_status, gps_err == ESP_OK);
    }
    
    gps_data->timestamp.data_valid = (gps_err == ESP_OK);
    
    // Log detailed errors if in verbose mode
    if (verbose_mode && gps_err != ESP_OK) {
        ESP_LOGD(TAG, "GPS read failed: %s", esp_err_to_name(gps_err));
    }
    
    return gps_err;
}

// =============================================================================
// ORIGINAL FUNCTIONS (maintained for backwards compatibility)
// =============================================================================

void sensors_init(void) {
    // Simple wrapper around enhanced function
    system_health_t temp_health;
    sensors_init_with_health(&temp_health);
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