#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/timing_config.h"
#include "sensors/sensors_common.h"
#include "utils/timing.h"

static const char *TAG = "IMU_TASK";

// High-frequency IMU task with enhanced timing and error handling
void imu_task(void *parameters) {
    ESP_LOGI(TAG, "Starting IMU task at %d Hz (%d ms intervals)", 
             1000/IMU_TASK_RATE_MS, IMU_TASK_RATE_MS);
    
    imu_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t sample_counter = 0;
    uint32_t successful_samples = 0;
    
    // Initialize local sensor status tracking
    sensor_status_t local_accel_status, local_gyro_status, local_mag_status;
    init_sensor_status(&local_accel_status);
    init_sensor_status(&local_gyro_status);
    init_sensor_status(&local_mag_status);
    
    while (1) {
        sample_counter++;
        
        // Create timestamp at start of sensor reading
        create_timestamp(&imu_data.timestamp, false); // Will update validity below
        
        // Initialize data to safe values
        imu_data.accel_x = imu_data.accel_y = imu_data.accel_z = 0.0f;
        imu_data.gyro_x = imu_data.gyro_y = imu_data.gyro_z = 0.0f;
        imu_data.mag_x = imu_data.mag_y = imu_data.mag_z = 0.0f;
        imu_data.temperature = 0.0f;
        
        // Read all IMU sensors in sequence (they share I2C bus)
        esp_err_t accel_err = accel_read(&imu_data.accel_x, &imu_data.accel_y, &imu_data.accel_z);
        esp_err_t gyro_err = gyro_read(&imu_data.gyro_x, &imu_data.gyro_y, &imu_data.gyro_z);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);
        
        // Update local sensor health tracking
        update_sensor_status(&local_accel_status, accel_err == ESP_OK);
        update_sensor_status(&local_gyro_status, gyro_err == ESP_OK);
        update_sensor_status(&local_mag_status, mag_err == ESP_OK);
        
        // Data is valid if at least accelerometer worked (critical for stroke detection)
        bool data_valid = (accel_err == ESP_OK);
        imu_data.timestamp.data_valid = data_valid;
        
        if (data_valid) {
            successful_samples++;
            
            // Send to queue with overflow handling
            if (SEND_IMU_DATA(imu_data)) {
                // Successfully sent
                LOG_IMU_RATE_LIMITED(D, "IMU sample %lu sent at %lu ms", 
                                   sample_counter, imu_data.timestamp.timestamp_ms);
            } else {
                ESP_LOGW(TAG, "Failed to send IMU data - queue system error");
            }
        } else {
            ESP_LOGW(TAG, "IMU sample %lu invalid - accelerometer failed: %s", 
                    sample_counter, esp_err_to_name(accel_err));
        }
        
        // Periodic health reporting and sensor reset attempts
        if (sample_counter % HEALTH_LOG_INTERVAL_COUNT == 0) { // Every ~5 seconds
            float success_rate = (float)successful_samples / (float)sample_counter * 100.0f;
            
            ESP_LOGI(TAG, "Health report - Sample %lu, Success rate: %.1f%%", 
                    sample_counter, success_rate);
            
            log_sensor_health(&local_accel_status, "Accelerometer");
            log_sensor_health(&local_gyro_status, "Gyroscope");
            log_sensor_health(&local_mag_status, "Magnetometer");
            
            // Attempt sensor resets if needed
            if (sensor_needs_reset(&local_accel_status)) {
                ESP_LOGW(TAG, "Attempting accelerometer reset...");
                if (accel_init() == ESP_OK) {
                    ESP_LOGI(TAG, "Accelerometer reset successful");
                    init_sensor_status(&local_accel_status); // Reset counters
                } else {
                    ESP_LOGE(TAG, "Accelerometer reset failed");
                }
            }
            
            if (sensor_needs_reset(&local_gyro_status)) {
                ESP_LOGW(TAG, "Attempting gyroscope reset...");
                if (gyro_init() == ESP_OK) {
                    ESP_LOGI(TAG, "Gyroscope reset successful");
                    init_sensor_status(&local_gyro_status);
                } else {
                    ESP_LOGE(TAG, "Gyroscope reset failed");
                }
            }
            
            if (sensor_needs_reset(&local_mag_status)) {
                ESP_LOGW(TAG, "Attempting magnetometer reset...");
                if (mag_init() == ESP_OK) {
                    ESP_LOGI(TAG, "Magnetometer reset successful");
                    init_sensor_status(&local_mag_status);
                } else {
                    ESP_LOGE(TAG, "Magnetometer reset failed");
                }
            }
            
            // Update global health status
            global_system_health.accel_status = local_accel_status;
            global_system_health.gyro_status = local_gyro_status;
            global_system_health.mag_status = local_mag_status;
        }
        
        // Log detailed sensor readings occasionally in verbose mode
        if (verbose_mode && sample_counter % 100 == 0) { // Every 1 second at 100Hz
            ESP_LOGD(TAG, "Detailed reading - Accel:[%.3f,%.3f,%.3f]g Gyro:[%.1f,%.1f,%.1f]°/s Mag:[%.1f,%.1f,%.1f]G", 
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
        }
        
        // Maintain exact timing discipline
        TASK_DELAY_UNTIL(last_wake_time, IMU_TASK_RATE_MS);
    }
}