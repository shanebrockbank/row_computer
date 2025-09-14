#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "sensors_common.h"

// High-frequency IMU task - combines all motion sensors
void imu_task(void *parameters) {
    ESP_LOGI("IMU_TASK", "Starting IMU task at 100Hz");
    
    imu_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Read all IMU sensors in sequence (they share I2C bus)
        esp_err_t accel_err = accel_read(&imu_data.accel_x, &imu_data.accel_y, &imu_data.accel_z);
        esp_err_t gyro_err = gyro_read(&imu_data.gyro_x, &imu_data.gyro_y, &imu_data.gyro_z);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);
        
        // Only send data if at least accelerometer worked (critical for stroke detection)
        if (accel_err == ESP_OK) {
            imu_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue (non-blocking to maintain timing)
            if (xQueueSend(imu_data_queue, &imu_data, 0) != pdTRUE) {
                ESP_LOGW("IMU_TASK", "Queue full - dropping IMU sample");
            }
        } else {
            ESP_LOGW("IMU_TASK", "Failed to read accelerometer: %s", esp_err_to_name(accel_err));
        }
        
        // Log sensor health periodically
        static int health_counter = 0;
        if (++health_counter >= 1000) { // Every 10 seconds at 100Hz
            ESP_LOGI("IMU_TASK", "Sensor health - Accel:%s Gyro:%s Mag:%s", 
                    (accel_err == ESP_OK) ? "OK" : "FAIL",
                    (gyro_err == ESP_OK) ? "OK" : "FAIL", 
                    (mag_err == ESP_OK) ? "OK" : "FAIL");
            health_counter = 0;
        }
        
        // Maintain exact 100Hz timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}