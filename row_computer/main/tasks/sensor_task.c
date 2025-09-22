#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"

// High-frequency IMU task - combines all motion sensors
void imu_task(void *parameters) {
    ESP_LOGD("IMU_TASK", "Starting IMU task at %dHz", 1000/IMU_TASK_PERIOD_MS);

    imu_data_t imu_data;
    mpu6050_data_t mpu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    static uint32_t health_counter = 0;

    // Health tracking counters
    static uint32_t mpu_total_reads = 0;
    static uint32_t mpu_successful_reads = 0;
    static uint32_t mag_total_reads = 0;
    static uint32_t mag_successful_reads = 0;

    while (1) {
        // Read MPU6050 data (accelerometer + gyroscope) in one efficient operation
        esp_err_t mpu_err = mpu6050_read_all(&mpu_data);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);

        // Update health tracking counters
        mpu_total_reads++;
        if (mpu_err == ESP_OK) {
            mpu_successful_reads++;
        }

        mag_total_reads++;
        if (mag_err == ESP_OK) {
            mag_successful_reads++;
        }

        // Copy MPU6050 data to IMU structure
        if (mpu_err == ESP_OK) {
            imu_data.accel_x = mpu_data.accel_x;
            imu_data.accel_y = mpu_data.accel_y;
            imu_data.accel_z = mpu_data.accel_z;
            imu_data.gyro_x = mpu_data.gyro_x;
            imu_data.gyro_y = mpu_data.gyro_y;
            imu_data.gyro_z = mpu_data.gyro_z;
            imu_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;


            // Send to legacy queue (non-blocking to maintain timing)
            if (xQueueSend(imu_data_queue, &imu_data, 0) != pdTRUE) {
                ESP_LOGW("IMU_TASK", "Legacy queue full - dropping IMU sample");
            }

            // Send to new ultra-responsive raw IMU queue with smart overflow management
            if (xQueueSend(raw_imu_data_queue, &imu_data, 0) != pdTRUE) {
                // Queue full - drop oldest sample and insert newest for ultra-responsiveness
                imu_data_t discarded_data;
                if (xQueueReceive(raw_imu_data_queue, &discarded_data, 0) == pdTRUE) {
                    // Successfully removed oldest, now add newest
                    if (xQueueSend(raw_imu_data_queue, &imu_data, 0) == pdTRUE) {
                        ESP_LOGW("IMU_TASK", "Raw IMU queue full - dropped sample from %lu ms, kept %lu ms",
                                discarded_data.timestamp_ms, imu_data.timestamp_ms);
                    } else {
                        ESP_LOGE("IMU_TASK", "Failed to add to raw IMU queue after clearing space");
                    }
                } else {
                    ESP_LOGE("IMU_TASK", "Raw IMU queue full but couldn't remove oldest sample");
                }
            }
        } else {
            ESP_LOGW("IMU_TASK", "Failed to read MPU6050: %s", esp_err_to_name(mpu_err));
        }

        // Log sensor health periodically
        if (++health_counter >= SENSOR_HEALTH_LOG_INTERVAL) {
            ESP_LOGI("IMU_TASK", "A[%.2f, %.2f, %.2f]g | G[%.1f, %.1f, %.1f]°/s | M[%.1f, %.1f, %.1f]μT",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
            health_counter = 0;
        }

        // Maintain exact timing using common constants
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}