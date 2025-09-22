#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"
#include "utils/queue_utils.h"
#include "utils/health_monitor.h"

// High-frequency IMU task - combines all motion sensors
void imu_task(void *parameters) {
    ESP_LOGD("IMU_TASK", "Starting IMU task at %dHz", 1000/IMU_TASK_PERIOD_MS);

    imu_data_t imu_data;
    mpu6050_data_t mpu_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // Read MPU6050 data (accelerometer + gyroscope) in one efficient operation
        esp_err_t mpu_err = mpu6050_read_all(&mpu_data);
        esp_err_t mag_err = mag_read(&imu_data.mag_x, &imu_data.mag_y, &imu_data.mag_z);

        // Update unified health tracking
        if (mpu_err == ESP_OK) {
            health_record_success(&g_system_health.imu_sensor);
        } else {
            health_record_failure(&g_system_health.imu_sensor);
        }

        if (mag_err == ESP_OK) {
            health_record_success(&g_system_health.mag_sensor);
        } else {
            health_record_failure(&g_system_health.mag_sensor);
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


            // Send to ultra-responsive raw IMU queue with smart overflow management
            queue_send_with_overflow(raw_imu_data_queue, &imu_data, sizeof(imu_data_t), "IMU_TASK", "Raw IMU", imu_data.timestamp_ms);
        } else {
            ESP_LOGW("IMU_TASK", "Failed to read MPU6050: %s", esp_err_to_name(mpu_err));
        }

        // Report unified health monitoring periodically
        if (health_should_report(&g_system_health.imu_sensor, SENSOR_HEALTH_LOG_INTERVAL)) {
            health_report_component(&g_system_health.imu_sensor, "IMU_TASK");
            health_report_component(&g_system_health.mag_sensor, "IMU_TASK");

            // Still log current sensor readings for debugging
            ESP_LOGI("IMU_TASK", "A[%.2f, %.2f, %.2f]g | G[%.1f, %.1f, %.1f]°/s | M[%.1f, %.1f, %.1f]μT",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
        }

        // Maintain exact timing using common constants
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}