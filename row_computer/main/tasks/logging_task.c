#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"

// Data processing and logging task
void logging_task(void *parameters) {
    ESP_LOGD("LOG_TASK", "Starting logging task");
    
    imu_data_t imu_data;
    gps_data_t gps_data;
    
    while (1) {
        // Process IMU data (high frequency)
        while (xQueueReceive(imu_data_queue, &imu_data, 0) == pdTRUE) {
            // Here you would:
            // 1. Apply filtering/calibration
            // 2. Detect rowing strokes (to come later after more data)
            // 3. Calculate boat motion metrics
            // 4. Log to SD card
            
            // For now, just log interesting events
            float accel_magnitude = sqrt(imu_data.accel_x * imu_data.accel_x + 
                                       imu_data.accel_y * imu_data.accel_y + 
                                       imu_data.accel_z * imu_data.accel_z);
            
            // Detect potential stroke (high acceleration event)
            if (accel_magnitude > STROKE_DETECTION_THRESHOLD) {
                ESP_LOGI("LOG_TASK", "Stroke detected: %.2fg at %lu ms",
                        accel_magnitude, imu_data.timestamp_ms);
            }
        }
        
        // Process GPS data (low frequency)
        if (xQueueReceive(gps_data_queue, &gps_data, 0) == pdTRUE) {
            if (gps_data.valid_fix) {
                // Calculate distance, track boat path, etc.
                ESP_LOGI("LOG_TASK", "GPS logged: %.6f,%.6f @ %.1f kts (%lu ms)",
                        gps_data.latitude, gps_data.longitude, 
                        gps_data.speed_knots, gps_data.timestamp_ms);
            }
        }
        
        // Run at moderate frequency to process queued data
        vTaskDelay(pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
    }
}