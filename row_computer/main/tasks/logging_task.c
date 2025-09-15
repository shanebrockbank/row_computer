#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/timing_config.h"
#include "sensors/sensors_common.h"
#include "utils/timing.h"

static const char *TAG = "LOG_TASK";

// Data processing and logging task with enhanced timing
void logging_task(void *parameters) {
    ESP_LOGI(TAG, "Starting logging task at %d Hz (%d ms intervals)", 
             1000/LOG_TASK_RATE_MS, LOG_TASK_RATE_MS);
    
    imu_data_t imu_data;
    gps_data_t gps_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Statistics tracking
    uint32_t imu_samples_processed = 0;
    uint32_t gps_samples_processed = 0;
    uint32_t stroke_detections = 0;
    uint32_t last_stats_time_ms = 0;
    
    // Simple stroke detection state
    float last_accel_magnitude = 0.0f;
    bool in_stroke = false;
    
    while (1) {
        uint32_t current_time = get_system_time_ms();
        bool processed_data = false;
        
        // Process all available IMU data (high frequency)
        while (xQueueReceive(imu_data_queue, &imu_data, 0) == pdTRUE) {
            processed_data = true;
            
            if (imu_data.timestamp.data_valid) {
                imu_samples_processed++;
                
                // Simple rowing stroke detection
                float accel_magnitude = sqrt(imu_data.accel_x * imu_data.accel_x + 
                                           imu_data.accel_y * imu_data.accel_y + 
                                           imu_data.accel_z * imu_data.accel_z);
                
                // Detect potential stroke (simple threshold with hysteresis)
                if (!in_stroke && accel_magnitude > 2.0f && last_accel_magnitude < 1.8f) {
                    // Rising edge - stroke start
                    in_stroke = true;
                    stroke_detections++;
                    
                    ESP_LOGI(TAG, "Stroke #%lu detected: %.2fg at %lu ms (age: %lu ms)", 
                            stroke_detections, accel_magnitude, 
                            imu_data.timestamp.timestamp_ms,
                            current_time - imu_data.timestamp.timestamp_ms);
                    
                    // Here you would:
                    // 1. Apply filtering/calibration
                    // 2. Calculate boat motion metrics  
                    // 3. Prepare data for sensor fusion
                    // 4. Log to SD card
                    
                } else if (in_stroke && accel_magnitude < 1.5f) {
                    // Falling edge - stroke end
                    in_stroke = false;
                }
                
                last_accel_magnitude = accel_magnitude;
                
                // Log detailed IMU data in verbose mode with rate limiting
                if (verbose_mode && should_log_general_high_freq()) {
                    ESP_LOGD(TAG, "IMU processed: A[%.2f,%.2f,%.2f]g G[%.1f,%.1f,%.1f]°/s M[%.1f,%.1f,%.1f]G (age: %lu ms)",
                            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                            imu_data.mag_x, imu_data.mag_y, imu_data.mag_z,
                            current_time - imu_data.timestamp.timestamp_ms);
                }
            } else {
                ESP_LOGD(TAG, "Received invalid IMU sample (sensor read failed)");
            }
        }
        
        // Process GPS data (low frequency)
        while (xQueueReceive(gps_data_queue, &gps_data, 0) == pdTRUE) {
            processed_data = true;
            
            if (gps_data.timestamp.data_valid) {
                gps_samples_processed++;
                
                if (gps_data.valid_fix) {
                    // Here you would:
                    // 1. Calculate distance traveled
                    // 2. Track boat path
                    // 3. Compute speed/course over ground
                    // 4. Prepare for sensor fusion with IMU data
                    
                    ESP_LOGI(TAG, "GPS processed: %.6f,%.6f @ %.1f kts, %d sats at %lu ms (age: %lu ms)",
                            gps_data.latitude, gps_data.longitude, 
                            gps_data.speed_knots, gps_data.satellites,
                            gps_data.timestamp.timestamp_ms,
                            current_time - gps_data.timestamp.timestamp_ms);
                } else {
                    ESP_LOGD(TAG, "GPS processed: No fix, %d sats (age: %lu ms)",
                            gps_data.satellites,
                            current_time - gps_data.timestamp.timestamp_ms);
                }
            } else {
                ESP_LOGD(TAG, "Received invalid GPS sample (sensor read failed)");
            }
        }
        
        // Log processing statistics periodically
        if (current_time - last_stats_time_ms >= 10000) { // Every 10 seconds
            float imu_rate = (float)imu_samples_processed / 10.0f;
            float gps_rate = (float)gps_samples_processed / 10.0f;
            float stroke_rate = (float)stroke_detections * 6.0f; // Strokes per minute (last 10s * 6)
            
            ESP_LOGI(TAG, "Processing stats: IMU=%.1fHz GPS=%.1fHz Strokes=%lu (%.1f/min)", 
                    imu_rate, gps_rate, stroke_detections, stroke_rate);
            
            // Log queue health
            log_queue_health(imu_data_queue, "IMU", IMU_QUEUE_SIZE, 100);
            log_queue_health(gps_data_queue, "GPS", GPS_QUEUE_SIZE, 1);
            
            // Reset counters
            imu_samples_processed = 0;
            gps_samples_processed = 0;
            stroke_detections = 0; // Reset for next interval
            last_stats_time_ms = current_time;
        }
        
        // If no data was processed, we can sleep a bit longer
        if (!processed_data) {
            ESP_LOGD(TAG, "No data to process, queues empty");
        }
        
        // Maintain task timing discipline
        TASK_DELAY_UNTIL(last_wake_time, LOG_TASK_RATE_MS);
    }
}

// System health monitoring task
void health_monitor_task(void *parameters) {
    ESP_LOGI("HEALTH_TASK", "Starting health monitoring task at %d ms intervals", 
             HEALTH_MONITOR_RATE_MS);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t health_report_counter = 0;
    
    while (1) {
        health_report_counter++;
        
        // Update global health timestamp
        global_system_health.last_health_report_ms = get_system_time_ms();
        
        // Log comprehensive system health
        ESP_LOGI("HEALTH_TASK", "=== System Health Report #%lu ===", health_report_counter);
        
        log_system_health(&global_system_health);
        
        // Log queue health for all queues
        log_queue_health(imu_data_queue, "IMU", IMU_QUEUE_SIZE, 100);
        log_queue_health(gps_data_queue, "GPS", GPS_QUEUE_SIZE, 1);
        
        // System resource monitoring
        uint32_t free_heap = esp_get_free_heap_size();
        ESP_LOGI("HEALTH_TASK", "System resources: Free heap=%lu bytes (%.1f KB)", 
                free_heap, free_heap / 1024.0f);
        
        if (free_heap < 50000) { // Less than 50KB free
            ESP_LOGW("HEALTH_TASK", "Low memory warning: only %lu bytes free", free_heap);
        }
        
        // Print detailed timing status occasionally
        if (health_report_counter % 12 == 0) { // Every minute
            print_system_timing_status();
        }
        
        // Maintain exact timing
        TASK_DELAY_UNTIL(last_wake_time, HEALTH_MONITOR_RATE_MS);
    }
}