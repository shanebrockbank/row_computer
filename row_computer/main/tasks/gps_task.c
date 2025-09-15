#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "sensors/gps.h"
#include "sensors/sensors_common.h"
#include "config/timing_config.h"
#include "utils/timing.h"

static const char *TAG = "GPS_TASK";

void gps_task(void *parameters) {
    ESP_LOGI(TAG, "Starting GPS task at %d Hz (%d ms intervals)", 
             1000/GPS_TASK_RATE_MS, GPS_TASK_RATE_MS);
    
    gps_data_t gps_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t sample_counter = 0;
    uint32_t successful_samples = 0;
    uint32_t valid_fix_samples = 0;
    
    // Initialize local GPS health tracking
    sensor_status_t local_gps_status;
    init_sensor_status(&local_gps_status);
    
    // Add a startup delay to let GPS module settle
    ESP_LOGI(TAG, "Waiting for GPS module to initialize...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Optional: Run a communication test
    if (gps_test_communication() == ESP_OK) {
        ESP_LOGI(TAG, "GPS communication verified");
        local_gps_status.sensor_available = true;
    } else {
        ESP_LOGI(TAG, "GPS communication test failed - will continue trying");
    }
    
    while (1) {
        sample_counter++;
        
        // Create timestamp at start of GPS reading
        create_timestamp(&gps_data.timestamp, false); // Will update validity below
        
        // Initialize GPS data to safe values
        memset(gps_data.time_string, 0, sizeof(gps_data.time_string));
        gps_data.latitude = 0.0;
        gps_data.longitude = 0.0;
        gps_data.speed_knots = 0.0f;
        gps_data.heading = 0.0f;
        gps_data.satellites = 0;
        gps_data.valid_fix = false;
        
        // Read GPS data
        esp_err_t gps_err = gps_read(&gps_data);
        
        // Update local sensor health tracking
        update_sensor_status(&local_gps_status, gps_err == ESP_OK);
        
        if (gps_err == ESP_OK) {
            successful_samples++;
            gps_data.timestamp.data_valid = true;
            
            if (gps_data.valid_fix) {
                valid_fix_samples++;
            }
            
            // Send to queue with overflow handling
            if (SEND_GPS_DATA(gps_data)) {
                // Successfully sent
                ESP_LOGD(TAG, "GPS sample %lu sent at %lu ms", 
                        sample_counter, gps_data.timestamp.timestamp_ms);
            } else {
                ESP_LOGW(TAG, "Failed to send GPS data - queue system error");
            }
            
            // Log GPS status periodically
            if (sample_counter % 10 == 0) { // Every 10 samples (10 seconds at 1Hz)
                float success_rate = (float)successful_samples / (float)sample_counter * 100.0f;
                float fix_rate = (float)valid_fix_samples / (float)successful_samples * 100.0f;
                
                if (gps_data.valid_fix) {
                    ESP_LOGI(TAG, "✓ Fix: %.6f°, %.6f° | Speed: %.1f kts | Sats: %d | Success: %.1f%% | Fix rate: %.1f%%",
                            gps_data.latitude, gps_data.longitude, 
                            gps_data.speed_knots, gps_data.satellites,
                            success_rate, fix_rate);
                } else {
                    ESP_LOGI(TAG, "⚠ No fix | Sats: %d | Success: %.1f%% | Fix rate: %.1f%%", 
                            gps_data.satellites, success_rate, fix_rate);
                }
            }
            
        } else {
            gps_data.timestamp.data_valid = false;
            
            // Log failures but don't spam
            LOG_GENERAL_RATE_LIMITED(TAG, W, "GPS read failed: %s (sample %lu)", 
                                   esp_err_to_name(gps_err), sample_counter);
        }
        
        // Periodic health reporting and sensor reset attempts
        if (sample_counter % 60 == 0) { // Every 60 samples (1 minute at 1Hz)
            float success_rate = (float)successful_samples / (float)sample_counter * 100.0f;
            float fix_rate = successful_samples > 0 ? 
                           (float)valid_fix_samples / (float)successful_samples * 100.0f : 0.0f;
            
            ESP_LOGI(TAG, "Health report - Sample %lu, Success rate: %.1f%%, Fix rate: %.1f%%", 
                    sample_counter, success_rate, fix_rate);
            
            log_sensor_health(&local_gps_status, "GPS");
            
            // Attempt GPS reset if needed
            if (sensor_needs_reset(&local_gps_status)) {
                ESP_LOGW(TAG, "Attempting GPS reset...");
                
                // Run debug to see what's happening
                ESP_LOGI(TAG, "Running GPS debug before reset...");
                gps_debug_raw_data();
                
                if (gps_init() == ESP_OK) {
                    ESP_LOGI(TAG, "GPS reset successful");
                    init_sensor_status(&local_gps_status); // Reset counters
                    
                    // Test communication after reset
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    if (gps_test_communication() == ESP_OK) {
                        ESP_LOGI(TAG, "GPS communication verified after reset");
                    }
                } else {
                    ESP_LOGE(TAG, "GPS reset failed");
                }
            }
            
            // Update global health status
            global_system_health.gps_status = local_gps_status;
        }
        
        // Log detailed GPS readings in verbose mode
        if (verbose_mode && gps_data.timestamp.data_valid) {
            ESP_LOGD(TAG, "Detailed reading - Time:%s Fix:%s Lat:%.6f Lon:%.6f Speed:%.1f Heading:%.1f Sats:%d", 
                    gps_data.time_string, gps_data.valid_fix ? "YES" : "NO",
                    gps_data.latitude, gps_data.longitude, 
                    gps_data.speed_knots, gps_data.heading, gps_data.satellites);
        }
        
        // Maintain exact timing discipline
        TASK_DELAY_UNTIL(last_wake_time, GPS_TASK_RATE_MS);
    }
}