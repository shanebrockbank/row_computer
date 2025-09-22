#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/common_constants.h"
#include "sensors/gps.h"
#include "sensors/sensors_common.h"

static const char *TAG = "GPS_TASK";

void gps_task(void *parameters) {
    ESP_LOGD(TAG, "Starting GPS task");

    gps_data_t gps_data;
    gps_health_t gps_health;
    uint32_t consecutive_failures = 0;
    uint32_t total_reads = 0;
    uint32_t successful_reads = 0;
    
    // Add a startup delay to let GPS module settle
    ESP_LOGD(TAG, "Waiting for GPS module to initialize...");
    vTaskDelay(pdMS_TO_TICKS(GPS_STARTUP_DELAY_MS));

    // Optional: Run a communication test
    if (gps_test_communication() == ESP_OK) {
        ESP_LOGD(TAG, "GPS communication verified");
    } else {
        ESP_LOGD(TAG, "GPS communication test failed - will continue trying");
    }
    
    while (1) {
        total_reads++;
        esp_err_t gps_err = gps_read(&gps_data);
        
        if (gps_err == ESP_OK) {
            consecutive_failures = 0;
            successful_reads++;
            
            // Add timestamp
            gps_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue
            if (xQueueSend(gps_data_queue, &gps_data, pdMS_TO_TICKS(TIMEOUT_QUEUE_MS)) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send GPS data to queue");
            }

            // Log GPS status periodically with health data
            static int gps_log_counter = 0;
            if (++gps_log_counter >= GPS_LOG_INTERVAL) { // Every N successful reads
                // Read health data for debug output
                gps_read_health(&gps_health);

                uint32_t success_rate = total_reads > 0 ? (successful_reads * 100) / total_reads : 0;

                if (gps_data.valid_fix) {
                    ESP_LOGI(TAG, "Pos: %.6f°, %.6f° | Sats: %d | Accuracy: H=%.1f m | Speed: %.2f m/s | Heading: %.1f°",
                            gps_data.latitude, gps_data.longitude, gps_data.satellites,
                            gps_health.horizontal_accuracy / 1000.0f, gps_data.speed_knots * 0.514444f, gps_data.heading);
                } else {
                    ESP_LOGI(TAG, "GPS: NO FIX | Sats: %d | Success: %lu%%",
                            gps_data.satellites, success_rate);
                }
                gps_log_counter = 0;
            }
            
        } else {
            consecutive_failures++;
            
            // Log failures but don't spam
            if (consecutive_failures == 1) {
                ESP_LOGW(TAG, "GPS read failed: %s", esp_err_to_name(gps_err));
            } else if (consecutive_failures % 30 == 0) { // Every 30 failures
                ESP_LOGE(TAG, "GPS has failed %lu consecutive times. Check hardware!", consecutive_failures);
                ESP_LOGE(TAG, "Success rate: %lu%% (%lu/%lu)", 
                        (successful_reads * 100) / total_reads, successful_reads, total_reads);
                
                // Optionally run a debug session
                ESP_LOGI(TAG, "Running GPS debug...");
                gps_debug_raw_data();
            }
        }
        
        // Standard GPS update rate
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_PERIOD_MS));
    }
}