#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/common_constants.h"
#include "utils/health_monitor.h"
#include "sensors/gps.h"
#include "sensors/sensors_common.h"

static const char *TAG = "GPS_TASK";

void gps_task(void *parameters) {
    ESP_LOGD(TAG, "Starting GPS task");

    gps_data_t gps_data;
    gps_health_t gps_health;
    uint32_t consecutive_failures = 0;
    
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
        esp_err_t gps_err = gps_read(&gps_data);

        if (gps_err == ESP_OK) {
            consecutive_failures = 0;
            health_record_success(&g_system_health.gps_sensor);
            
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

                health_report_component(&g_system_health.gps_sensor, "GPS_TASK");

                if (gps_data.valid_fix) {
                    ESP_LOGI(TAG, "Pos: %.6f°, %.6f° | Sats: %d | Accuracy: H=%.1f m | Speed: %.2f m/s | Heading: %.1f°",
                            gps_data.latitude, gps_data.longitude, gps_data.satellites,
                            gps_health.horizontal_accuracy / (float)MS_TO_US_MULTIPLIER, gps_data.speed_knots * 0.514444f, gps_data.heading);
                } else {
                    ESP_LOGI(TAG, "GPS: NO FIX | Sats: %d", gps_data.satellites);
                }
                gps_log_counter = 0;
            }
            
        } else {
            consecutive_failures++;
            health_record_failure(&g_system_health.gps_sensor);

            // Log failures but don't spam
            if (consecutive_failures == 1) {
                ESP_LOGW(TAG, "GPS read failed: %s", esp_err_to_name(gps_err));
            } else if (consecutive_failures % 30 == 0) { // Every 30 failures
                ESP_LOGE(TAG, "GPS has failed %lu consecutive times. Check hardware!", consecutive_failures);
                health_report_component(&g_system_health.gps_sensor, "GPS_TASK");

                // Optionally run a debug session
                ESP_LOGI(TAG, "Running GPS debug...");
                gps_debug_raw_data();
            }
        }
        
        // Standard GPS update rate
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_PERIOD_MS));
    }
}