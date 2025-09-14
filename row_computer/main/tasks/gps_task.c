#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "sensors_common.h"

// Low-frequency GPS task
void gps_task(void *parameters) {
    ESP_LOGI("GPS_TASK", "Starting GPS task at 1Hz");
    
    gps_data_t gps_data;
    
    while (1) {
        esp_err_t gps_err = gps_read(&gps_data);
        
        if (gps_err == ESP_OK) {
            // Add timestamp
            gps_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue
            if (xQueueSend(gps_data_queue, &gps_data, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGW("GPS_TASK", "Failed to send GPS data to queue");
            }
            
            // Log GPS status every 10 seconds
            static int gps_log_counter = 0;
            if (++gps_log_counter >= 10) {
                if (gps_data.valid_fix) {
                    ESP_LOGI("GPS_TASK", "Position: %.6f°, %.6f° | Speed: %.1f kts | Sats: %d",
                            gps_data.latitude, gps_data.longitude, 
                            gps_data.speed_knots, gps_data.satellites);
                } else {
                    ESP_LOGI("GPS_TASK", "Searching for fix... Satellites: %d", gps_data.satellites);
                }
                gps_log_counter = 0;
            }
        }
        
        // 1Hz update rate
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}