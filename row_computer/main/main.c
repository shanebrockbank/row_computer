// main/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "config/pin_definitions.h"
#include "sensors/sensors_common.h"
#include "tasks/tasks_common.h"
#include "utils/protocol_init.h"

static const char *TAG = "MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");
    
    // Initialize hardware protocols
    ESP_LOGI(TAG, "Initializing communication protocols...");
    if (protocols_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocols - system halt");
        return;
    }
    
    // Initialize sensors
    ESP_LOGI(TAG, "Initializing sensors...");
    if (accel_init() != ESP_OK) ESP_LOGW(TAG, "Accelerometer init failed");
    if (gyro_init() != ESP_OK) ESP_LOGW(TAG, "Gyroscope init failed"); 
    if (mag_init() != ESP_OK) ESP_LOGW(TAG, "Magnetometer init failed");
    if (gps_init() != ESP_OK) ESP_LOGW(TAG, "GPS init failed");
    
    create_inter_task_comm();
    
    // Create sensor tasks with appropriate priorities
    esp_err_t create_tasks();
    
    // Main task becomes system monitor
    while (1) {
        // Monitor system health
        ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());
        
        // // Check task health (optional)
        // if (eTaskGetState(imu_task_handle) == eSuspended) {
        //     ESP_LOGE(TAG, "IMU task has stopped!");
        // }
        
        vTaskDelay(pdMS_TO_TICKS(10000)); // Report every 10 seconds
    }
}