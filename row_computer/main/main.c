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

// Define the global variables that are declared extern in tasks_common.h
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t logging_task_handle = NULL;

QueueHandle_t imu_data_queue = NULL;
QueueHandle_t gps_data_queue = NULL;

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");
    
    // Enable detailed GPS logging
    esp_log_level_set("GPS", ESP_LOG_DEBUG);        // Show all GPS debug messages
    esp_log_level_set("GPS_TASK", ESP_LOG_DEBUG);   // Show GPS task debug messages
    
    // Optional: Enable UART driver logging to see low-level UART issues
    // esp_log_level_set("uart", ESP_LOG_DEBUG);
    
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
    
    // // Optional: Add a manual GPS communication test
    // ESP_LOGI(TAG, "Testing GPS communication...");
    // if (gps_test_communication() == ESP_OK) {
    //     ESP_LOGI(TAG, "✓ GPS module is responding");
    // } else {
    //     ESP_LOGE(TAG, "✗ GPS module not responding - check wiring!");
    //     // Run raw debug to see what's happening
    //     ESP_LOGI(TAG, "Running GPS raw data debug...");
    //     gps_debug_raw_data();
    // }
    // manual_gps_test();
    
    // Create communication queues
    create_inter_task_comm();
    
    // Create sensor tasks with appropriate priorities
    create_tasks();
    
    // Main task becomes system monitor
    while (1) {
        // Monitor system health
        ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(10000)); // Report every 10 seconds
    }
}