#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "sensors_common.h"

static const char *TAG = "SENSORS_COMMON"; 

// Task handles for monitoring/control
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t logging_task_handle = NULL;

// Communication queues
QueueHandle_t imu_data_queue = NULL;
QueueHandle_t gps_data_queue = NULL;

// Create inter-task communication queues
esp_err_t create_inter_task_comm(void){
    imu_data_queue = xQueueCreate(50, sizeof(imu_data_t));   // Buffer 0.5 seconds at 100Hz
    gps_data_queue = xQueueCreate(10, sizeof(gps_data_t)); // Buffer 10 GPS fixes

    if (imu_data_queue == NULL || gps_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create communication queues");
        return;
    }
    ESP_LOGI(TAG, "Communication queues created");
}

// Create sensor tasks with appropriate priorities
esp_err_t create_tasks(void){


    xTaskCreatePinnedToCore(
        imu_task,           // Task function
        "IMU_TASK",         // Task name
        4096,               // Stack size
        NULL,               // Parameters
        6,                  // Priority (high - time critical)
        &imu_task_handle,   // Task handle
        1                   // Pin to core 1 (app core)
    );
    
    xTaskCreatePinnedToCore(
        gps_task,
        "GPS_TASK", 
        4096,
        NULL,
        4,                  // Priority (medium)
        &gps_task_handle,
        1                   // Pin to core 1
    );
    
    xTaskCreatePinnedToCore(
        logging_task,
        "LOG_TASK",
        8192,               // Larger stack for data processing
        NULL, 
        2,                  // Priority (low - non-time critical)
        &logging_task_handle,
        0                   // Pin to core 0 (protocol core)
    );
    
    ESP_LOGI(TAG, "All tasks created and running");
    ESP_LOGI(TAG, "=== System Ready for Rowing ===");

}
