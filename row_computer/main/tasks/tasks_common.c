#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"
#include "utils/error_utils.h"

static const char *TAG = "TASKS_COMMON";

// Create inter-task communication queues
esp_err_t create_inter_task_comm(void){
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_data_t));
    gps_data_queue = xQueueCreate(GPS_QUEUE_SIZE, sizeof(gps_data_t));

    if (imu_data_queue == NULL || gps_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create communication queues");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Communication queues created (IMU:%d, GPS:%d)", IMU_QUEUE_SIZE, GPS_QUEUE_SIZE);
    return ESP_OK;
}

// Create sensor tasks with appropriate priorities
esp_err_t create_tasks(void){
    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        imu_task,
        "IMU_TASK",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        &imu_task_handle,
        1                   // Pin to core 1 (app core)
    );
    CHECK_AND_LOG_ERROR(result != pdPASS ? ESP_FAIL : ESP_OK, TAG, "Failed to create IMU task");

    result = xTaskCreatePinnedToCore(
        gps_task,
        "GPS_TASK",
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,
        &gps_task_handle,
        1                   // Pin to core 1
    );
    CHECK_AND_LOG_ERROR(result != pdPASS ? ESP_FAIL : ESP_OK, TAG, "Failed to create GPS task");

    result = xTaskCreatePinnedToCore(
        logging_task,
        "LOG_TASK",
        LOG_TASK_STACK_SIZE,
        NULL,
        LOG_TASK_PRIORITY,
        &logging_task_handle,
        0                   // Pin to core 0 (protocol core)
    );
    CHECK_AND_LOG_ERROR(result != pdPASS ? ESP_FAIL : ESP_OK, TAG, "Failed to create logging task");

    ESP_LOGI(TAG, "All tasks created and running");
    ESP_LOGI(TAG, "=== System Ready for Rowing ===");
    return ESP_OK;
}