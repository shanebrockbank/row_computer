#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors_common.h"
#include "utils/error_utils.h"
#include "utils/boot_progress.h"

static const char *TAG = "TASKS_COMMON";

// Create inter-task communication queues
esp_err_t create_inter_task_comm(void){
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_data_t));
    gps_data_queue = xQueueCreate(GPS_QUEUE_SIZE, sizeof(gps_data_t));

    if (imu_data_queue == NULL) {
        boot_progress_failure(BOOT_QUEUES, "IMU queue", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_QUEUES, "IMU queue");
    }

    if (gps_data_queue == NULL) {
        boot_progress_failure(BOOT_QUEUES, "GPS queue", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_QUEUES, "GPS queue");
    }

    boot_progress_report_category(BOOT_QUEUES, "QUEUES");
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
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "IMU task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "IMU task");
    }

    result = xTaskCreatePinnedToCore(
        gps_task,
        "GPS_TASK",
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,
        &gps_task_handle,
        1                   // Pin to core 1
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "GPS task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "GPS task");
    }

    result = xTaskCreatePinnedToCore(
        logging_task,
        "LOG_TASK",
        LOG_TASK_STACK_SIZE,
        NULL,
        LOG_TASK_PRIORITY,
        &logging_task_handle,
        0                   // Pin to core 0 (protocol core)
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "LOG task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "LOG task");
    }

    boot_progress_report_category(BOOT_TASKS, "TASKS");
    return ESP_OK;
}