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
    // Legacy queues (kept for compatibility during transition)
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_data_t));
    gps_data_queue = xQueueCreate(GPS_DATA_QUEUE_SIZE, sizeof(gps_data_t));  // Updated to 3-buffer for ultra-responsive

    // New ultra-responsive queues
    raw_imu_data_queue = xQueueCreate(RAW_IMU_QUEUE_SIZE, sizeof(imu_data_t));
    processed_imu_data_queue = xQueueCreate(PROCESSED_IMU_QUEUE_SIZE, sizeof(processed_imu_data_t));
    motion_state_queue = xQueueCreate(MOTION_STATE_QUEUE_SIZE, sizeof(motion_state_t));

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

    // Check new ultra-responsive queues
    if (raw_imu_data_queue == NULL) {
        boot_progress_failure(BOOT_QUEUES, "Raw IMU queue", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_QUEUES, "Raw IMU queue");
    }

    if (processed_imu_data_queue == NULL) {
        boot_progress_failure(BOOT_QUEUES, "Processed IMU queue", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_QUEUES, "Processed IMU queue");
    }

    if (motion_state_queue == NULL) {
        boot_progress_failure(BOOT_QUEUES, "Motion state queue", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_QUEUES, "Motion state queue");
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
        processing_task,
        "LOG_TASK",
        LOG_TASK_STACK_SIZE,
        NULL,
        LOG_TASK_PRIORITY,
        &processing_task_handle,
        0                   // Pin to core 0 (protocol core)
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "LOG task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "LOG task");
    }

    // Create new ultra-responsive processing tasks
    result = xTaskCreatePinnedToCore(
        calibration_filter_task,
        "CALIBRATION_TASK",
        CALIBRATION_TASK_STACK_SIZE,
        NULL,
        CALIBRATION_TASK_PRIORITY,
        &calibration_filter_task_handle,
        1                   // Pin to core 1 (app core) with other processing
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "Calibration task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "Calibration task");
    }

    result = xTaskCreatePinnedToCore(
        motion_fusion_task,
        "MOTION_FUSION_TASK",
        MOTION_FUSION_STACK_SIZE,
        NULL,
        MOTION_FUSION_PRIORITY,
        &motion_fusion_task_handle,
        1                   // Pin to core 1 (app core)
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "Motion fusion task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "Motion fusion task");
    }

    result = xTaskCreatePinnedToCore(
        display_task,
        "DISPLAY_TASK",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &display_task_handle,
        1                   // Pin to core 1 (app core)
    );
    if (result != pdPASS) {
        boot_progress_failure(BOOT_TASKS, "Display task", "Creation failed");
        return ESP_FAIL;
    } else {
        boot_progress_success(BOOT_TASKS, "Display task");
    }

    boot_progress_report_category(BOOT_TASKS, "TASKS");
    return ESP_OK;
}