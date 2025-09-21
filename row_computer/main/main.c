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
#include "sensors/mpu6050.h"
#include "tasks/tasks_common.h"
#include "utils/protocol_init.h"
#include "utils/boot_progress.h"

static const char *TAG = "MAIN";

// Define the global variables that are declared extern in tasks_common.h
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t processing_task_handle = NULL;

// New ultra-responsive task handles
TaskHandle_t calibration_filter_task_handle = NULL;
TaskHandle_t motion_fusion_task_handle = NULL;
TaskHandle_t display_task_handle = NULL;

QueueHandle_t imu_data_queue = NULL;
QueueHandle_t gps_data_queue = NULL;

// New ultra-responsive queue handles
QueueHandle_t raw_imu_data_queue = NULL;
QueueHandle_t processed_imu_data_queue = NULL;
QueueHandle_t motion_state_queue = NULL;

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");

    // Initialize boot progress tracking
    boot_progress_init();

    // Set log levels for condensed boot experience
    // Reduce verbosity on subsystems to only show warnings/errors by default
    esp_log_level_set("GPS", ESP_LOG_WARN);
    esp_log_level_set("PROTOCOLS", ESP_LOG_WARN);
    esp_log_level_set("MPU6050", ESP_LOG_INFO);  // 🔍 ENABLE MPU6050 DEBUGGING
    esp_log_level_set("MAG", ESP_LOG_WARN);
    esp_log_level_set("LOG_TASK", ESP_LOG_WARN);

    // Keep sensor health reporting visible (INFO level)
    esp_log_level_set("GPS_TASK", ESP_LOG_INFO);  // For GPS health reports
    esp_log_level_set("IMU_TASK", ESP_LOG_INFO);  // For IMU health reports

    // Enable debug logging only for troubleshooting - uncomment as needed:
    // esp_log_level_set("GPS", ESP_LOG_DEBUG);
    // esp_log_level_set("GPS_TASK", ESP_LOG_DEBUG);
    // esp_log_level_set("PROTOCOLS", ESP_LOG_DEBUG);
    // esp_log_level_set("BOOT", ESP_LOG_DEBUG);

    // Initialize hardware protocols
    if (protocols_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize protocols - system halt");
        return;
    }

    // Initialize sensors
    ESP_LOGI(TAG, "=== SENSOR INITIALIZATION PHASE ===");
    ESP_LOGI(TAG, "Initializing MPU6050 (accelerometer + gyroscope)...");
    esp_err_t mpu_result = mpu6050_init();
    if (mpu_result != ESP_OK) {
        ESP_LOGE(TAG, "❌ MPU6050 initialization failed: %s", esp_err_to_name(mpu_result));
        boot_progress_failure(BOOT_SENSORS, "MPU6050", "Init failed");
    } else {
        ESP_LOGI(TAG, "✅ MPU6050 initialization successful");
        boot_progress_success(BOOT_SENSORS, "MPU6050");
    }

    if (mag_init() != ESP_OK) {
        boot_progress_failure(BOOT_SENSORS, "Magnetometer", "Init failed");
    } else {
        boot_progress_success(BOOT_SENSORS, "Magnetometer");
    }

    if (gps_init() != ESP_OK) {
        boot_progress_failure(BOOT_SENSORS, "GPS", "Init failed");
    } else {
        boot_progress_success(BOOT_SENSORS, "GPS");
    }

    // Test GPS communication
    ESP_LOGD(TAG, "Testing GPS communication...");
    if (gps_test_communication() == ESP_OK) {
        boot_progress_success(BOOT_SENSORS, "GPS comm test");
    } else {
        boot_progress_failure(BOOT_SENSORS, "GPS comm test", "No response");
        // Run raw debug to see what's happening
        ESP_LOGD(TAG, "Running GPS raw data debug...");
        gps_debug_raw_data();
    }

    // Report sensor summary
    boot_progress_report_category(BOOT_SENSORS, "SENSORS");

    // Create communication queues
    create_inter_task_comm();

    // Create sensor tasks with appropriate priorities
    create_tasks();

    // Final boot summary
    boot_progress_report_final();

    // Main task becomes system monitor
    while (1) {
        // Monitor system health
        ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000)); // Report every 5 seconds
    }
}