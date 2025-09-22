// main/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "config/pin_definitions.h"
#include "config/common_constants.h"
#include "sensors/sensors_common.h"
#include "sensors/mpu6050.h"
#include "tasks/tasks_common.h"
#include "utils/protocol_init.h"
#include "utils/boot_progress.h"
#include "utils/health_monitor.h"

static const char *TAG = "MAIN";

// Define the global variables that are declared extern in tasks_common.h
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;

// New ultra-responsive task handles
TaskHandle_t calibration_filter_task_handle = NULL;
TaskHandle_t motion_fusion_task_handle = NULL;
TaskHandle_t display_task_handle = NULL;

QueueHandle_t gps_data_queue = NULL;

// New ultra-responsive queue handles
QueueHandle_t raw_imu_data_queue = NULL;
QueueHandle_t processed_imu_data_queue = NULL;
QueueHandle_t motion_state_queue = NULL;

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");

    // Initialize boot progress tracking
    boot_progress_init();

    // Initialize unified health monitoring system
    health_stats_init(&g_system_health.imu_sensor, "MPU6050");
    health_stats_init(&g_system_health.mag_sensor, "HMC5883L");
    health_stats_init(&g_system_health.gps_sensor, "GPS");
    health_stats_init(&g_system_health.calibration_task, "Calibration");
    health_stats_init(&g_system_health.motion_fusion_task, "Motion Fusion");
    health_stats_init(&g_system_health.display_task, "Display");

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
    ESP_LOGD(TAG, "Initializing sensors...");
    if (mpu6050_init() != ESP_OK) {
        boot_progress_failure(BOOT_SENSORS, "MPU6050", "Init failed");
    } else {
        // Quick validation test - read sensor data to verify it's working
        mpu6050_data_t test_data;
        if (mpu6050_read_all(&test_data) == ESP_OK) {
            // Check if we get realistic values (not all zeros)
            float total_accel = sqrt(test_data.accel_x * test_data.accel_x +
                                   test_data.accel_y * test_data.accel_y +
                                   test_data.accel_z * test_data.accel_z);
            if (total_accel > 0.5f && total_accel < 2.0f) { // Reasonable range for gravity ±noise
                boot_progress_success(BOOT_SENSORS, "MPU6050");
                ESP_LOGD(TAG, "MPU6050 data validation: %.2fg total acceleration", total_accel);
            } else {
                boot_progress_failure(BOOT_SENSORS, "MPU6050", "Invalid data");
                ESP_LOGW(TAG, "MPU6050 reads unusual values: %.2fg total acceleration", total_accel);
            }
        } else {
            boot_progress_failure(BOOT_SENSORS, "MPU6050", "Read test failed");
        }
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
        vTaskDelay(pdMS_TO_TICKS(SYSTEM_MONITOR_REPORT_INTERVAL_MS)); // Report every 5 seconds
    }
}