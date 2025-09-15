#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/tasks_common.h"
#include "config/pin_definitions.h"
#include "config/timing_config.h"
#include "sensors/sensors_common.h"
#include "utils/timing.h"

static const char *TAG = "TASKS_COMMON"; 

// =============================================================================
// GLOBAL VARIABLE DEFINITIONS (declared extern in header)
// =============================================================================

// Task handles
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t logging_task_handle = NULL;
TaskHandle_t health_monitor_task_handle = NULL;

// Queue handles  
QueueHandle_t imu_data_queue = NULL;
QueueHandle_t gps_data_queue = NULL;
QueueHandle_t output_data_queue = NULL;

// Global system health
system_health_t global_system_health = {0};

// =============================================================================
// INTER-TASK COMMUNICATION SETUP
// =============================================================================

esp_err_t create_inter_task_comm(void) {
    ESP_LOGI(TAG, "Creating inter-task communication queues...");
    
    // Create queues with power-of-2 sizes for efficiency
    imu_data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(imu_data_t));
    gps_data_queue = xQueueCreate(GPS_QUEUE_SIZE, sizeof(gps_data_t));
    output_data_queue = xQueueCreate(OUTPUT_QUEUE_SIZE, sizeof(output_data_t));

    if (imu_data_queue == NULL || gps_data_queue == NULL || output_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create communication queues");
        return ESP_FAIL;
    }
    
    // Log queue memory usage
    uint32_t imu_queue_memory = IMU_QUEUE_SIZE * sizeof(imu_data_t);
    uint32_t gps_queue_memory = GPS_QUEUE_SIZE * sizeof(gps_data_t);
    uint32_t output_queue_memory = OUTPUT_QUEUE_SIZE * sizeof(output_data_t);
    uint32_t total_queue_memory = imu_queue_memory + gps_queue_memory + output_queue_memory;
    
    ESP_LOGI(TAG, "Queue memory allocation:");
    ESP_LOGI(TAG, "  IMU queue:    %u entries × %u bytes = %u bytes (%.1f KB)", 
            IMU_QUEUE_SIZE, sizeof(imu_data_t), imu_queue_memory, imu_queue_memory / 1024.0f);
    ESP_LOGI(TAG, "  GPS queue:    %u entries × %u bytes = %u bytes (%.1f KB)", 
            GPS_QUEUE_SIZE, sizeof(gps_data_t), gps_queue_memory, gps_queue_memory / 1024.0f);
    ESP_LOGI(TAG, "  Output queue: %u entries × %u bytes = %u bytes (%.1f KB)", 
            OUTPUT_QUEUE_SIZE, sizeof(output_data_t), output_queue_memory, output_queue_memory / 1024.0f);
    ESP_LOGI(TAG, "  Total:        %.1f KB", total_queue_memory / 1024.0f);
    
    ESP_LOGI(TAG, "Communication queues created successfully");
    return ESP_OK;
}

// =============================================================================
// SYSTEM HEALTH INITIALIZATION
// =============================================================================

esp_err_t init_system_health(void) {
    ESP_LOGI(TAG, "Initializing system health monitoring...");
    
    // Initialize all sensor status structures
    init_sensor_status(&global_system_health.accel_status);
    init_sensor_status(&global_system_health.gyro_status);
    init_sensor_status(&global_system_health.mag_status);
    init_sensor_status(&global_system_health.gps_status);
    
    global_system_health.last_health_report_ms = get_system_time_ms();
    
    ESP_LOGI(TAG, "System health monitoring initialized");
    return ESP_OK;
}

// =============================================================================
// TASK CREATION WITH ENHANCED TIMING
// =============================================================================

esp_err_t create_tasks(void) {
    ESP_LOGI(TAG, "Creating sensor tasks with timing discipline...");
    
    // Log task configuration
    ESP_LOGI(TAG, "Task configuration:");
    ESP_LOGI(TAG, "  IMU Task:     Priority=%d, Rate=%dms (%.1fHz), Stack=%d bytes", 
            IMU_TASK_PRIORITY, IMU_TASK_RATE_MS, 1000.0f/IMU_TASK_RATE_MS, IMU_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  GPS Task:     Priority=%d, Rate=%dms (%.1fHz), Stack=%d bytes", 
            GPS_TASK_PRIORITY, GPS_TASK_RATE_MS, 1000.0f/GPS_TASK_RATE_MS, GPS_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  Log Task:     Priority=%d, Rate=%dms (%.1fHz), Stack=%d bytes", 
            LOG_TASK_PRIORITY, LOG_TASK_RATE_MS, 1000.0f/LOG_TASK_RATE_MS, LOG_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  Health Task:  Priority=%d, Rate=%dms (%.3fHz), Stack=%d bytes", 
            LOG_TASK_PRIORITY-1, HEALTH_MONITOR_RATE_MS, 1000.0f/HEALTH_MONITOR_RATE_MS, 4096);
    
    // Create high-priority IMU task (time-critical)
    BaseType_t result = xTaskCreatePinnedToCore(
        imu_task,                   // Task function
        "IMU_TASK",                 // Task name
        IMU_TASK_STACK_SIZE,        // Stack size
        NULL,                       // Parameters
        IMU_TASK_PRIORITY,          // Priority (high - time critical)
        &imu_task_handle,           // Task handle
        1                           // Pin to core 1 (app core)
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_FAIL;
    }
    
    // Create medium-priority GPS task
    result = xTaskCreatePinnedToCore(
        gps_task,
        "GPS_TASK", 
        GPS_TASK_STACK_SIZE,
        NULL,
        GPS_TASK_PRIORITY,          // Priority (medium)
        &gps_task_handle,
        1                           // Pin to core 1
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS task");
        return ESP_FAIL;
    }
    
    // Create low-priority logging task (data processing)
    result = xTaskCreatePinnedToCore(
        logging_task,
        "LOG_TASK",
        LOG_TASK_STACK_SIZE,        // Larger stack for data processing
        NULL, 
        LOG_TASK_PRIORITY,          // Priority (low - non-time critical)
        &logging_task_handle,
        0                           // Pin to core 0 (protocol core)
    );
    if (result != pdPASS) {
        ESP_LOGE