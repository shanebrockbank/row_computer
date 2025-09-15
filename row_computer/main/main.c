// main/main.c - Enhanced with comprehensive timing system
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"

// Project includes
#include "config/pin_definitions.h"
#include "config/timing_config.h"
#include "sensors/sensors_common.h"
#include "tasks/tasks_common.h"
#include "utils/protocol_init.h"
#include "utils/timing.h"

static const char *TAG = "MAIN";

// Define the global variables that are declared extern in tasks_common.h
// (These are defined here to avoid multiple definition errors)

void app_main(void) {
    ESP_LOGI(TAG, "=== Rowing Computer Starting ===");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap at startup: %lu bytes (%.1f KB)", 
             esp_get_free_heap_size(), esp_get_free_heap_size() / 1024.0f);
    
    // =============================================================================
    // PHASE 1: TIMING SYSTEM INITIALIZATION
    // =============================================================================
    
    ESP_LOGI(TAG, "Initializing timing system...");
    
    // Set initial log levels based on configuration
    esp_log_level_set("IMU_TASK", LOG_LEVEL_IMU);
    esp_log_level_set("GPS_TASK", LOG_LEVEL_GPS);
    esp_log_level_set("LOG_TASK", ESP_LOG_INFO);
    esp_log_level_set("HEALTH_TASK", ESP_LOG_INFO);
    esp_log_level_set("TIMING", LOG_LEVEL_TIMING);
    
    // Print initial timing configuration
    print_system_timing_status();
    
    // =============================================================================
    // PHASE 2: HARDWARE PROTOCOLS INITIALIZATION
    // =============================================================================
    
    ESP_LOGI(TAG, "Initializing communication protocols...");
    if (protocols_init() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize protocols - system halt");
        return;
    }
    ESP_LOGI(TAG, "✅ Communication protocols initialized");
    
    // =============================================================================
    // PHASE 3: SENSOR INITIALIZATION
    // =============================================================================
    
    ESP_LOGI(TAG, "Initializing sensors...");
    
    // Initialize system health monitoring
    if (init_system_health() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize health monitoring");
        return;
    }
    
    // Initialize sensors with health tracking
    bool critical_sensor_ok = true;
    
    if (accel_init() != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  Accelerometer init failed - CRITICAL SENSOR");
        critical_sensor_ok = false;
    } else {
        ESP_LOGI(TAG, "✅ Accelerometer initialized");
        global_system_health.accel_status.sensor_available = true;
    }
    
    if (gyro_init() != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  Gyroscope init failed");
    } else {
        ESP_LOGI(TAG, "✅ Gyroscope initialized");
        global_system_health.gyro_status.sensor_available = true;
    }
    
    if (mag_init() != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  Magnetometer init failed");
    } else {
        ESP_LOGI(TAG, "✅ Magnetometer initialized");
        global_system_health.mag_status.sensor_available = true;
    }
    
    if (gps_init() != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  GPS init failed");
    } else {
        ESP_LOGI(TAG, "✅ GPS initialized");
        global_system_health.gps_status.sensor_available = true;
    }
    
    // Check if we can continue without critical sensors
    if (!critical_sensor_ok) {
        ESP_LOGE(TAG, "❌ Critical sensor (accelerometer) failed - cannot detect rowing strokes");
        ESP_LOGE(TAG, "   System will continue but stroke detection will be unavailable");
        ESP_LOGE(TAG, "   Check wiring and restart system");
    }
    
    // =============================================================================
    // PHASE 4: TASK SYSTEM INITIALIZATION
    // =============================================================================
    
    ESP_LOGI(TAG, "Creating task communication system...");
    
    // Create communication queues with overflow handling
    if (create_inter_task_comm() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to create communication queues");
        return;
    }
    ESP_LOGI(TAG, "✅ Inter-task communication created");
    
    // Create sensor tasks with appropriate priorities and timing
    if (create_tasks() != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to create sensor tasks");
        return;
    }
    ESP_LOGI(TAG, "✅ All sensor tasks created and running");
    
    // =============================================================================
    // PHASE 5: MAIN SYSTEM MONITORING LOOP
    // =============================================================================
    
    ESP_LOGI(TAG, "🚣 === ROWING COMPUTER READY ===");
    ESP_LOGI(TAG, "System startup completed in %lu ms", get_system_time_ms());
    
    // Log initial system status
    log_system_health(&global_system_health);
    
    // Main task becomes system monitor with timing discipline
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t monitor_counter = 0;
    
    while (1) {
        monitor_counter++;
        uint32_t current_time = get_system_time_ms();
        uint32_t free_heap = esp_get_free_heap_size();
        
        // System health check every 30 seconds
        if (monitor_counter % 6 == 0) { // Every 6 * 5 seconds = 30 seconds
            ESP_LOGI(TAG, "=== System Monitor Report #%lu ===", monitor_counter);
            ESP_LOGI(TAG, "Uptime: %lu ms (%.1f minutes)", 
                    current_time, current_time / 60000.0f);
            ESP_LOGI(TAG, "Free heap: %lu bytes (%.1f KB)", free_heap, free_heap / 1024.0f);
            
            // Check for memory leaks
            static uint32_t last_free_heap = 0;
            if (last_free_heap > 0) {
                int32_t heap_change = (int32_t)free_heap - (int32_t)last_free_heap;
                if (heap_change < -1000) {
                    ESP_LOGW(TAG, "Potential memory leak: heap decreased by %ld bytes", -heap_change);
                }
            }
            last_free_heap = free_heap;
            
            // Log queue health
            log_queue_health(imu_data_queue, "IMU", IMU_QUEUE_SIZE, 100);
            log_queue_health(gps_data_queue, "GPS", GPS_QUEUE_SIZE, 1);
        }
        
        // Check for low memory condition
        if (free_heap < 30000) { // Less than 30KB
            ESP_LOGE(TAG, "❌ CRITICAL: Low memory - only %lu bytes free", free_heap);
            ESP_LOGE(TAG, "   System may become unstable - consider reducing queue sizes");
        }
        
        // Simple command interface check (could be expanded for serial commands)
        // For now, just demonstrate verbose mode toggle capability
        if (monitor_counter == 12) { // After 1 minute
            ESP_LOGI(TAG, "Demonstrating verbose mode toggle...");
            toggle_verbose_mode();
        } else if (monitor_counter == 24) { // After 2 minutes  
            ESP_LOGI(TAG, "Returning to operational mode...");
            toggle_verbose_mode();
        }
        
        // Maintain exact 5 second timing for monitoring
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));
    }
}