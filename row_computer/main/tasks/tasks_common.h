#ifndef TASKS_COMMON_H
#define TASKS_COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sensors/sensors_common.h"
#include "config/timing_config.h"
#include "utils/timing.h"

// =============================================================================
// TASK FUNCTION DECLARATIONS
// =============================================================================

void imu_task(void *parameters);
void gps_task(void *parameters);
void logging_task(void *parameters);
void health_monitor_task(void *parameters);  // New: system health monitoring

// =============================================================================
// TASK MANAGEMENT FUNCTIONS
// =============================================================================

/**
 * @brief Create all sensor tasks with proper priorities and timing
 * @return ESP_OK if all tasks created successfully
 */
esp_err_t create_tasks(void);

/**
 * @brief Create inter-task communication queues with proper sizing
 * @return ESP_OK if all queues created successfully  
 */
esp_err_t create_inter_task_comm(void);

/**
 * @brief Initialize system health monitoring
 * @return ESP_OK if health system initialized successfully
 */
esp_err_t init_system_health(void);

// =============================================================================
// GLOBAL TASK HANDLES (extern declarations for use in other files)
// =============================================================================

extern TaskHandle_t imu_task_handle;
extern TaskHandle_t gps_task_handle;
extern TaskHandle_t logging_task_handle;
extern TaskHandle_t health_monitor_task_handle;

// =============================================================================
// GLOBAL QUEUE HANDLES (extern declarations for use in other files)
// =============================================================================

extern QueueHandle_t imu_data_queue;
extern QueueHandle_t gps_data_queue;
extern QueueHandle_t output_data_queue;  // For future processed data

// =============================================================================
// GLOBAL HEALTH MONITORING (extern declarations for use in other files)
// =============================================================================

extern system_health_t global_system_health;

// =============================================================================
// QUEUE HELPER MACROS
// =============================================================================

// Convenient macros for sending to queues with overflow handling
#define SEND_IMU_DATA(data) \
    send_to_queue_with_overflow(imu_data_queue, &(data), sizeof(imu_data_t), "IMU")

#define SEND_GPS_DATA(data) \
    send_to_queue_with_overflow(gps_data_queue, &(data), sizeof(gps_data_t), "GPS")

#define SEND_OUTPUT_DATA(data) \
    send_to_queue_with_overflow(output_data_queue, &(data), sizeof(output_data_t), "OUTPUT")

// =============================================================================
// TIMING HELPER MACROS
// =============================================================================

// Create delay until next task iteration
#define TASK_DELAY_UNTIL(last_wake_time, rate_ms) \
    vTaskDelayUntil(&(last_wake_time), pdMS_TO_TICKS(rate_ms))

// Log with rate limiting for high-frequency tasks
#define LOG_IMU_RATE_LIMITED(level, format, ...) \
    do { \
        if (should_log_imu_high_freq()) { \
            ESP_LOG##level("IMU_TASK", format, ##__VA_ARGS__); \
        } \
    } while(0)

#define LOG_GENERAL_RATE_LIMITED(tag, level, format, ...) \
    do { \
        if (should_log_general_high_freq()) { \
            ESP_LOG##level(tag, format, ##__VA_ARGS__); \
        } \
    } while(0)

#endif