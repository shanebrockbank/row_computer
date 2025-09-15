#ifndef TIMING_H
#define TIMING_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CORE TIMING STRUCTURES
// =============================================================================

typedef struct {
    uint32_t timestamp_ms;      // Milliseconds since system boot
    bool data_valid;            // Flag indicating if sensor read was successful
} sensor_timestamp_t;

typedef struct {
    bool sensor_available;                  // Is sensor currently working?
    uint32_t last_successful_read_ms;       // Timestamp of last successful read
    uint32_t consecutive_failures;          // Count of consecutive failed reads
    uint32_t total_error_count;             // Total errors since boot
} sensor_status_t;

// =============================================================================
// CORE TIMING FUNCTIONS
// =============================================================================

/**
 * @brief Get current system time in milliseconds since boot
 * @return Time in milliseconds (1ms resolution)
 */
uint32_t get_system_time_ms(void);

/**
 * @brief Create a timestamp structure
 * @param timestamp Pointer to timestamp structure to fill
 * @param data_valid Whether the associated data is valid
 */
void create_timestamp(sensor_timestamp_t *timestamp, bool data_valid);

// =============================================================================
// RATE LIMITING FOR LOGS
// =============================================================================

/**
 * @brief Check if IMU high-frequency log should be output (respects verbose mode)
 * @return true if should log, false if suppressed
 */
bool should_log_imu_high_freq(void);

/**
 * @brief Check if general high-frequency log should be output (respects verbose mode)
 * @return true if should log, false if suppressed
 */
bool should_log_general_high_freq(void);

// =============================================================================
// QUEUE MANAGEMENT WITH OVERFLOW HANDLING
// =============================================================================

/**
 * @brief Send item to queue with automatic overflow handling
 * @param queue Queue handle
 * @param item Pointer to item to send
 * @param item_size Size of item in bytes (for overflow handling)
 * @param queue_name Name for logging purposes
 * @return true if sent successfully, false if failed
 */
bool send_to_queue_with_overflow(QueueHandle_t queue, const void *item, 
                                size_t item_size, const char *queue_name);

/**
 * @brief Log queue health statistics
 * @param queue Queue handle
 * @param queue_name Name for logging
 * @param max_size Maximum queue size
 * @param sample_rate_hz Sample rate for buffer time calculation
 */
void log_queue_health(QueueHandle_t queue, const char *queue_name, 
                     uint32_t max_size, uint32_t sample_rate_hz);

// =============================================================================
// SENSOR HEALTH MONITORING
// =============================================================================

/**
 * @brief Initialize sensor status structure
 * @param status Pointer to sensor status structure
 */
void init_sensor_status(sensor_status_t *status);

/**
 * @brief Update sensor status based on read result
 * @param status Pointer to sensor status structure
 * @param read_success Whether the sensor read was successful
 */
void update_sensor_status(sensor_status_t *status, bool read_success);

/**
 * @brief Check if sensor needs a reset attempt
 * @param status Pointer to sensor status structure
 * @return true if sensor should be reset
 */
bool sensor_needs_reset(const sensor_status_t *status);

/**
 * @brief Log sensor health status
 * @param status Pointer to sensor status structure
 * @param sensor_name Name of sensor for logging
 */
void log_sensor_health(const sensor_status_t *status, const char *sensor_name);

// =============================================================================
// SYSTEM STATUS REPORTING
// =============================================================================

/**
 * @brief Print comprehensive system timing status
 */
void print_system_timing_status(void);

/**
 * @brief Toggle verbose mode on/off
 */
void toggle_verbose_mode(void);

// =============================================================================
// EXTERNAL VARIABLES
// =============================================================================

extern bool verbose_mode;  // Runtime verbose mode toggle

#endif // TIMING_H