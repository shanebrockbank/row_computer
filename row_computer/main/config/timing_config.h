#ifndef TIMING_CONFIG_H
#define TIMING_CONFIG_H

#include "freertos/FreeRTOS.h"

// =============================================================================
// TIMING CONSTANTS
// =============================================================================

// Task execution rates (in milliseconds)
#define IMU_TASK_RATE_MS            10      // 100 Hz
#define GPS_TASK_RATE_MS            1000    // 1 Hz  
#define LOG_TASK_RATE_MS            50      // 20 Hz
#define HEALTH_MONITOR_RATE_MS      5000    // Every 5 seconds

// Task priorities (higher number = higher priority)
#define IMU_TASK_PRIORITY           6       // High (time-critical)
#define GPS_TASK_PRIORITY           4       // Medium
#define LOG_TASK_PRIORITY           2       // Low (processing)

// Task stack sizes (in bytes)
#define IMU_TASK_STACK_SIZE         4096
#define GPS_TASK_STACK_SIZE         4096
#define LOG_TASK_STACK_SIZE         8192    // Larger for data processing

// =============================================================================
// QUEUE CONFIGURATION
// =============================================================================

// Queue buffer times (in seconds)
#define IMU_BUFFER_TIME_S           2.5f
#define GPS_BUFFER_TIME_S           16.0f
#define OUTPUT_BUFFER_TIME_S        5.0f

// Queue sizes (power of 2 for efficiency)
// IMU: 100 Hz × 2.5s = 250 → rounded to 256
#define IMU_QUEUE_SIZE              256
// GPS: 1 Hz × 16s = 16 (already power of 2)
#define GPS_QUEUE_SIZE              16
// Output: ~20 Hz × 5s = 100 → rounded to 128
#define OUTPUT_QUEUE_SIZE           128

// Queue wait times (in ticks)
#define QUEUE_SEND_TIMEOUT_MS       0       // Non-blocking send (drop if full)
#define QUEUE_RECEIVE_TIMEOUT_MS    0       // Non-blocking receive

// =============================================================================
// DATA STRUCTURE SIZES (for memory calculations)
// =============================================================================

#define IMU_DATA_SIZE_BYTES         36      // Estimated size of imu_data_t
#define GPS_DATA_SIZE_BYTES         64      // Estimated size of gps_data_t
#define OUTPUT_DATA_SIZE_BYTES      64      // Estimated size of output data

// Memory usage estimates:
// IMU Queue:    256 × 36 = 9,216 bytes  (~9.2 KB)
// GPS Queue:    16 × 64  = 1,024 bytes  (~1.0 KB)
// Output Queue: 128 × 64 = 8,192 bytes  (~8.2 KB)
// Total:                  ~18.4 KB

// =============================================================================
// ERROR HANDLING CONSTANTS
// =============================================================================

// Failure thresholds
#define MAX_CONSECUTIVE_FAILURES    10      // Before attempting sensor reset
#define SENSOR_TIMEOUT_MS           5000    // Mark sensor as stale after this time
#define HEALTH_LOG_INTERVAL_COUNT   (HEALTH_MONITOR_RATE_MS / IMU_TASK_RATE_MS) // ~500 samples

// =============================================================================
// LOGGING CONFIGURATION
// =============================================================================

// Default log levels (can be overridden at compile time)
#ifndef LOG_LEVEL_IMU
#define LOG_LEVEL_IMU               ESP_LOG_INFO
#endif

#ifndef LOG_LEVEL_GPS
#define LOG_LEVEL_GPS               ESP_LOG_INFO
#endif

#ifndef LOG_LEVEL_TIMING
#define LOG_LEVEL_TIMING            ESP_LOG_INFO
#endif

// Runtime verbose mode toggle
extern bool verbose_mode;

// Rate limiting for high-frequency logs (prevent serial flooding)
#define HIGH_FREQ_LOG_LIMIT_HZ      10      // Max log rate in operational mode

#endif // TIMING_CONFIG_H