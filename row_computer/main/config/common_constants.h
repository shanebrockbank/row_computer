#ifndef COMMON_CONSTANTS_H
#define COMMON_CONSTANTS_H

// Common timing constants (in milliseconds)
#define TIMEOUT_I2C_MS              1000
#define TIMEOUT_GPS_MS              500
#define TIMEOUT_UART_MS             100
#define TIMEOUT_QUEUE_MS            100

// Task timing constants
#define IMU_TASK_PERIOD_MS          10      // 100Hz
#define GPS_TASK_PERIOD_MS          1000    // 1Hz
#define LOG_TASK_PERIOD_MS          50      // 20Hz
#define SYSTEM_MONITOR_PERIOD_MS    10000   // 0.1Hz

// Task priorities
#define IMU_TASK_PRIORITY           6       // High priority - time critical
#define GPS_TASK_PRIORITY           4       // Medium priority
#define LOG_TASK_PRIORITY           2       // Low priority - non-time critical

// Task stack sizes
#define IMU_TASK_STACK_SIZE         4096
#define GPS_TASK_STACK_SIZE         4096
#define LOG_TASK_STACK_SIZE         8192    // Larger for data processing

// Queue configurations
#define IMU_QUEUE_SIZE              50      // Buffer 0.5 seconds at 100Hz
#define GPS_QUEUE_SIZE              10      // Buffer 10 GPS fixes

// Sensor thresholds and constants
#define STROKE_DETECTION_THRESHOLD  2.0f    // 2g acceleration threshold
#define GPS_STARTUP_DELAY_MS        2000    // GPS module settling time
#define SENSOR_HEALTH_LOG_INTERVAL  1000    // Log sensor health every N samples
#define GPS_LOG_INTERVAL            10      // Log GPS status every N reads

// Communication and protocol constants
#define GPS_DATA_TIMEOUT_MS         5000    // No GPS data warning threshold
#define GPS_DEBUG_LOG_INTERVAL      50      // Log GPS debug data every N calls
#define SENSOR_STABILIZE_DELAY_MS   100     // Sensor stabilization delay
#define I2C_SCAN_TIMEOUT_MS         50      // I2C device scan timeout
#define GPS_COMMUNICATION_TEST_TIMEOUT_MS  2000  // GPS communication test timeout

#endif // COMMON_CONSTANTS_H