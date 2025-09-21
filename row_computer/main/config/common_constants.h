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
#define LOG_TASK_PERIOD_MS          50      // 20Hz (legacy processing task)
#define SYSTEM_MONITOR_PERIOD_MS    10000   // 0.1Hz

// New ultra-responsive task periods
#define CALIBRATION_TASK_PERIOD_MS  10      // 100Hz - matches IMU rate
#define MOTION_FUSION_PERIOD_MS     20      // 50Hz - high-frequency fusion
#define DISPLAY_TASK_PERIOD_MS      100     // 10Hz - smooth UI updates
#define SD_LOGGING_PERIOD_MS        1000    // 1Hz - efficient batch logging

// Task priorities
#define IMU_TASK_PRIORITY           6       // High priority - time critical
#define GPS_TASK_PRIORITY           4       // Medium priority
#define LOG_TASK_PRIORITY           2       // Low priority - non-time critical (legacy)

// New ultra-responsive task priorities
#define CALIBRATION_TASK_PRIORITY   7       // Highest - processes IMU immediately
#define MOTION_FUSION_PRIORITY      5       // High - sensor fusion critical
#define DISPLAY_TASK_PRIORITY       3       // Medium - user interface
#define SD_LOGGING_PRIORITY         1       // Lowest - batch storage

// Task stack sizes
#define IMU_TASK_STACK_SIZE         4096
#define GPS_TASK_STACK_SIZE         4096
#define LOG_TASK_STACK_SIZE         8192    // Larger for data processing (legacy)

// New task stack sizes
#define CALIBRATION_TASK_STACK_SIZE 6144    // Extra space for filtering algorithms
#define MOTION_FUSION_STACK_SIZE    8192    // Large for sensor fusion calculations
#define DISPLAY_TASK_STACK_SIZE     4096    // Standard for UI operations
#define SD_LOGGING_STACK_SIZE       6144    // Extra space for file I/O buffering

// Queue configurations (legacy)
#define IMU_QUEUE_SIZE              50      // Buffer 0.5 seconds at 100Hz (legacy)
#define GPS_QUEUE_SIZE              10      // Buffer 10 GPS fixes (legacy)

// Ultra-responsive queue configurations (<200ms total latency)
#define RAW_IMU_QUEUE_SIZE          15      // 0.15s buffer at 100Hz (IMU → Calibration)
#define PROCESSED_IMU_QUEUE_SIZE    10      // 0.1s buffer at 100Hz (Calibration → Fusion)
#define MOTION_STATE_QUEUE_SIZE     30      // 0.6s buffer at 50Hz (Fusion → Display/Logging)
#define GPS_DATA_QUEUE_SIZE         3       // 3s buffer at 1Hz (GPS → Fusion)

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