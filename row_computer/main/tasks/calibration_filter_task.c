#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tasks/tasks_common.h"
#include "config/common_constants.h"
#include "sensors/sensors_common.h"
#include "utils/timing_utils.h"
#include "utils/queue_utils.h"
#include "utils/health_monitor.h"

static const char *TAG = "CALIBRATION_TASK";

void calibration_filter_task(void *parameters) {
    ESP_LOGI(TAG, "Starting calibration filter task at %dHz", 1000/CALIBRATION_TASK_PERIOD_MS);

    imu_data_t raw_imu_data;
    imu_data_t processed_imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();

    // Calibration and filtering state variables
    static uint32_t sample_count = 0;
    static uint32_t processed_samples = 0;
    static uint32_t dropped_samples = 0;

    // Timing measurement
    static timing_stats_t processing_stats;
    static bool timing_initialized = false;
    if (!timing_initialized) {
        timing_stats_init(&processing_stats);
        timing_initialized = true;
    }

    while (1) {
        // Read from raw IMU queue (non-blocking)
        if (xQueueReceive(raw_imu_data_queue, &raw_imu_data, 0) == pdTRUE) {
            sample_count++;
            uint64_t processing_start = get_timestamp_us();

            // TODO: Implement calibration and filtering algorithms
            // For now, pass through data with basic structure conversion
            processed_imu_data.timestamp_ms = raw_imu_data.timestamp_ms;
            processed_imu_data.accel_x = raw_imu_data.accel_x;
            processed_imu_data.accel_y = raw_imu_data.accel_y;
            processed_imu_data.accel_z = raw_imu_data.accel_z;
            processed_imu_data.gyro_x = raw_imu_data.gyro_x;
            processed_imu_data.gyro_y = raw_imu_data.gyro_y;
            processed_imu_data.gyro_z = raw_imu_data.gyro_z;
            processed_imu_data.mag_x = raw_imu_data.mag_x;
            processed_imu_data.mag_y = raw_imu_data.mag_y;
            processed_imu_data.mag_z = raw_imu_data.mag_z;


            // Send to processed IMU queue with smart overflow management
            if (queue_send_with_overflow(processed_imu_data_queue, &processed_imu_data, sizeof(imu_data_t), TAG, "Processed IMU", processed_imu_data.timestamp_ms) == ESP_OK) {
                processed_samples++;
                health_record_success(&g_system_health.calibration_task);
            } else {
                dropped_samples++;
                health_record_drop(&g_system_health.calibration_task);
            }

            // Measure processing time
            uint64_t processing_end = get_timestamp_us();
            uint32_t processing_time = calc_elapsed_us(processing_start, processing_end);
            timing_stats_update(&processing_stats, processing_time);
        }

        // Only report timing if there are performance issues
        static uint32_t timing_counter = 0;
        if (++timing_counter >= CALIBRATION_TIMING_CHECK_INTERVAL) { // Every 30 seconds
            if (processing_stats.max_latency_us > CALIBRATION_LATENCY_THRESHOLD_US) { // Only if processing takes >5ms
                timing_stats_report(&processing_stats, "CALIBRATION", 0);  // Force report
            }
            timing_counter = 0;
        }

        // Only log calibration health if there are dropped samples
        if (health_should_report(&g_system_health.calibration_task, SENSOR_HEALTH_LOG_INTERVAL * 2)) {
            health_report_component(&g_system_health.calibration_task, "CALIBRATION");
            if (dropped_samples > 0) {
                ESP_LOGW(TAG, "Calibration Issues - Processed: %lu | Dropped: %lu | Rate: %.1f%%",
                        processed_samples, dropped_samples,
                        sample_count > 0 ? (float)processed_samples * PERCENTAGE_CALCULATION_FACTOR / sample_count : 0.0f);
            }
        }

        // Maintain precise 100Hz timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CALIBRATION_TASK_PERIOD_MS));
    }
}