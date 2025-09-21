#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tasks/tasks_common.h"
#include "config/common_constants.h"
#include "sensors/sensors_common.h"
#include "sensors/gps.h"
#include <math.h>

static const char *TAG = "MOTION_FUSION_TASK";

void motion_fusion_task(void *parameters) {
    ESP_LOGI(TAG, "Starting motion fusion task at %dHz", 1000/MOTION_FUSION_PERIOD_MS);

    processed_imu_data_t processed_imu;
    gps_data_t gps_data;
    motion_state_t motion_state;
    TickType_t last_wake_time = xTaskGetTickCount();

    // Fusion state variables
    static uint32_t imu_samples_processed = 0;
    static uint32_t gps_updates_received = 0;
    static uint32_t motion_states_generated = 0;
    static uint32_t dropped_states = 0;
    static bool has_gps_reference = false;
    static gps_data_t last_valid_gps;

    while (1) {
        bool new_imu_data = false;
        bool new_gps_data = false;

        // Process all available IMU data (consume queue to prevent backup)
        while (xQueueReceive(processed_imu_data_queue, &processed_imu, 0) == pdTRUE) {
            new_imu_data = true;
            imu_samples_processed++;
        }

        // Check for new GPS data
        if (xQueueReceive(gps_data_queue, &gps_data, 0) == pdTRUE) {
            new_gps_data = true;
            gps_updates_received++;

            if (gps_data.valid_fix) {
                has_gps_reference = true;
                last_valid_gps = gps_data;
            }
        }

        // Generate motion state if we have IMU data
        if (new_imu_data) {
            // TODO: Implement proper sensor fusion algorithm (Extended Kalman Filter, etc.)
            // For now, create basic motion state with latest data

            motion_state.timestamp_ms = processed_imu.timestamp_ms;

            // IMU-based motion data
            motion_state.accel_x = processed_imu.accel_x;
            motion_state.accel_y = processed_imu.accel_y;
            motion_state.accel_z = processed_imu.accel_z;
            motion_state.gyro_x = processed_imu.gyro_x;
            motion_state.gyro_y = processed_imu.gyro_y;
            motion_state.gyro_z = processed_imu.gyro_z;

            // GPS-based position data (if available)
            if (has_gps_reference) {
                motion_state.latitude = last_valid_gps.latitude;
                motion_state.longitude = last_valid_gps.longitude;
                motion_state.gps_speed_knots = last_valid_gps.speed_knots;
                motion_state.gps_valid = last_valid_gps.valid_fix;
                motion_state.satellites = last_valid_gps.satellites;
            } else {
                motion_state.gps_valid = false;
                motion_state.satellites = 0;
            }

            // Basic derived motion metrics (placeholder for future algorithms)
            float accel_magnitude = sqrt(motion_state.accel_x * motion_state.accel_x +
                                       motion_state.accel_y * motion_state.accel_y +
                                       motion_state.accel_z * motion_state.accel_z);
            motion_state.total_acceleration = accel_magnitude;

            // Debug logging - temporary to trace motion fusion
            static int fusion_debug_counter = 0;
            if (++fusion_debug_counter % 50 == 0) { // Log every 50 samples (once per second at 50Hz)
                ESP_LOGI(TAG, "MOTION_FUSION: AX=%.3f AY=%.3f AZ=%.3f | GX=%.2f GY=%.2f GZ=%.2f | Total: %.3fg",
                        motion_state.accel_x, motion_state.accel_y, motion_state.accel_z,
                        motion_state.gyro_x, motion_state.gyro_y, motion_state.gyro_z,
                        motion_state.total_acceleration);
            }

            // Send to motion state queue (non-blocking)
            if (xQueueSend(motion_state_queue, &motion_state, 0) == pdTRUE) {
                motion_states_generated++;
            } else {
                dropped_states++;
                ESP_LOGW(TAG, "Motion state queue full - dropping state");
            }
        }

        // Log fusion health periodically
        static uint32_t health_counter = 0;
        if (++health_counter >= (SENSOR_HEALTH_LOG_INTERVAL / 5)) { // More frequent for fusion task
            ESP_LOGI(TAG, "Fusion Health - IMU: %lu | GPS: %lu | States: %lu | Dropped: %lu | GPS: %s",
                    imu_samples_processed, gps_updates_received, motion_states_generated, dropped_states,
                    has_gps_reference ? "VALID" : "NO_FIX");
            health_counter = 0;
        }

        // Maintain precise 50Hz timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTION_FUSION_PERIOD_MS));
    }
}