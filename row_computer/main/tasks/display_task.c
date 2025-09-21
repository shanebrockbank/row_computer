#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tasks/tasks_common.h"
#include "config/common_constants.h"
#include "sensors/sensors_common.h"
#include "utils/timing_utils.h"

static const char *TAG = "DISPLAY_TASK";

void display_task(void *parameters) {
    ESP_LOGI(TAG, "Starting display task at %dHz", 1000/DISPLAY_TASK_PERIOD_MS);

    motion_state_t motion_state;
    motion_state_t latest_motion_state;
    TickType_t last_wake_time = xTaskGetTickCount();
    bool has_valid_data = false;

    // Display and UI state variables
    static uint32_t display_updates = 0;
    static uint32_t data_points_consumed = 0;
    static uint32_t ui_refresh_count = 0;

    // Timing measurement
    static timing_stats_t latency_stats;
    static bool timing_initialized = false;
    if (!timing_initialized) {
        timing_stats_init(&latency_stats);
        timing_initialized = true;
    }

    while (1) {
        // Implement "read newest" logic - consume all available data but only use the latest
        bool received_new_data = false;
        while (xQueueReceive(motion_state_queue, &motion_state, 0) == pdTRUE) {
            latest_motion_state = motion_state;
            received_new_data = true;
            data_points_consumed++;
        }

        if (received_new_data) {
            has_valid_data = true;
        }

        // Update display with latest motion state (if available)
        if (has_valid_data) {
            // TODO: Implement actual display/UI updates
            // For now, log key metrics for demonstration

            display_updates++;

            // Calculate end-to-end latency
            uint32_t current_time = get_timestamp_ms();
            uint32_t latency_ms = calc_elapsed_ms(latest_motion_state.timestamp_ms, current_time);

            // Update timing statistics
            timing_stats_update(&latency_stats, latency_ms);

            // Generate timing reports periodically (every 5 seconds)
            timing_stats_report(&latency_stats, "END-TO-END", 5000);

            // Log display data periodically
            if (++ui_refresh_count >= (10 * (1000/DISPLAY_TASK_PERIOD_MS))) { // Every 10 seconds
                ESP_LOGI(TAG, "Display Update - Accel: %.2fg | Gyro: %.1f°/s | GPS: %s | Current Latency: %lums",
                        latest_motion_state.total_acceleration,
                        latest_motion_state.gyro_z, // Show Z-axis rotation
                        latest_motion_state.gps_valid ? "VALID" : "NO_FIX",
                        latency_ms);

                if (latest_motion_state.gps_valid) {
                    ESP_LOGI(TAG, "Position: %.6f°, %.6f° | Speed: %.1f kts | Sats: %d",
                            latest_motion_state.latitude, latest_motion_state.longitude,
                            latest_motion_state.gps_speed_knots, latest_motion_state.satellites);
                }

                ESP_LOGI(TAG, "Display Health - Updates: %lu | Data consumed: %lu",
                        display_updates, data_points_consumed);
                ui_refresh_count = 0;
            }

            // TODO: Here you would update:
            // - LCD/OLED display
            // - LED indicators
            // - Buzzer/audio feedback
            // - Wireless data transmission
        }

        // Maintain precise 10Hz timing for smooth display updates
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(DISPLAY_TASK_PERIOD_MS));
    }
}