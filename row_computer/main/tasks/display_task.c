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

            // Calculate end-to-end latency with microsecond precision
            uint64_t current_time_us = get_timestamp_us();
            uint64_t motion_timestamp_us = (uint64_t)latest_motion_state.timestamp_ms * MS_TO_US_MULTIPLIER; // Convert ms to us
            uint32_t latency_us = calc_elapsed_us(motion_timestamp_us, current_time_us);

            // Update timing statistics
            timing_stats_update(&latency_stats, latency_us);


            // Generate timing reports periodically (every 5 seconds)
            timing_stats_report(&latency_stats, "END-TO-END", TIMING_REPORT_INTERVAL_MS);


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