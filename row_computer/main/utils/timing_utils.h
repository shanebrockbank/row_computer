#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Timing measurement utilities for ultra-responsive pipeline
typedef struct {
    uint32_t imu_timestamp_ms;
    uint32_t calibration_start_ms;
    uint32_t calibration_end_ms;
    uint32_t fusion_start_ms;
    uint32_t fusion_end_ms;
    uint32_t display_start_ms;
    uint32_t display_end_ms;
} pipeline_timing_t;

// Timing statistics
typedef struct {
    uint32_t total_samples;
    uint32_t max_latency_ms;
    uint32_t min_latency_ms;
    uint32_t avg_latency_ms;
    uint32_t samples_over_200ms;
    uint32_t last_report_time;
} timing_stats_t;

// Get current timestamp in milliseconds
static inline uint32_t get_timestamp_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Calculate elapsed time between two timestamps
static inline uint32_t calc_elapsed_ms(uint32_t start_ms, uint32_t end_ms) {
    if (end_ms >= start_ms) {
        return end_ms - start_ms;
    } else {
        // Handle timer rollover (occurs every ~49 days)
        return (UINT32_MAX - start_ms) + end_ms + 1;
    }
}

// Initialize timing statistics
void timing_stats_init(timing_stats_t *stats);

// Update timing statistics with new latency measurement
void timing_stats_update(timing_stats_t *stats, uint32_t latency_ms);

// Generate timing report (returns true if report was generated)
bool timing_stats_report(timing_stats_t *stats, const char *task_name, uint32_t report_interval_ms);

#endif // TIMING_UTILS_H