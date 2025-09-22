#ifndef TIMING_UTILS_H
#define TIMING_UTILS_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// Timing measurement utilities for ultra-responsive pipeline
typedef struct {
    uint64_t imu_timestamp_us;
    uint64_t calibration_start_us;
    uint64_t calibration_end_us;
    uint64_t fusion_start_us;
    uint64_t fusion_end_us;
    uint64_t display_start_us;
    uint64_t display_end_us;
} pipeline_timing_t;

// Timing statistics (internal storage in microseconds)
typedef struct {
    uint32_t total_samples;
    uint32_t max_latency_us;
    uint32_t min_latency_us;
    uint32_t avg_latency_us;
    uint32_t samples_over_50ms;
    uint64_t last_report_time_us;
} timing_stats_t;

// Get current timestamp in microseconds
static inline uint64_t get_timestamp_us(void) {
    return esp_timer_get_time();
}

// Get current timestamp in milliseconds (for compatibility)
static inline uint32_t get_timestamp_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Calculate elapsed time between two timestamps in microseconds
static inline uint32_t calc_elapsed_us(uint64_t start_us, uint64_t end_us) {
    if (end_us >= start_us) {
        return (uint32_t)(end_us - start_us);
    } else {
        // Handle timer rollover (very unlikely with 64-bit timer)
        return (uint32_t)((UINT64_MAX - start_us) + end_us + 1);
    }
}

// Calculate elapsed time between two timestamps in milliseconds
static inline uint32_t calc_elapsed_ms(uint32_t start_ms, uint32_t end_ms) {
    if (end_ms >= start_ms) {
        return end_ms - start_ms;
    } else {
        // Handle timer rollover
        return (UINT32_MAX - start_ms) + end_ms + 1;
    }
}

// Initialize timing statistics
void timing_stats_init(timing_stats_t *stats);

// Update timing statistics with new latency measurement (in microseconds)
void timing_stats_update(timing_stats_t *stats, uint32_t latency_us);

// Generate timing report (returns true if report was generated)
bool timing_stats_report(timing_stats_t *stats, const char *task_name, uint32_t report_interval_ms);

#endif // TIMING_UTILS_H