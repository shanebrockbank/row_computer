#include "timing_utils.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TIMING";

void timing_stats_init(timing_stats_t *stats) {
    memset(stats, 0, sizeof(timing_stats_t));
    stats->min_latency_us = UINT32_MAX;
    stats->last_report_time_us = get_timestamp_us();
}

void timing_stats_update(timing_stats_t *stats, uint32_t latency_us) {
    stats->total_samples++;

    if (latency_us > stats->max_latency_us) {
        stats->max_latency_us = latency_us;
    }

    if (latency_us < stats->min_latency_us) {
        stats->min_latency_us = latency_us;
    }

    if (latency_us > 50000) { // 50ms in microseconds
        stats->samples_over_50ms++;
    }

    // Update running average (simple moving average)
    if (stats->total_samples == 1) {
        stats->avg_latency_us = latency_us;
    } else {
        // Use a simple exponential moving average to prevent overflow
        stats->avg_latency_us = (stats->avg_latency_us * 9 + latency_us) / 10;
    }
}

bool timing_stats_report(timing_stats_t *stats, const char *task_name, uint32_t report_interval_ms) {
    uint64_t current_time_us = get_timestamp_us();
    uint64_t elapsed_since_report_us = current_time_us - stats->last_report_time_us;
    uint32_t elapsed_since_report_ms = (uint32_t)(elapsed_since_report_us / 1000);

    if (elapsed_since_report_ms >= report_interval_ms && stats->total_samples > 0) {
        float success_rate = ((float)(stats->total_samples - stats->samples_over_50ms) / stats->total_samples) * 100.0f;

        // Convert microseconds to milliseconds for display, with decimal precision
        float avg_ms = stats->avg_latency_us / 1000.0f;
        float max_ms = stats->max_latency_us / 1000.0f;
        float min_ms = stats->min_latency_us / 1000.0f;

        ESP_LOGI(TAG, "%s Latency - Avg: %.1fms | Max: %.1fms | Min: %.1fms | >50ms: %lu/%lu (%.1f%% success)",
                task_name, avg_ms, max_ms, min_ms,
                stats->samples_over_50ms, stats->total_samples, success_rate);

        if (stats->samples_over_50ms > 0) {
            ESP_LOGW(TAG, "%s WARNING: %lu samples exceeded 50ms latency target!",
                    task_name, stats->samples_over_50ms);
        }

        // Reset statistics for next reporting period
        stats->max_latency_us = 0;
        stats->min_latency_us = UINT32_MAX;
        stats->avg_latency_us = 0;
        stats->samples_over_50ms = 0;
        stats->total_samples = 0;
        stats->last_report_time_us = current_time_us;
        return true;
    }

    return false;
}