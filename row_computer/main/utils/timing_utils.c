#include "timing_utils.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "TIMING";

void timing_stats_init(timing_stats_t *stats) {
    memset(stats, 0, sizeof(timing_stats_t));
    stats->min_latency_ms = UINT32_MAX;
    stats->last_report_time = get_timestamp_ms();
}

void timing_stats_update(timing_stats_t *stats, uint32_t latency_ms) {
    stats->total_samples++;

    if (latency_ms > stats->max_latency_ms) {
        stats->max_latency_ms = latency_ms;
    }

    if (latency_ms < stats->min_latency_ms) {
        stats->min_latency_ms = latency_ms;
    }

    if (latency_ms > 200) {
        stats->samples_over_200ms++;
    }

    // Update running average (simple moving average)
    if (stats->total_samples == 1) {
        stats->avg_latency_ms = latency_ms;
    } else {
        // Use a simple exponential moving average to prevent overflow
        stats->avg_latency_ms = (stats->avg_latency_ms * 9 + latency_ms) / 10;
    }
}

bool timing_stats_report(timing_stats_t *stats, const char *task_name, uint32_t report_interval_ms) {
    uint32_t current_time = get_timestamp_ms();
    uint32_t elapsed_since_report = calc_elapsed_ms(stats->last_report_time, current_time);

    if (elapsed_since_report >= report_interval_ms && stats->total_samples > 0) {
        float success_rate = ((float)(stats->total_samples - stats->samples_over_200ms) / stats->total_samples) * 100.0f;

        ESP_LOGI(TAG, "%s Latency - Avg: %lums | Max: %lums | Min: %lums | >200ms: %lu/%lu (%.1f%% success)",
                task_name, stats->avg_latency_ms, stats->max_latency_ms, stats->min_latency_ms,
                stats->samples_over_200ms, stats->total_samples, success_rate);

        if (stats->samples_over_200ms > 0) {
            ESP_LOGW(TAG, "%s WARNING: %lu samples exceeded 200ms latency target!",
                    task_name, stats->samples_over_200ms);
        }

        stats->last_report_time = current_time;
        return true;
    }

    return false;
}