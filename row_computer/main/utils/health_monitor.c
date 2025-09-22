#include "health_monitor.h"
#include "config/common_constants.h"
#include <string.h>

static const char *TAG = "HEALTH";

// Global system health instance
system_health_t g_system_health;

void health_stats_init(health_stats_t *stats, const char *component_name) {
    memset(stats, 0, sizeof(health_stats_t));
    stats->component_name = component_name;
    stats->has_issues = false;
}

void health_record_success(health_stats_t *stats) {
    stats->total_operations++;
    stats->successful_operations++;
}

void health_record_failure(health_stats_t *stats) {
    stats->total_operations++;
    stats->failed_operations++;
    stats->has_issues = true;
}

void health_record_drop(health_stats_t *stats) {
    stats->dropped_items++;
    stats->has_issues = true;
}

uint32_t health_get_success_rate(health_stats_t *stats) {
    if (stats->total_operations == 0) {
        return 0;
    }
    return (stats->successful_operations * PERCENTAGE_CALCULATION_FACTOR) / stats->total_operations;
}

bool health_should_report(health_stats_t *stats, uint32_t report_interval) {
    uint32_t operations_since_report = stats->total_operations - stats->last_report_count;
    return operations_since_report >= report_interval;
}

void health_report_component(health_stats_t *stats, const char *task_name) {
    if (stats->total_operations == 0) {
        return; // No data to report
    }

    // Don't report on small sample sizes to avoid noise
    if (stats->total_operations < HEALTH_MIN_OPERATIONS_TO_REPORT) {
        return;
    }

    uint32_t success_rate = health_get_success_rate(stats);

    // SILENT SUCCESS, LOUD PROBLEMS: Only report if there are actual issues
    bool has_problems = (stats->has_issues ||
                        stats->dropped_items > 0 ||
                        success_rate < HEALTH_SUCCESS_RATE_THRESHOLD);

    if (has_problems) {
        if (stats->dropped_items > 0) {
            ESP_LOGW(TAG, "%s [%s] - ISSUES: Success: %lu%% (%lu/%lu) | Drops: %lu",
                    task_name, stats->component_name, success_rate,
                    stats->successful_operations, stats->total_operations, stats->dropped_items);
        } else {
            ESP_LOGW(TAG, "%s [%s] - ISSUES: Success: %lu%% (%lu/%lu)",
                    task_name, stats->component_name, success_rate,
                    stats->successful_operations, stats->total_operations);
        }
    }
    // No else branch - silent when healthy!

    // Reset issue flag after reporting
    stats->has_issues = false;
    stats->last_report_count = stats->total_operations;
}

void health_report_system(system_health_t *system_health) {
    ESP_LOGI(TAG, "=== SYSTEM HEALTH SUMMARY ===");

    health_report_component(&system_health->imu_sensor, "IMU_TASK");
    health_report_component(&system_health->mag_sensor, "IMU_TASK");
    health_report_component(&system_health->gps_sensor, "GPS_TASK");
    health_report_component(&system_health->calibration_task, "CALIBRATION");
    health_report_component(&system_health->motion_fusion_task, "FUSION");
    health_report_component(&system_health->display_task, "DISPLAY");

    ESP_LOGI(TAG, "=== END HEALTH SUMMARY ===");
}