#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"

// Health monitoring structure for individual components
typedef struct {
    uint32_t total_operations;
    uint32_t successful_operations;
    uint32_t failed_operations;
    uint32_t dropped_items;
    uint32_t last_report_count;
    bool has_issues;
    const char *component_name;
} health_stats_t;

// System-wide health monitoring
typedef struct {
    health_stats_t imu_sensor;
    health_stats_t mag_sensor;
    health_stats_t gps_sensor;
    health_stats_t calibration_task;
    health_stats_t motion_fusion_task;
    health_stats_t display_task;
} system_health_t;

// Initialize health monitoring for a component
void health_stats_init(health_stats_t *stats, const char *component_name);

// Update health statistics
void health_record_success(health_stats_t *stats);
void health_record_failure(health_stats_t *stats);
void health_record_drop(health_stats_t *stats);

// Calculate success rate percentage
uint32_t health_get_success_rate(health_stats_t *stats);

// Check if component should report health (based on interval)
bool health_should_report(health_stats_t *stats, uint32_t report_interval);

// Report health status for a component
void health_report_component(health_stats_t *stats, const char *task_name);

// Report overall system health
void health_report_system(system_health_t *system_health);

// Global system health instance
extern system_health_t g_system_health;

#endif