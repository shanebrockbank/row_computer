#include "timing.h"
#include "config/timing_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TIMING";

// Runtime verbose mode toggle (can be changed via debugger or command interface)
bool verbose_mode = false;

// =============================================================================
// CORE TIMING FUNCTIONS
// =============================================================================

uint32_t get_system_time_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void create_timestamp(sensor_timestamp_t *timestamp, bool data_valid) {
    timestamp->timestamp_ms = get_system_time_ms();
    timestamp->data_valid = data_valid;
}

// =============================================================================
// RATE LIMITING FOR HIGH-FREQUENCY LOGS
// =============================================================================

typedef struct {
    uint32_t last_log_time_ms;
    uint32_t suppressed_count;
} rate_limiter_t;

static rate_limiter_t imu_rate_limiter = {0, 0};
static rate_limiter_t general_rate_limiter = {0, 0};

bool should_log_high_freq(rate_limiter_t *limiter, const char *tag) {
    if (verbose_mode) {
        return true; // No rate limiting in verbose mode
    }
    
    uint32_t current_time = get_system_time_ms();
    uint32_t min_interval_ms = 1000 / HIGH_FREQ_LOG_LIMIT_HZ; // Convert Hz to ms interval
    
    if (current_time - limiter->last_log_time_ms >= min_interval_ms) {
        // Time to log again
        if (limiter->suppressed_count > 0) {
            ESP_LOGI(TAG, "[%s] %lu messages suppressed (rate limiting)", 
                    tag, limiter->suppressed_count);
            limiter->suppressed_count = 0;
        }
        limiter->last_log_time_ms = current_time;
        return true;
    } else {
        // Suppress this log
        limiter->suppressed_count++;
        return false;
    }
}

bool should_log_imu_high_freq(void) {
    return should_log_high_freq(&imu_rate_limiter, "IMU");
}

bool should_log_general_high_freq(void) {
    return should_log_high_freq(&general_rate_limiter, "GENERAL");
}

// =============================================================================
// QUEUE OVERFLOW HANDLING
// =============================================================================

bool send_to_queue_with_overflow(QueueHandle_t queue, const void *item, 
                                size_t item_size, const char *queue_name) {
    if (xQueueSend(queue, item, QUEUE_SEND_TIMEOUT_MS) == pdTRUE) {
        return true; // Success
    }
    
    // Queue is full - drop oldest item and try again
    void *dropped_item = malloc(item_size);
    if (dropped_item != NULL) {
        if (xQueueReceive(queue, dropped_item, 0) == pdTRUE) {
            // Successfully dropped oldest item
            if (should_log_general_high_freq()) {
                ESP_LOGW(TAG, "Queue '%s' overflow - dropped oldest sample", queue_name);
            }
            
            // Now try to send new item
            if (xQueueSend(queue, item, 0) == pdTRUE) {
                free(dropped_item);
                return true;
            } else {
                ESP_LOGE(TAG, "Queue '%s' still full after drop - this shouldn't happen", queue_name);
            }
        } else {
            ESP_LOGE(TAG, "Queue '%s' reported full but couldn't drop item", queue_name);
        }
        free(dropped_item);
    } else {
        ESP_LOGE(TAG, "Failed to allocate memory for queue overflow handling");
    }
    
    return false; // Failed to send
}

// =============================================================================
// QUEUE HEALTH MONITORING
// =============================================================================

void log_queue_health(QueueHandle_t queue, const char *queue_name, 
                     uint32_t max_size, uint32_t sample_rate_hz) {
    UBaseType_t items_waiting = uxQueueMessagesWaiting(queue);
    UBaseType_t spaces_available = uxQueueSpacesAvailable(queue);
    
    float fill_percentage = (float)items_waiting / (float)max_size * 100.0f;
    float buffer_time_s = (float)items_waiting / (float)sample_rate_hz;
    
    ESP_LOGI(TAG, "Queue '%s': %u/%u items (%.1f%%), %.2fs buffered", 
            queue_name, items_waiting, max_size, fill_percentage, buffer_time_s);
    
    // Warn if queue is getting full
    if (fill_percentage > 80.0f) {
        ESP_LOGW(TAG, "Queue '%s' is %.1f%% full - potential overflow risk", 
                queue_name, fill_percentage);
    }
}

// =============================================================================
// SYSTEM HEALTH MONITORING
// =============================================================================

void init_sensor_status(sensor_status_t *status) {
    status->sensor_available = false;
    status->last_successful_read_ms = 0;
    status->consecutive_failures = 0;
    status->total_error_count = 0;
}

void update_sensor_status(sensor_status_t *status, bool read_success) {
    if (read_success) {
        status->sensor_available = true;
        status->last_successful_read_ms = get_system_time_ms();
        status->consecutive_failures = 0;
    } else {
        status->consecutive_failures++;
        status->total_error_count++;
        
        // Check if sensor should be marked unavailable
        uint32_t time_since_last_read = get_system_time_ms() - status->last_successful_read_ms;
        if (time_since_last_read > SENSOR_TIMEOUT_MS) {
            status->sensor_available = false;
        }
    }
}

bool sensor_needs_reset(const sensor_status_t *status) {
    return status->consecutive_failures >= MAX_CONSECUTIVE_FAILURES;
}

void log_sensor_health(const sensor_status_t *status, const char *sensor_name) {
    uint32_t current_time = get_system_time_ms();
    uint32_t time_since_last_read = current_time - status->last_successful_read_ms;
    
    if (status->sensor_available) {
        ESP_LOGI(TAG, "Sensor '%s': OK (last read %lu ms ago, %lu total errors)", 
                sensor_name, time_since_last_read, status->total_error_count);
    } else {
        ESP_LOGW(TAG, "Sensor '%s': UNAVAILABLE (%lu consecutive failures, %lu ms since last read)", 
                sensor_name, status->consecutive_failures, time_since_last_read);
    }
    
    if (sensor_needs_reset(status)) {
        ESP_LOGW(TAG, "Sensor '%s' needs reset (>%d consecutive failures)", 
                sensor_name, MAX_CONSECUTIVE_FAILURES);
    }
}

// =============================================================================
// SYSTEM STATUS REPORTING
// =============================================================================

void print_system_timing_status(void) {
    uint32_t current_time = get_system_time_ms();
    uint32_t free_heap = esp_get_free_heap_size();
    
    ESP_LOGI(TAG, "=== System Timing Status ===");
    ESP_LOGI(TAG, "System time: %lu ms (%.1f seconds)", current_time, current_time / 1000.0f);
    ESP_LOGI(TAG, "Free heap: %lu bytes (%.1f KB)", free_heap, free_heap / 1024.0f);
    ESP_LOGI(TAG, "Verbose mode: %s", verbose_mode ? "ON" : "OFF");
    ESP_LOGI(TAG, "Task rates: IMU=%dms (%.1fHz), GPS=%dms (%.1fHz), Log=%dms (%.1fHz)",
            IMU_TASK_RATE_MS, 1000.0f/IMU_TASK_RATE_MS,
            GPS_TASK_RATE_MS, 1000.0f/GPS_TASK_RATE_MS, 
            LOG_TASK_RATE_MS, 1000.0f/LOG_TASK_RATE_MS);
    ESP_LOGI(TAG, "Queue sizes: IMU=%d, GPS=%d, Output=%d", 
            IMU_QUEUE_SIZE, GPS_QUEUE_SIZE, OUTPUT_QUEUE_SIZE);
}

// =============================================================================
// SIMPLE COMMAND INTERFACE FOR VERBOSE MODE
// =============================================================================

void toggle_verbose_mode(void) {
    verbose_mode = !verbose_mode;
    ESP_LOGI(TAG, "Verbose mode %s", verbose_mode ? "ENABLED" : "DISABLED");
    
    // Update log levels if needed
    if (verbose_mode) {
        esp_log_level_set("IMU_TASK", ESP_LOG_DEBUG);
        esp_log_level_set("GPS_TASK", ESP_LOG_DEBUG);
        esp_log_level_set("LOG_TASK", ESP_LOG_DEBUG);
    } else {
        esp_log_level_set("IMU_TASK", LOG_LEVEL_IMU);
        esp_log_level_set("GPS_TASK", LOG_LEVEL_GPS);
        esp_log_level_set("LOG_TASK", ESP_LOG_INFO);
    }
}