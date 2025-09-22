#ifndef QUEUE_UTILS_H
#define QUEUE_UTILS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

// Smart queue send with overflow management
// If queue is full, drops oldest item and inserts newest for ultra-responsiveness
// Returns: ESP_OK on success, ESP_FAIL on failure
esp_err_t queue_send_with_overflow(QueueHandle_t queue, const void *item, size_t item_size, const char *task_name, const char *queue_name, uint32_t timestamp_ms);

// Simple queue send with drop on full
// If queue is full, drops the new item and logs warning
// Returns: ESP_OK on success, ESP_FAIL if dropped
esp_err_t queue_send_or_drop(QueueHandle_t queue, const void *item, const char *task_name, const char *queue_name);

#endif