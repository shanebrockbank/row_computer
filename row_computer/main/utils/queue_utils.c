#include "queue_utils.h"
#include <string.h>

esp_err_t queue_send_with_overflow(QueueHandle_t queue, const void *item, size_t item_size, const char *task_name, const char *queue_name, uint32_t timestamp_ms) {
    if (xQueueSend(queue, item, 0) == pdTRUE) {
        return ESP_OK;
    }

    // Queue full - drop oldest and insert newest for ultra-responsiveness
    void *discarded_data = malloc(item_size);
    if (discarded_data == NULL) {
        ESP_LOGE(task_name, "%s queue full - couldn't allocate temp buffer", queue_name);
        return ESP_FAIL;
    }

    if (xQueueReceive(queue, discarded_data, 0) == pdTRUE) {
        // Successfully removed oldest, now add newest
        if (xQueueSend(queue, item, 0) == pdTRUE) {
            // Extract timestamp from discarded data (assumes first field is timestamp_ms)
            uint32_t discarded_timestamp = *(uint32_t*)discarded_data;
            ESP_LOGW(task_name, "%s queue full - dropped sample from %lu ms, kept %lu ms",
                    queue_name, discarded_timestamp, timestamp_ms);
            free(discarded_data);
            return ESP_OK;
        } else {
            ESP_LOGE(task_name, "Failed to add to %s queue after clearing space", queue_name);
            free(discarded_data);
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE(task_name, "%s queue full but couldn't remove oldest sample", queue_name);
        free(discarded_data);
        return ESP_FAIL;
    }
}

esp_err_t queue_send_or_drop(QueueHandle_t queue, const void *item, const char *task_name, const char *queue_name) {
    if (xQueueSend(queue, item, 0) == pdTRUE) {
        return ESP_OK;
    } else {
        ESP_LOGW(task_name, "%s queue full - dropping item", queue_name);
        return ESP_FAIL;
    }
}