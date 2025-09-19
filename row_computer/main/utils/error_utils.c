#include "error_utils.h"

esp_err_t validate_sensor_data_ptr(const void *ptr, const char *sensor_name) {
    if (ptr == NULL) {
        ESP_LOGE(sensor_name, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t log_sensor_status(const char *sensor_name, esp_err_t result, const char *operation) {
    if (result == ESP_OK) {
        ESP_LOGD(sensor_name, "%s successful", operation);
    } else {
        ESP_LOGE(sensor_name, "%s failed: %s", operation, esp_err_to_name(result));
    }
    return result;
}

esp_err_t check_initialization_status(bool is_initialized, const char *module_name) {
    if (!is_initialized) {
        ESP_LOGE(module_name, "Module not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}