#ifndef ERROR_UTILS_H
#define ERROR_UTILS_H

#include "esp_err.h"
#include "esp_log.h"

// Common error handling macros
#define LOG_ERROR_RETURN(tag, err, msg, ...) \
    do { \
        ESP_LOGE(tag, msg ": %s", ##__VA_ARGS__, esp_err_to_name(err)); \
        return err; \
    } while(0)

#define LOG_ERROR_RETURN_FAIL(tag, err, msg, ...) \
    do { \
        ESP_LOGE(tag, msg ": %s", ##__VA_ARGS__, esp_err_to_name(err)); \
        return ESP_FAIL; \
    } while(0)

#define CHECK_AND_LOG_ERROR(err, tag, msg, ...) \
    do { \
        if ((err) != ESP_OK) { \
            ESP_LOGE(tag, msg ": %s", ##__VA_ARGS__, esp_err_to_name(err)); \
            return err; \
        } \
    } while(0)

#define CHECK_NULL_RETURN(ptr, tag, msg) \
    do { \
        if ((ptr) == NULL) { \
            ESP_LOGE(tag, msg ": null pointer"); \
            return ESP_ERR_INVALID_ARG; \
        } \
    } while(0)

// Common error handling functions
esp_err_t validate_sensor_data_ptr(const void *ptr, const char *sensor_name);
esp_err_t log_sensor_status(const char *sensor_name, esp_err_t result, const char *operation);
esp_err_t check_initialization_status(bool is_initialized, const char *module_name);

#endif // ERROR_UTILS_H