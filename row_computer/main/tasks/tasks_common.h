#ifndef TASKS_COMMON_H
#define TASKS_COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sensors/sensors_common.h"
#include "sensors/gps.h"

// Task function declarations
void imu_task(void *parameters);
void gps_task(void *parameters);
void logging_task(void *parameters);

// Task management functions
esp_err_t create_tasks(void);
esp_err_t create_inter_task_comm(void);

// Global task handles (extern declarations for use in other files)
extern TaskHandle_t imu_task_handle;
extern TaskHandle_t gps_task_handle;
extern TaskHandle_t logging_task_handle;

// Global queue handles (extern declarations for use in other files)
extern QueueHandle_t imu_data_queue;
extern QueueHandle_t gps_data_queue;

#endif