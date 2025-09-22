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

// New ultra-responsive task functions
void calibration_filter_task(void *parameters);
void motion_fusion_task(void *parameters);
void display_task(void *parameters);

// Task management functions
esp_err_t create_tasks(void);
esp_err_t create_inter_task_comm(void);

// Global task handles (extern declarations for use in other files)
extern TaskHandle_t imu_task_handle;
extern TaskHandle_t gps_task_handle;

// New ultra-responsive task handles
extern TaskHandle_t calibration_filter_task_handle;
extern TaskHandle_t motion_fusion_task_handle;
extern TaskHandle_t display_task_handle;

// Global queue handles (extern declarations for use in other files)
extern QueueHandle_t gps_data_queue;

// New ultra-responsive queue handles
extern QueueHandle_t raw_imu_data_queue;          // IMU → Calibration
extern QueueHandle_t processed_imu_data_queue;    // Calibration → Fusion
extern QueueHandle_t motion_state_queue;          // Fusion → Display/Logging

#endif