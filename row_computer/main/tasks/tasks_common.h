#ifndef TASK_COMMON
#define TASK_COMMON

#include "tasks/gps_task.h"
#include "tasks/logging_task.h"
#include "tasks/sensor_task.h"

esp_err_t create_tasks(void);
esp_err_t create_inter_task_comm(void);

#endif
