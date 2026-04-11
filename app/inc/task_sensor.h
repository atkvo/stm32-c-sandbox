#pragma once

#include <stdbool.h>
#include "si7021.h"

typedef struct {
    si7021_handle_t sensor;
    uint32_t humidity;
} sensor_task_ctx_t;

sensor_task_ctx_t task_sensor_create_ctx(si7021_handle_t sensor);
void task_sensor(sensor_task_ctx_t *ctx);
