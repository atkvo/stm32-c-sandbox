#pragma once

#include <stdbool.h>
#include "si7021.h"

typedef struct {
    si7021_handle_t sensor;
    uint32_t humidity;
    uint32_t temperature;
} task_sensor_ctx_t;

task_sensor_ctx_t task_sensor_create_ctx(si7021_handle_t sensor);
void task_sensor(task_sensor_ctx_t *ctx);
