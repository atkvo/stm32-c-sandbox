#pragma once

#include <stdint.h>
#include "i2c.h"

typedef struct si7021_ctx {
    i2c_handle_t i2c;
} si7021_ctx_t;

typedef si7021_ctx_t* si7021_handle_t;

si7021_handle_t si7021_handle_create(si7021_ctx_t *ctx, i2c_handle_t i2c);
uint32_t si7021_temp_get(si7021_handle_t h);
uint32_t si7021_humid_get(si7021_handle_t h);
