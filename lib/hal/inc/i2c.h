#pragma once

#include <stdint.h>
#include "gpio.h"
#include "slice.h"

typedef enum {
    I2C_WRITE = 0,
    I2C_READ  = 1
} i2c_direction_t;

struct i2c_ctx;

typedef struct i2c_ctx* i2c_handle_t;

/**
 * @brief Take an I2C handle if available
 */
i2c_handle_t i2c_take(uint8_t i2c_idx);
void i2c_release(i2c_handle_t);

/**
 * @brief Configures I2C device
 */
void i2c_init(i2c_handle_t handle, gpio_pin_handle_t scl, gpio_pin_handle_t sda);

/**
 * @brief Burst read/write
 */
// @todo: remove the reg_addr parameter
void i2c_burst_write(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, slice_t data);
void i2c_burst_read(i2c_handle_t handle, uint8_t dev_addr, slice_mutable_t data);

void i2c_burst_write_read(i2c_handle_t handle, uint8_t dev_addr, slice_t write_data, slice_mutable_t read_data);

// @todo: should this take a callback to handle completion?
void i2c_burst_write_nb(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, slice_t data);
