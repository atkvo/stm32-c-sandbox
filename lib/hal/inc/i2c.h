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
 * @brief Chained/Vector write. Useful for [Command + Data] without copying.
 * This is your "Master" write function.
 */
void i2c_write_v(i2c_handle_t handle, uint8_t dev_addr, const slice_t* slices, size_t num_slices);

/**
 * @brief Standard block write. Wraps i2c_write_v.
 */
static inline void i2c_write(i2c_handle_t handle, uint8_t dev_addr, slice_t data) {
    i2c_write_v(handle, dev_addr, &data, 1);
}

/**
 * @brief Standard block read.
 */
void i2c_read(i2c_handle_t handle, uint8_t dev_addr, slice_mutable_t data);

/**
 * @brief Combined Write then Read (Repeated Start).
 * Perfect for register reads: i2c_write_read(h, addr, {&reg, 1}, {&rx, 1});
 */
void i2c_write_read(i2c_handle_t handle, uint8_t dev_addr, slice_t write_data, slice_mutable_t read_data);

/**
 * @brief Standard block write. Non-blocking.
 * @todo: add callback handler parameter
 */
void i2c_write_v_nb(i2c_handle_t handle, uint8_t dev_addr, const slice_t* slices, size_t num_slices);

/**
 * @brief Standard block write. Wraps i2c_write_v_nb. Non-blocking.
 */
static void i2c_write_nb(i2c_handle_t handle, uint8_t dev_addr, slice_t data) {
    i2c_write_v_nb(handle, dev_addr, &data, 1);
}
