#pragma once

#include <stdint.h>
#include "gpio.h"

typedef enum {
    I2C_WRITE = 0,
    I2C_READ  = 1
} i2c_direction_t;

struct i2c_ctx;

typedef struct i2c_ctx* i2c_handle_t;

/**
 * @brief Initialize hardware (Clocks, GPIOs, Baud rate, Duty cycle)
 */
i2c_handle_t i2c_acquire(uint8_t i2c_idx);

/**
 * @brief Configures I2C device
 */
void i2c_init(i2c_handle_t handle, gpio_pin_handle_t scl, gpio_pin_handle_t sda);

/**
 * @brief Simple byte write to a device register
 */
void i2c_write_reg(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Simple byte read from a device register
 */
uint8_t i2c_read_reg(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr);

/**
 * @brief Burst read/write
 */
void i2c_burst_write(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
void i2c_burst_read(i2c_handle_t handle, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
