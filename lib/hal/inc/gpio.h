#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Gpio mode
 */
typedef enum {
    GPIO_MODE_INPUT       = 0x0,
    GPIO_MODE_OUTPUT      = 0x1,
    GPIO_MODE_ALTERNATE   = 0x2,
    GPIO_MODE_ANALOG      = 0x3
} gpio_mode_t;

/**
 * @brief Gpio output types
 */
typedef enum {
    GPIO_OUTPUT_MODE_PUSH_PULL  = 0x0,
    GPIO_OUTPUT_MODE_OPEN_DRAIN = 0x1
} gpio_output_mode_t;

/**
 * @brief Gpio pull up pull down modes
*/
typedef enum
{
    GPIO_PUPD_NONE     = 0x0,
    GPIO_PUPD_PULLUP   = 0x1,
    GPIO_PUPD_PULLDOWN = 0x2,
} gpio_pupd_mode_t;

/**
 * @brief Gpio speed modes
*/
typedef enum {
    GPIO_SPEED_LOW    = 0x0,
    GPIO_SPEED_MEDIUM = 0x1,
    GPIO_SPEED_FAST   = 0x2,
    GPIO_SPEED_HIGH   = 0x3,
} gpio_speed_t;

/**
 * @brief gpio_port_t
 */
typedef enum {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
} gpio_port_t;

struct gpio_port_ctx;
typedef struct gpio_port_ctx *gpio_port_handle_t;

gpio_port_handle_t gpio_handle_acquire(gpio_port_t port);
void gpio_handle_release(gpio_port_handle_t handle);

/* maybe this isn't required - or maybe just supply a configuration
 * like gpio_port_config_t
 */
void gpio_init(gpio_port_handle_t handle);

void gpio_set_mode(gpio_port_handle_t handle, gpio_mode_t mode);
void gpio_set_output_mode(gpio_port_handle_t handle, gpio_output_mode_t mode);
void gpio_set_pupd(gpio_port_handle_t handle, gpio_pupd_mode_t mode);

bool gpio_read_pin(gpio_port_handle_t handle, uint8_t pin_index);
uint32_t gpio_read_port(gpio_port_handle_t handle);

uint32_t gpio_write_pin(gpio_port_handle_t handle, bool state);
uint32_t gpio_write_port(gpio_port_handle_t handle, uint32_t value);
