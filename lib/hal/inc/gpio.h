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
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
} gpio_port_t;

/**
 * @brief GPIO configuration field
 *
 * Caller is expected to set the appropriate fields
 * depending on the `gpio_mode_t` requested
 */
typedef struct {
    gpio_mode_t mode;
    gpio_pupd_mode_t pupd;

    union {
        struct {
            gpio_speed_t speed;
            gpio_output_mode_t output_mode;
        } output;
        struct {
            /* nothing to configure */
        } input;
        struct {
            gpio_speed_t speed;
            gpio_output_mode_t output_mode;
            uint8_t af_mode;
        } af;
    };
} gpio_pin_config_t;

/**
 * @brief GPIO pin context struct to be defined by
 * implementating module.
 */
struct gpio_pin_ctx;

/**
 * @brief GPIO pin handle
 */
typedef struct gpio_pin_ctx *gpio_pin_handle_t;

/**
 * @brief Acquire a GPIO pin handle
 *
 * Caller will now own the GPIO pin handle after this. Any subsequent
 * calls to the same GPIO pin handle will result in NULL being returned
 *
 * @param port - GPIO port
 * @param pin - GPIO pin
 *
 * @return handle to port-pin if valid
 * @return NULL if invalid
 */
gpio_pin_handle_t gpio_pin_acquire(gpio_port_t port, uint8_t pin_index);

/**
 * @brief Release a GPIO pin handle
 *
 * @param handle
 *
 * @return none
 */
void gpio_pin_release(gpio_pin_handle_t handle);

/**
 * @brief Configure a GPIO pin
 *
 * @param handle - pin to configure
 * @param cfg - configuration to apply to GPIO pin
 *
 * @return none
 */
void gpio_pin_configure(gpio_pin_handle_t handle, gpio_pin_config_t cfg);

/**
 * @brief Read a GPIO pin
 *
 * @return true if pin is high
 * @return false if pin is low
bool gpio_pin_read(gpio_pin_handle_t handle);

/**
 * @brief Write a value to GPIO pin
 *
 * @param handle
 * @param state - value to set to pin
 *
 * @return none
 */
void gpio_pin_write(gpio_pin_handle_t handle, bool state);
