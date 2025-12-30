#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "stm32f411xe.h"
#include "bit_utils.h"

#define GPIO_MAX_PORTS        (6)
#define GPIO_MAX_PIN_PER_PORT (16)

typedef struct gpio_pin_ctx {
    volatile GPIO_TypeDef *port_ptr;
    uint8_t pool_idx;
    uint8_t pin_index;
} gpio_pin_ctx_t;

typedef struct {
    uint32_t taken_map;
    gpio_pin_ctx_t pins[GPIO_MAX_PIN_PER_PORT];
} gpio_port_resource_t;

static gpio_port_resource_t gpio_port_pool[GPIO_MAX_PORTS] = { 0 };

static inline void gpio_mode_set(gpio_pin_handle_t handle, gpio_mode_t mode);
static inline void gpio_output_mode_set(gpio_pin_handle_t handle, gpio_output_mode_t mode);
static void gpio_pupd_set(gpio_pin_handle_t handle, gpio_pupd_mode_t mode);

static uint8_t port_to_pool(gpio_port_t port) {
    if (port == GPIO_PORT_H) {
        return GPIO_MAX_PORTS - 1;
    }
    else {
        return port;
    }
}

static gpio_port_t pool_to_port(uint8_t pool_idx) {
    if (pool_idx == (GPIO_MAX_PORTS - 1)) {
        return GPIO_PORT_H;
    }
    else {
        return pool_idx;
    }
}

static inline bool is_port_valid(gpio_port_t port) {
    const uint8_t valid_port_mask = 0x9F;
    return (valid_port_mask & (1 << port)) ? true : false;
}

static inline bool is_port_pin_valid(gpio_port_t port, uint8_t pin_index) {
    if (!is_port_valid(port) || (pin_index >= GPIO_MAX_PIN_PER_PORT)) {
        return false;
    }
    else {
        return true;
    }
}

static inline volatile GPIO_TypeDef* mmio_from_port(gpio_port_t port) {
    enum { GPIO_PORT_WIDTH = (GPIOB_BASE - GPIOA_BASE)  };
    static_assert(GPIO_PORT_WIDTH == 0x400, "Gpio offset assumptions will not work");
    static_assert(GPIO_PORT_A == 0, "Gpio offset handling for port A will not work");
    static_assert(GPIO_PORT_B == 1, "Gpio offset handling for port B will not work");
    static_assert(GPIO_PORT_C == 2, "Gpio offset handling for port C will not work");
    static_assert(GPIO_PORT_D == 3, "Gpio offset handling for port D will not work");
    static_assert(GPIO_PORT_E == 4, "Gpio offset handling for port E will not work");
    static_assert(GPIO_PORT_H == 7, "Gpio offset handling for port H will not work");

    return (volatile GPIO_TypeDef*)(GPIOA_BASE + (GPIO_PORT_WIDTH * port));
}

gpio_pin_handle_t gpio_pin_acquire(gpio_port_t port, uint8_t pin_index) {
    if (!is_port_pin_valid(port, pin_index)) {
        return NULL;
    }

    const uint8_t pool_idx = port_to_pool(port);
    if (!BIT_IS_SET(gpio_port_pool[pool_idx].taken_map, pin_index)) {
        gpio_port_pool[pool_idx].taken_map = BIT(pin_index);

        gpio_pin_ctx_t *pin_ctx = &gpio_port_pool[pool_idx].pins[pin_index];
        pin_ctx->pool_idx = pool_idx;
        pin_ctx->pin_index = pin_index;

        pin_ctx->port_ptr = mmio_from_port(port);

        return pin_ctx;
    }

    return NULL;
}

static inline void gpio_mode_set(gpio_pin_handle_t handle, gpio_mode_t mode) {

    static_assert(GPIO_MODE_INPUT == 0x0, "Unexpected gpio mode value");
    static_assert(GPIO_MODE_OUTPUT == 0x1, "Unexpected gpio mode value");
    static_assert(GPIO_MODE_ALTERNATE == 0x2, "Unexpected gpio mode value");
    static_assert(GPIO_MODE_ANALOG == 0x3, "Unexpected gpio mode value");

    const uint8_t mode_field_bit_width = 2;
    const uint8_t mode_field_pos = handle->pin_index * mode_field_bit_width;
    const uint32_t mode_mask = (BIT_MASK_CREATE(mode_field_bit_width) << mode_field_pos);

    uint32_t reg_value = handle->port_ptr->MODER;
    reg_value &= ~mode_mask;
    reg_value |= ((mode & BIT_MASK_CREATE(mode_field_bit_width))<< mode_field_pos);

    handle->port_ptr->MODER = reg_value;
}

static inline void gpio_output_mode_set(gpio_pin_handle_t handle, gpio_output_mode_t mode) {
    static_assert(GPIO_OUTPUT_MODE_PUSH_PULL == 0x0, "Unexpected gpio output mode value");
    static_assert(GPIO_OUTPUT_MODE_OPEN_DRAIN == 0x1, "Unexpected gpio output mode value");

    uint32_t value = handle->port_ptr->OTYPER;
    value = (value & ~BIT(handle->pin_index)) | (mode << handle->pin_index);

    handle->port_ptr->OTYPER = value;
}

static inline void gpio_pupd_set(gpio_pin_handle_t handle, gpio_pupd_mode_t mode) {
    static_assert(GPIO_PUPD_NONE == 0x0, "Unexpected gpio pu/pd mode value");
    static_assert(GPIO_PUPD_PULLUP == 0x1, "Unexpected gpio pu/pd mode value");
    static_assert(GPIO_PUPD_PULLDOWN == 0x2, "Unexpected gpio pu/pd mode value");

    const uint8_t bits_per_mode_control = 2;
    const uint8_t mask = BIT_MASK_CREATE(bits_per_mode_control);
    const uint8_t mode_index = handle->pin_index * bits_per_mode_control;

    uint32_t value = handle->port_ptr->PUPDR;
    value &= ~(mask << mode_index);
    value |= ((mode & mask) << mode_index);
    handle->port_ptr->PUPDR = value;
}

static inline void gpio_af_set(gpio_pin_handle_t handle, uint8_t af_mode) {
    const uint8_t pins_per_af_reg = 8;
    volatile uint32_t *af_reg = (handle->pin_index < pins_per_af_reg) ?
        &handle->port_ptr->AFR[0] :
        &handle->port_ptr->AFR[1];

    const uint8_t af_bits_per_pin = 4;
    const uint8_t af_mask = BIT_MASK_CREATE(af_bits_per_pin);
    const uint8_t af_pos = (handle->pin_index % pins_per_af_reg)* af_bits_per_pin;

    // clear then set
    uint32_t value = *af_reg;
    value &= ~(af_mask << af_pos);
    value |= (af_mode & af_mask) << (af_pos);
    *af_reg = value;
}


static inline void gpio_speed_set(gpio_pin_handle_t handle, gpio_speed_t speed) {
    static_assert(GPIO_SPEED_LOW == 0x0, "Unexpected gpio speed mode value");
    static_assert(GPIO_SPEED_MEDIUM == 0x1, "Unexpected gpio speed mode value");
    static_assert(GPIO_SPEED_FAST == 0x2, "Unexpected gpio speed mode value");
    static_assert(GPIO_SPEED_HIGH == 0x3, "Unexpected gpio speed mode value");

    const uint8_t bits_per_pin = 2;
    const uint32_t mask = BIT_MASK_CREATE(bits_per_pin);
    const uint32_t field_pos = handle->pin_index * bits_per_pin;

    uint32_t value = handle->port_ptr->OSPEEDR;
    value &= ~(mask << (field_pos));
    value |= (speed & mask) << field_pos;
    handle->port_ptr->OSPEEDR = value;
}

void gpio_pin_configure(gpio_pin_handle_t handle, gpio_pin_config_t cfg) {
    if ((handle == NULL) || (handle->port_ptr == NULL)) {
        return;
    }

    RCC->AHB1ENR |= BIT(pool_to_port(handle->pool_idx));

    gpio_mode_set(handle, cfg.mode);
    gpio_pupd_set(handle, cfg.pupd);

    if (cfg.mode == GPIO_MODE_OUTPUT) {
        gpio_output_mode_set(handle, cfg.output.output_mode);
        gpio_speed_set(handle, cfg.output.speed);
    }
    else if (cfg.mode == GPIO_MODE_ALTERNATE) {
        gpio_output_mode_set(handle, cfg.af.output_mode);
        gpio_speed_set(handle, cfg.af.speed);
        gpio_af_set(handle, cfg.af.af_mode);
    }
}

void gpio_pin_release(gpio_pin_handle_t handle) {
    if (handle
            && (handle->pool_idx < GPIO_MAX_PORTS)
            && (handle->pin_index < GPIO_MAX_PIN_PER_PORT)) {

        // mark the pin untaken/available
        gpio_port_pool[handle->pool_idx].taken_map &= ~BIT(handle->pin_index);
    }
}

bool gpio_pin_read(gpio_pin_handle_t handle) {
    if (handle == NULL) {
        return false;
    }

    return (handle->port_ptr->IDR & BIT(handle->pin_index)) ? true : false;
}

void gpio_pin_write(gpio_pin_handle_t handle, bool state) {
    if (handle == NULL) {
        return;
    }

    if (state) {
        handle->port_ptr->BSRR = BIT(handle->pin_index);
    }
    else {
        handle->port_ptr->BSRR = BIT(handle->pin_index + GPIO_MAX_PIN_PER_PORT);
    }
}
