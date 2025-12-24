#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "stm32f411xe.h"

typedef struct gpio_port_ctx {
    /**
     * @brief port_index
     * This can be used to calculate the proper MMIO GPIO register
     * block field even if PORT H is used
     */
    int port_index;

    /**
     * @brief pointer to MMIO GPIO field
     */
    volatile GPIO_TypeDef *port_ptr;
} gpio_context_t;


#define GPIO_MAX_PORTS (6)

typedef struct {
    bool taken;
    gpio_context_t ctx;
} gpio_context_resource_t;

static gpio_context_resource_t gpio_port_pool[GPIO_MAX_PORTS] = { {0, {0}} };

static uint8_t port_to_index(gpio_port_t port) {
    if (port == GPIO_PORT_H) {
        port = GPIO_MAX_PORTS - 1;
    }

    return port;
}

static gpio_port_t index_to_port(uint8_t port) {
    if ((port == GPIO_MAX_PORTS) - 1) {
        return GPIO_PORT_H;
    }
    else {
        return port;
    }
}

static inline bool is_port_valid(gpio_port_t port) {
    const uint8_t valid_port_mask = 0x9F;
    return (valid_port_mask & (1 << port)) ? true : false;
}

gpio_port_handle_t gpio_handle_acquire(gpio_port_t port) {
    if (!is_port_valid(port)) {
        return NULL;
    }

    uint8_t port_index = port_to_index(port);

    if (gpio_port_pool[port_index].taken == false) {
        gpio_port_pool[port_index].taken = true;
        gpio_port_pool[port_index].ctx.port_index = port_index;
        gpio_port_pool[port_index].ctx.port_ptr = GPIOA + (sizeof(GPIO_TypeDef) * port_index);
        return &gpio_port_pool[port].ctx;
    }

    return NULL;
}

void gpio_init(gpio_port_handle_t handle) {
    if (handle == NULL) {
        return;
    }

    handle->port_ptr = GPIOA + (sizeof(GPIO_TypeDef) * handle->port_index);
}

void gpio_handle_release(gpio_port_handle_t handle) {
    if (handle && (handle->port_index < GPIO_MAX_PORTS)) {
        gpio_port_pool[handle->port_index].taken = false;
    }
}
