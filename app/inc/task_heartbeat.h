#pragma once

#include <stdbool.h>
#include "gpio.h"

typedef struct {
    gpio_pin_handle_t led_pin;
    uint32_t delay;
    bool state;
} heartbeat_task_ctx_t;

heartbeat_task_ctx_t task_heartbeat_create_ctx(gpio_pin_handle_t led, uint32_t delay);
void task_heartbeat(heartbeat_task_ctx_t *ctx);
