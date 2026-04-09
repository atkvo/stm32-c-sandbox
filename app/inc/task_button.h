#pragma once

#include <stdint.h>
#include "antos.h"
#include "gpio.h"

typedef enum {
    BTN_TASK_INIT = 0,
    BTN_TASK_WAIT_FOR_PRESS,
    BTN_TASK_DEBOUNCE_CHECK,
} btn_task_state_t;

typedef struct {
    btn_task_state_t state;
    gpio_pin_handle_t button;
    bool last_state;
    uint32_t press_count;
} button_task_ctx_t;

button_task_ctx_t task_button_create_ctx(gpio_pin_handle_t btn);
ant_task_status_t task_read_button(button_task_ctx_t *ctx);

