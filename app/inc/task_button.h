#pragma once

#include <stdint.h>
#include "gpio.h"

typedef enum {
    BTN_TASK_INIT = 0,
    BTN_TASK_WAIT_FOR_PRESS,
    BTN_TASK_WAIT_DEBOUNCE,
    BTN_TASK_CHECK_FINAL_STATE,
} btn_task_state_t;

typedef struct {
    btn_task_state_t state;
    gpio_pin_handle_t button;
    bool last_state;
    uint32_t last_pressed;
    uint32_t press_count;
} button_task_ctx_t;

button_task_ctx_t task_button_create_ctx(gpio_pin_handle_t btn);
void task_read_button(button_task_ctx_t *ctx);

