#include "task_button.h"
#include "antos.h"

button_task_ctx_t task_button_create_ctx(gpio_pin_handle_t btn) {
    button_task_ctx_t button_ctx = { 0 };
    button_ctx.state = BTN_TASK_INIT;
    button_ctx.button = btn;
    button_ctx.press_count = 0;

    return button_ctx;
}

ant_task_status_t task_read_button(button_task_ctx_t *ctx) {
    enum {
        DEBOUNCE_TIME = 80,
        IS_PRESSED = false,
    };

    uint32_t next_deadline_ms = 5;
    switch (ctx->state) {
        case BTN_TASK_INIT:
            ctx->state = BTN_TASK_WAIT_FOR_PRESS;
            break;

        case BTN_TASK_WAIT_FOR_PRESS:
            if (gpio_pin_read(ctx->button) == IS_PRESSED) {
                ctx->state = BTN_TASK_DEBOUNCE_CHECK;
                next_deadline_ms = DEBOUNCE_TIME;
            }
            break;

        case BTN_TASK_DEBOUNCE_CHECK:
            if (gpio_pin_read(ctx->button) == IS_PRESSED) {
                ctx->press_count++;
            }

            ctx->state = BTN_TASK_WAIT_FOR_PRESS;
            break;
    }

    ant_task_schedule_next(next_deadline_ms);

    return ANT_TASK_OK;
}
