#include "task_button.h"
#include "antos.h"

button_task_ctx_t task_button_create_ctx(gpio_pin_handle_t btn) {
    button_task_ctx_t button_ctx = { 0 };
    button_ctx.button = btn;
    button_ctx.press_count = 0;

    return button_ctx;
}

void task_read_button(button_task_ctx_t *ctx) {
    enum {
        DEBOUNCE_TIME = 80,
        IS_PRESSED = false,
        IS_NOT_PRESSED = true,
    };

    uint64_t elapsed_time = 0;
    switch (ctx->state) {
        case BTN_TASK_INIT:
            ctx->state = BTN_TASK_WAIT_FOR_PRESS;
            break;

        case BTN_TASK_WAIT_FOR_PRESS:
            if (gpio_pin_read(ctx->button) == IS_PRESSED) {
                ctx->state = BTN_TASK_WAIT_DEBOUNCE;
                ctx->last_pressed = ant_get_tick_count();
            }
            break;

        case BTN_TASK_WAIT_DEBOUNCE:
            {
                elapsed_time = ant_get_tick_count() - ctx->last_pressed;
                if (elapsed_time < DEBOUNCE_TIME) {
                    break;
                }
                else {
                    ctx->state = BTN_TASK_CHECK_FINAL_STATE;
                    /* fall through switch case */
                }
            }

        case BTN_TASK_CHECK_FINAL_STATE:
            if (gpio_pin_read(ctx->button) == IS_PRESSED) {
                ctx->press_count++;
            }

            ctx->state = BTN_TASK_WAIT_FOR_PRESS;
            break;
    }

    ant_task_schedule_next(5);
}
