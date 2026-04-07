#include "task_heartbeat.h"
#include "antos.h"

heartbeat_task_ctx_t task_heartbeat_create_ctx(gpio_pin_handle_t led, uint32_t delay) {
    heartbeat_task_ctx_t hb_ctx;
    hb_ctx.led_pin = led;
    hb_ctx.delay = delay;

    return hb_ctx;
}

void task_heartbeat(heartbeat_task_ctx_t *ctx) {
    gpio_pin_write(ctx->led_pin, ctx->state);
    ctx->state = !ctx->state;

    ant_delay_next(ctx->delay);
}

