#pragma once
#include <stdint.h>
#include "ssd1306.h"

typedef struct task_display_ctx {
    bool initialized;
    ssd1306_handle_t disp;

    uint8_t count;
} task_display_ctx_t;

void task_display(task_display_ctx_t *ctx);
task_display_ctx_t task_display_create_ctx(ssd1306_handle_t oled);
