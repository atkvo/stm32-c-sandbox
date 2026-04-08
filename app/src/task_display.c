#include "task_display.h"
#include "framebuffer.h"

#include <stdio.h>
#include <string.h>

static void splash(ssd1306_handle_t oled);
static void write_number(slice_mutable_t s, size_t col, size_t row, size_t num);

task_display_ctx_t task_display_create_ctx(ssd1306_handle_t oled) {
    return (task_display_ctx_t) {
        .initialized = false,
        .count = 0,
        .disp = oled
    };
}

ant_task_status_t task_display(task_display_ctx_t *ctx) {
    if (!ctx->initialized) {
        ssd1306_init(ctx->disp);
        ctx->count = 0;
        ctx->initialized = true;
        splash(ctx->disp);

        fb_clear(ctx->disp->ram);
        ant_task_schedule_next(1000);

        return ANT_TASK_OK;
    }

    write_number(ctx->disp->ram, 0, 0, ctx->count);
    write_number(ctx->disp->ram, 0, 4, *ctx->press_count);

    ctx->count++;
    if (ctx->count >= 255) { ctx->count = 0; }

    // should have a flag or something to update?
    ssd1306_update_nb(ctx->disp);

    ant_task_schedule_next(500);
    return ANT_TASK_OK;
}

static void splash(ssd1306_handle_t oled) {
    const uint8_t face[] = "(^_^)~";
    const uint8_t msg[] = "hello!";

    fb_clear(oled->ram);
    const size_t face_col =  oled->disp_info.columns / 2;
    const size_t face_page = (oled->disp_info.rows / 8) / 2;
    fb_write_ascii(oled->ram,
            face_col,
            face_page,
            slice_view(face, strlen(face)));

    fb_write_ascii(oled->ram,
            face_col + 28,
            face_page - 2,
            slice_view(msg, strlen(msg)));

    ssd1306_update(oled);
}

static void write_number(slice_mutable_t s, size_t col, size_t row, size_t num) {
    enum {
        MAX_DIGITS = 8,
        MAX_VALUE = 9999,
    };
    char digits[MAX_DIGITS + 1];
    if (num > (MAX_VALUE)) {
        num = MAX_VALUE;
    }

    snprintf(digits, sizeof(digits), "%08d", num);
    fb_write_ascii(s, col, row, slice_view((uint8_t*)digits, sizeof(digits) - 1));
}
