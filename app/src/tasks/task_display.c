#include "task_display.h"
#include "antos.h"
#include "framebuffer.h"
#include "slice.h"

#include <stdio.h>
#include <string.h>

static void splash(ssd1306_handle_t oled);
static void write_number(slice_mutable_t s, size_t col, size_t row, size_t num);
static void draw_stat(slice_mutable_t fb, uint8_t row, const char* label, uint32_t value);
static ant_task_status_t task_init(task_display_ctx_t *ctx);

task_display_ctx_t task_display_create_ctx(ssd1306_handle_t oled) {
    return (task_display_ctx_t) {
        .initialized = false,
        .count = 0,
        .disp = oled
    };
}


ant_task_status_t task_display(task_display_ctx_t *ctx) {
    if (!ctx->initialized) {
        return task_init(ctx);
    }

    draw_stat(ctx->disp->ram, 0, "uptime", ant_ticks_to_ms(ant_get_tick_count()));
    draw_stat(ctx->disp->ram, 1, "count", ctx->count);
    draw_stat(ctx->disp->ram, 2, "press", *ctx->press_count);
    draw_stat(ctx->disp->ram, 3, "humid", *ctx->humidity);

    ctx->count++;

    ssd1306_update_nb(ctx->disp);

    ant_task_schedule_next(500);
    return ANT_TASK_OK;
}

static ant_task_status_t task_init(task_display_ctx_t *ctx) {
    ssd1306_init(ctx->disp);
    ctx->count = 0;
    ctx->initialized = true;
    splash(ctx->disp);

    fb_clear(ctx->disp->ram);
    ant_task_schedule_next(1000);

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
        MAX_DIGITS = 4,
        MAX_VALUE = 999,
    };
    while (num > (MAX_VALUE)) {
        num = num / MAX_VALUE;
    }

    char digits[MAX_DIGITS + 1];
    snprintf(digits, sizeof(digits), "%04d", num);
    fb_write_ascii(s, col, row, slice_view((uint8_t*)digits, sizeof(digits) - 1));
}

static void draw_stat(slice_mutable_t fb, uint8_t row, const char* label, uint32_t value) {
    size_t label_len = strlen(label);
    fb_write_ascii(fb, 0, row, slice_view((const uint8_t*)label, label_len));

    const uint8_t number_stat_col = 64;
    write_number(fb, number_stat_col, row, value);
}

