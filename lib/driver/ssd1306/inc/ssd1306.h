#pragma once

#include <stddef.h>
#include <stdint.h>

#include "i2c.h"
#include "slice.h"

#define SSD1306_PAGE_HEIGHT (8)
#define SSD1306_GET_RAM_SIZE(cols, rows) (((cols) * (rows) + (SSD1306_PAGE_HEIGHT - 1)) / (SSD1306_PAGE_HEIGHT))

typedef struct ssd1306_display_info {
    size_t columns;
    size_t rows;
} ssd1306_display_info_t;

typedef struct ssd1306_ctx {
    i2c_handle_t i2c;
    ssd1306_display_info_t disp_info;
    uint8_t dev_addr;
    slice_mutable_t ram;
} ssd1306_ctx_t;

typedef struct ssd1306_ctx* ssd1306_handle_t;

ssd1306_handle_t ssd1306_handle_create(
        ssd1306_ctx_t *ctx,
        i2c_handle_t i2c_handle,
        uint8_t dev_addr,
        ssd1306_display_info_t disp_info,
        slice_mutable_t display_buffer);

void ssd1306_init(ssd1306_handle_t);

void ssd1306_update(ssd1306_handle_t);

void ssd1306_display_state_set(ssd1306_handle_t, bool on);
