#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "slice.h"

// @todo make configurable (ideally at compile time)
enum {
    FB_COLUMNS = 128,
    FB_PAGES = 8,
};

typedef struct disp_bitmap {
    slice_t bits;
    size_t width;
    size_t height;
} fb_bitmap_t;

void fb_clear(slice_mutable_t ram);
void fb_set_pixel(slice_mutable_t ram, size_t col, size_t page, bool state);
void fb_set_bitmap(slice_mutable_t ram, size_t col, size_t page, slice_t bitmap);
void fb_write_ascii(slice_mutable_t ram, size_t col, size_t page, slice_t src);
void fb_copy(slice_mutable_t ram, slice_t src);
