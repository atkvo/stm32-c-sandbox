#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "slice.h"

enum
{
    DISP_BUFF_PAGES = 8,
    DISP_BUFF_COLS = 128,
};

typedef struct disp_bitmap {
    slice_t bits;
    size_t width;
    size_t height;
} disp_bitmap_t;

void disp_clear(slice_mutable_t ram);
void disp_set_pixel(slice_mutable_t ram, size_t col, size_t page, bool state);
void disp_set_bitmap(slice_mutable_t ram, size_t col, size_t page, slice_t bitmap);
void disp_write_ascii(slice_mutable_t ram, size_t col, size_t page, slice_t src);
void disp_copy(slice_mutable_t ram, slice_t src);

