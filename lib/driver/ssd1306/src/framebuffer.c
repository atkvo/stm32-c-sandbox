#include <assert.h>
#include <string.h>

#include "framebuffer.h"
#include "font.h"

static size_t cursor_from_coord(size_t col, size_t page) {
    return (col) + (page * FB_COLUMNS);
}

void fb_clear(slice_mutable_t ram) {
    for (size_t i = 0; i < (ram.len); i++) {
        ram.ptr[i] = 0;
    }
}

void fb_set_pixel(slice_mutable_t ram, size_t col, size_t page, bool state) {
    for (size_t i = 0; i < (ram.len); i++) {
        ram.ptr[i] = 0;
    }
}

void fb_set_bitmap(slice_mutable_t ram, size_t col, size_t page, slice_t bitmap) {
}

void fb_write_ascii(slice_mutable_t ram, size_t col, size_t page, slice_t src) {
    const uint8_t *msg = src.ptr;
    size_t cursor = cursor_from_coord(col, page);
    for (size_t i = 0; i < src.len; i++) {

        font_copy_char(
                slice_mut_view(&ram.ptr[cursor], ram.len - cursor),
                msg[i]);

        cursor += FONT_PX_WIDTH;

        // Add 1 pixel of "kerning" (space) so letters don't touch
        static_assert(FONT_PX_KERNING == 1, "Logic for kerning != 1 is not implemented!\n");
        if (cursor < ram.len) {
            ram.ptr[cursor++] = 0x00;
        }
    }
}

void fb_copy(slice_mutable_t ram, slice_t src) {
    const size_t len = ram.len < src.len ? ram.len : src.len;
    for (size_t i = 0; i < len; i++) {
        ram.ptr[i] = src.ptr[i];
    }
}
