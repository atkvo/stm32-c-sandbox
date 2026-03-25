#pragma once

#include <stddef.h>
#include <stdint.h>
#include "slice.h"

#define FONT_PX_WIDTH     (5)
#define FONT_PX_HEIGHT    (7)
#define FONT_PX_KERNING   (1)

#define FONT_CHAR_BYTES (5)

slice_t font_get_unknown_char();
void font_copy_char(slice_mutable_t dest, char c);

