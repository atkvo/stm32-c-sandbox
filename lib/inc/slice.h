#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct slice {
    const uint8_t *ptr;
    const size_t len;
} slice_t;

typedef struct slice_mutable {
    uint8_t *ptr;
    size_t len;
} slice_mutable_t;
