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

static inline slice_mutable_t slice_mut_view(uint8_t *buffer, size_t len) {
    return (slice_mutable_t) { .ptr = buffer, .len = len };
}

static inline slice_t slice_view(const uint8_t *buffer, size_t len) {
    return (slice_t) { .ptr = buffer, .len = len };
}
