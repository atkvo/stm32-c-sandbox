#pragma once

#ifndef RESOURCE_MAP_H
#define RESOURCE_MAP_H

#include "bit_utils.h"
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t resource_map_t;
static_assert(sizeof(resource_map_t) == sizeof(uint32_t),
        "API assumes resource is a 32 bit type");

/* Initial state is that all resources are not taken */
#define RESOURCE_MAP_INIT_STATE (0)
#define RESOURCE_MAP_ALL_TAKEN  (UINT32_MAX)

static inline resource_map_t resource_init(void) {
    return RESOURCE_MAP_INIT_STATE;
}

/* @warn there is no NULL checking in these APIs.
 * Caller must ensure:
 *
 * 1. resource != NULL
 * 2. index < 32
 */

static inline bool resource_is_free(const resource_map_t *resource, uint8_t index) {
    return !BIT_IS_SET(*resource, index);
}

static inline bool resource_is_taken(const resource_map_t *resource, uint8_t index) {
    return !resource_is_free(resource, index);
}

static inline void resource_take(resource_map_t *resource, uint8_t index) {
    *resource |= BIT(index);
}

static inline void resource_free(resource_map_t *resource, uint8_t index) {
    *resource &= ~BIT(index);
}

static inline void resource_take_all(resource_map_t *resource) {
    *resource = RESOURCE_MAP_ALL_TAKEN;
}

static inline void resource_free_all(resource_map_t *resource) {
    *resource = 0;
}

#endif
