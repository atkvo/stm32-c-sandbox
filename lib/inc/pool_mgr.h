#pragma once

#include <stddef.h>
#ifndef POOL_MGR_H
#define POOL_MGR_H

/* @brief Pool manager can be used to easily manage generic pool
 *
 * @details In a pool each resource can be taken only once and will no
 * longer be available until returned.
 **/

#include "bit_utils.h"
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t resource_map_t;
static_assert(sizeof(resource_map_t) == sizeof(uint32_t),
        "API assumes resource is a 32 bit type");

/* Initial state is that all resources are not taken */
#define POOL_MGR_INIT_STATE (0)
#define POOL_MGR_ALL_TAKEN  (UINT32_MAX)

typedef struct {
    void *buffer;
    size_t obj_size;
    uint32_t capacity;
    resource_map_t bitmap;
} pool_manager_t;

#define POOL_MGR_INIT(mem, obj_type, cap) \
{                                         \
    .buffer = (mem),                      \
    .obj_size = sizeof(obj_type),         \
    .capacity = (cap),                    \
    .bitmap = POOL_MGR_INIT_STATE         \
}

pool_manager_t pool_mgr_init(void *pool_buffer, size_t obj_size, uint32_t capacity);

void* pool_mgr_take(pool_manager_t *pool, uint8_t index);

void pool_mgr_release(pool_manager_t *pool, void *object);

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
    *resource = POOL_MGR_ALL_TAKEN;
}

static inline void resource_free_all(resource_map_t *resource) {
    *resource = POOL_MGR_INIT_STATE;
}

#endif
