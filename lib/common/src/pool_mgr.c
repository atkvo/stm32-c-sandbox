#include "pool_mgr.h"

#define POOL_INVALID_RESOURCE -1
static int32_t resource_to_index(pool_manager_t *pool, void *obj) {
    if (obj < pool->buffer) {
        return POOL_INVALID_RESOURCE;
    }

    int32_t idx = ((obj - pool->buffer) / pool->obj_size);
    if (idx > pool->capacity) {
        return POOL_INVALID_RESOURCE;
    }
    else {
        return idx;
    }
}

static void* index_to_resource(pool_manager_t *pool, uint32_t idx) {
    return pool->buffer + (pool->obj_size * (idx));
}

pool_manager_t pool_mgr_init(void *pool_buffer, size_t obj_size, uint32_t capacity) {
    return (pool_manager_t) {
        .buffer = pool_buffer,
        .obj_size = obj_size,
        .capacity = capacity,
        .bitmap = POOL_MGR_INIT_STATE,
    };
}

void* pool_mgr_take(pool_manager_t *pool, uint8_t index) {
    if ((pool == NULL) || (index >= pool->capacity)) {
        return NULL;
    }

    if (resource_is_taken(&pool->bitmap, index)) {
        return NULL;
    }

    resource_take(&pool->bitmap, index);
    return index_to_resource(pool, index);
}

void pool_mgr_release(pool_manager_t *pool, void *object) {
    if (pool == NULL) {
        return;
    }

    int32_t idx = resource_to_index(pool, object);
    if (idx == POOL_INVALID_RESOURCE) {
        return;
    }

    resource_free(&pool->bitmap, resource_to_index(pool, object));
}
