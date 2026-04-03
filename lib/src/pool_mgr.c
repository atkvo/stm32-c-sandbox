#include "pool_mgr.h"

static uint32_t resource_to_index(pool_manager_t *pool, void *obj) {
    return ((obj - pool->buffer) / pool->obj_size);
}

static void* index_to_resource(pool_manager_t *pool, uint32_t idx) {
    return pool->buffer + (pool->obj_size * (idx + 1));
}

pool_manager_t pool_mgr_init(void *pool_buffer, size_t obj_size, uint32_t capacity) {
    return (pool_manager_t) {
        .buffer = pool_buffer,
        .obj_size = obj_size,
        .capacity = capacity,
        .bitmap = POOL_MGR_INIT_STATE,
    };
}

void pool_mgr_return(pool_manager_t *pool, void *object) {
    if (pool == NULL) {
        return;
    }

    resource_free(&pool->bitmap, resource_to_index(pool, object));
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
