#include "antos.h"
#include <string.h>

typedef struct {
    uint8_t max_tasks;
    uint8_t current_task_count;
    ant_tcb_t* tasks;
} ant_kernel_t;

static ant_kernel_t kernel = { .max_tasks = 0 };

static bool is_kernel_initialized() {
    return kernel.max_tasks != 0;
}

extern void delay(const uint32_t count);

ant_status_t ant_init(slice_mutable_t mem, uint8_t total_tasks) {
    if (is_kernel_initialized()) {
        return ANT_STATUS_ALREADY_INITIALIZED;
    }

    if (total_tasks == 0) {
        return ANT_STATUS_INVALID_TASK_COUNT;
    }

    if (mem.len < ANT_REQUIRED_MEM(total_tasks)) {
        return ANT_STATUS_FAIL;
    }

    memset(&kernel, 0, sizeof(kernel));
    memset(mem.ptr, 0, mem.len);

    kernel.max_tasks = total_tasks;
    kernel.current_task_count = 0;

    kernel.tasks = (ant_tcb_t*)mem.ptr;

    return ANT_STATUS_OK;
}

ant_status_t ant_register(ant_task_t fn, void *ctx) {
    if (!is_kernel_initialized()) {
        return ANT_STATUS_FAIL;
    }

    if (kernel.current_task_count >= kernel.max_tasks) {
        return ANT_STATUS_OOM;
    }

    kernel.tasks[kernel.current_task_count].ctx = ctx;
    kernel.tasks[kernel.current_task_count].fn = fn;

    kernel.current_task_count++;

    return ANT_STATUS_OK;
}

void ant_run() {
    size_t task_index = 0;
    while (1) {
        ant_tcb_t *task = &kernel.tasks[task_index];

        // @todo: get return
        task->fn(task->ctx);

        task_index++;
        if (task_index == kernel.current_task_count) {
            task_index = 0;
        }

        // @todo: do not use this!
        delay(1000 * 100);
    }
}
