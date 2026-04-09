#include <string.h>

#include "antos.h"

typedef struct {
    uint8_t max_tasks;
    uint8_t current_task_count;
    uint8_t active_task_idx;
    ant_tcb_t* tasks;
    uint32_t tick_hz;

    volatile uint64_t tick_count;
} ant_kernel_t;

static ant_kernel_t kernel = { .max_tasks = 0 };
#define ANT_WAITING_FOR_SCHEDULE (UINT32_MAX)

static bool is_kernel_initialized() {
    return kernel.max_tasks != 0;
}

void ant_tick_handler(void) {
    kernel.tick_count++;
}

ant_status_t ant_init(slice_mutable_t mem, uint8_t total_tasks, uint32_t tick_hz) {
    if (is_kernel_initialized()) {
        return ANT_STATUS_ALREADY_INITIALIZED;
    }

    if (total_tasks == 0) {
        return ANT_STATUS_INVALID_TASK_COUNT;
    }

    if (mem.len < ANT_REQUIRED_MEM(total_tasks)) {
        return ANT_STATUS_FAIL;
    }

    if (tick_hz == 0) {
        return ANT_STATUS_INVALID_TICK_RATE;
    }

    memset(&kernel, 0, sizeof(kernel));
    memset(mem.ptr, 0, mem.len);

    kernel.max_tasks = total_tasks;
    kernel.current_task_count = 0;
    kernel.tick_hz = tick_hz;

    kernel.tasks = (ant_tcb_t*)mem.ptr;

    return ANT_STATUS_OK;
}

ant_status_t ant_register_task(ant_task_t fn, void *ctx) {
    if (!is_kernel_initialized()) {
        return ANT_STATUS_FAIL;
    }

    if (kernel.current_task_count >= kernel.max_tasks) {
        return ANT_STATUS_OOM;
    }

    ant_tcb_t *t = &kernel.tasks[kernel.current_task_count];
    t->ctx = ctx;
    t->fn = fn;
    t->next_deadline_ms = 0;

    kernel.current_task_count++;

    return ANT_STATUS_OK;
}

uint64_t ant_ticks_to_ms(uint64_t tick) {
    return ((tick * 1000) + (kernel.tick_hz - 1)) / kernel.tick_hz;
}

uint64_t ant_ms_to_ticks(uint64_t ms) {
    return (ms * kernel.tick_hz) / (1000);
}

static void ant_handle_task_exec(ant_tcb_t *t) {
    if (t->next_deadline_ms == ANT_WAITING_FOR_SCHEDULE) {
        return;
    }

    const uint32_t current_ms = ant_ticks_to_ms(ant_get_tick_count());
    if (current_ms >= t->next_deadline_ms) {
        /* Task must call ant_task_schedule_next before returning */
        t->next_deadline_ms = ANT_WAITING_FOR_SCHEDULE;
        t->state = t->fn(t->ctx);
    }
}

void ant_run() {
    kernel.active_task_idx = 0;
    while (1) {
        ant_tcb_t *task = &kernel.tasks[kernel.active_task_idx];
        ant_handle_task_exec(task);

        kernel.active_task_idx++;
        if (kernel.active_task_idx == kernel.current_task_count) {
            kernel.active_task_idx = 0;
        }
    }
}

void ant_task_schedule_next(uint32_t ms) {
    if (kernel.active_task_idx >= kernel.current_task_count) {
        return;
    }

    ant_tcb_t *t = &kernel.tasks[kernel.active_task_idx];
    uint32_t now = ant_ticks_to_ms(ant_get_tick_count());

    if (t->next_deadline_ms == ANT_WAITING_FOR_SCHEDULE || t->next_deadline_ms == 0) {
        t->next_deadline_ms = now + ms;
    }
    else {
        t->next_deadline_ms += ms;
    }

    /* If there was a large delay before this task then resync the deadline */
    if (t->next_deadline_ms < now) {
        t->next_deadline_ms = now + ms;
    }
}

uint64_t ant_get_tick_count() {
    return kernel.tick_count;
}
