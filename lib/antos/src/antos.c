#include <string.h>

#include "antos.h"
#include "timer.h"

typedef struct {
    uint8_t max_tasks;
    uint8_t current_task_count;
    uint8_t active_task_idx;
    ant_tcb_t* tasks;
    timer_handle_t timer;

    volatile uint64_t tick_count;
} ant_kernel_t;

static ant_kernel_t kernel = { .max_tasks = 0 };

static bool is_kernel_initialized() {
    return kernel.max_tasks != 0;
}

void ant_tick_handler(void) {
    kernel.tick_count++;
}

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

    timer_cfg_t timer_cfg = {
        .hz = 16000,
        .periodic = true,
    };

    kernel.timer = timer_take(2);
    timer_init(kernel.timer, timer_cfg);
    timer_int_enable(kernel.timer);
    timer_start(kernel.timer);

    return ANT_STATUS_OK;
}

ant_status_t ant_register(ant_task_t fn, void *ctx) {
    if (!is_kernel_initialized()) {
        return ANT_STATUS_FAIL;
    }

    if (kernel.current_task_count >= kernel.max_tasks) {
        return ANT_STATUS_OOM;
    }

    ant_tcb_t *t = &kernel.tasks[kernel.current_task_count];
    t->ctx = ctx;
    t->fn = fn;
    t->last_run_count = 0;
    t->next_run_ms = 0;
    t->period_ms = 0;

    kernel.current_task_count++;

    return ANT_STATUS_OK;
}

static uint64_t ant_get_tick() {
    return kernel.tick_count;
}

static void ant_handle_task_exec(ant_tcb_t *t) {
    const uint32_t current_count = ant_get_tick();
    const uint32_t elapsed_count = current_count - t->last_run_count;

    if (elapsed_count >= t->next_run_ms) {
        t->fn(t->ctx);
        t->last_run_count = ant_get_tick();
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

void ant_delay_next(uint32_t c) {
    if (kernel.active_task_idx >= kernel.current_task_count) {
        return;
    }

    kernel.tasks[kernel.active_task_idx].next_run_ms = c;
}

const timer_handle_t ant_get_system_timer() {
    return kernel.timer;
}
