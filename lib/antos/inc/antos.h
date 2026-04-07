#pragma once

#ifndef ANT_H
#define ANT_H

#include "slice.h"
#include "timer.h"
#include <stdint.h>

/** @brief AntOS task status
 */
typedef enum {
    ANT_TASK_OK,
    ANT_TASK_IDLE,
    ANT_TASK_FINISHED,
    ANT_TASK_ERROR
} ant_task_status_t;

typedef enum {
    ANT_STATUS_OK,
    ANT_STATUS_FAIL,
    ANT_STATUS_INVALID_TASK_COUNT,
    ANT_STATUS_ALREADY_INITIALIZED,
    ANT_STATUS_OOM,
} ant_status_t;

/**
 * @brief AntOS task function signature.
 *
 * @details Tasks in AntOS are run-to-completion. To ensure system stability,
 * task implementations must adhere to the following:
 *   * **Non-blocking:** Tasks must return after every execution. `ant_yield()`
 * is not supported.
 *   * **Execution Time:** Tasks must minimize execution time. Avoid busy-waits
 * (e.g., `delay_ms`) to prevent starving other tasks.
 *
 * @param[in] context User-defined pointer passed to the task on every execution.
 * @return ant_task_status_t Status code indicating task success or request for state change.
 */
typedef ant_task_status_t (*ant_task_t)(void* context);

/**
 * @brief AntOS Task Control Block
 */
typedef struct ant_tcb {
    ant_task_t fn;
    ant_task_status_t state;
    uint32_t next_run_ms;
    uint32_t last_run_count;
    uint32_t period_ms;
    void *ctx;
} ant_tcb_t __attribute__((aligned(4)));

#define ANT_REQUIRED_MEM(num_tasks) \
    ((sizeof(ant_tcb_t) * (num_tasks)))

/* @brief Initializes AntOS kernel. Required before any other calls.
 *
 * @param mem - slice to memory used for tasks
 * @param total_tasks - number of tasks to be managed by AntOS
 *
 * @return ANT_STATUS_OK on success
 */
ant_status_t ant_init(slice_mutable_t task_mem, uint8_t total_tasks);

/* @brief Registers a task to the AntOS kernel.
 *
 * @param task - function pointer to task. see `ant_task_t` for requirements
 * @param ctx - context ptr used on every call to `task`
 *
 * @return ANT_STATUS_OK on success
 */
ant_status_t ant_register(ant_task_t task, void *ctx);

/* @brief Start the ant os kernel. Should never return
 */
void ant_run();

void ant_delay_next(uint32_t c);

void ant_tick_handler(void);

const timer_handle_t ant_get_system_timer();

uint64_t ant_get_tick_count();

#endif
