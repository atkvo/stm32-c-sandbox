#pragma once

#ifndef ANT_H
#define ANT_H

#include "slice.h"
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
    ANT_STATUS_INVALID_TICK_RATE,
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
    uint64_t next_deadline_ms;
    void *ctx;
} ant_tcb_t __attribute__((aligned(4)));

#define ANT_REQUIRED_MEM(num_tasks) \
    ((sizeof(ant_tcb_t) * (num_tasks)))

/* @brief Initializes AntOS kernel. Required before any other calls.
 *
 * @param[in] mem - slice to memory used for tasks
 * @param[in] total_tasks - number of tasks to be managed by AntOS
 * @param[in] tick_hz - tick rate at which `ant_tick_handler` will be called
 *
 * @return ANT_STATUS_OK on success
 */
ant_status_t ant_init(slice_mutable_t task_mem, uint8_t total_tasks, uint32_t tick_hz);

/* @brief AntOS tick handler
 *
 * This should be called periodically (at the `tick_hz`) rate supplied during `ant_init`
 * If this is not called then the AntOS scheduler will not function properly
 * */
void ant_tick_handler(void);

/* @brief Registers a task to the AntOS kernel.
 *
 * @param[in] task - function pointer to task. see `ant_task_t` for requirements
 * @param[in] ctx - context ptr used on every call to `task`
 *
 * @return ANT_STATUS_OK on success
 */
ant_status_t ant_register_task(ant_task_t task, void *ctx);

/* @brief Set the next time the current task will be called
 *
 * @details This API must be called if the task is to be executed again when done
 * otherwise the task will no longer be called by the scheduler.
 *
 * @param[in] ms - the minimum time that must go by before called (in milliseconds)
 * */
void ant_task_schedule_next(uint32_t ms);

/* @brief Gets the current tick count
 *
 * @return tick count
 * */
uint64_t ant_get_tick_count();

/* @brief Converts number of ticks to ms
 *
 * @param[in] tick - number of ticks to convert
 *
 * @return duration in ms
 * */
uint64_t ant_ticks_to_ms(uint64_t tick);


/* @brief Start the ant os kernel. Should never return
 */
void ant_run(void);

#endif
