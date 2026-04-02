#pragma once

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>

typedef struct {
    uint32_t hz;
    bool periodic;
} timer_cfg_t;

struct timer_ctx;
typedef struct timer_ctx *timer_handle_t;

typedef void (*timer_callback_t)(void *data);

/** @brief Get timer handle
 *
 * @details Does not handle any ownership to the timer - any caller
 * can get a handle to the timer. It is up to the caller to manage
 * ownership unlike the GPIO.
 *
 * @param[in] timer_number - timer to get (1 based) (TIM1 = 1, TIM2 = 2, etc)
 *
 * @return timer_handle_t or NULL if not supported
 * */
timer_handle_t timer_get_handle(uint8_t timer_number);

void timer_init(timer_handle_t, const timer_cfg_t cfg);
void timer_start(timer_handle_t);
void timer_stop(timer_handle_t);

void timer_int_enable(timer_handle_t);
void timer_int_disable(timer_handle_t);

uint32_t timer_read(timer_handle_t);
void timer_register_callback(timer_handle_t, timer_callback_t cb, void *user_data);

#endif
