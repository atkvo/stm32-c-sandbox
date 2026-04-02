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

timer_handle_t timer_get_handle(uint8_t timer_index);

void timer_init(timer_handle_t, const timer_cfg_t cfg);
void timer_start(timer_handle_t);
void timer_stop(timer_handle_t);

void timer_int_enable(timer_handle_t);
void timer_int_disable(timer_handle_t);

uint32_t timer_read(timer_handle_t);
void timer_register_callback(timer_handle_t, timer_callback_t cb, void *user_data);

// @todo: move this to a `platform.h` interface instead
uint32_t timer_system_clock_freq();
void timer_system_clock_update();

#endif
