#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include "timer.h"
#include "platform.h"
#include "stm32f411xe.h"

typedef struct timer_ctx {
    volatile TIM_TypeDef *reg;
    timer_callback_t cb;
    void *user_data;
    uint32_t counter_resolution;
} timer_ctx_t;

#define TIMER_MAX_COUNT (8)

/* Possible timers:
 * TIM 1    : 16 bit
 * TIM 2,5  : 32 bit
 * TIM 3,4  : 16 bit
 * TIM 9    : 16 bit
 * TIM 10,11: 16 bit
 */
static timer_ctx_t timer_ctx_pool[TIMER_MAX_COUNT] = { 0 };

enum {
    TIMER_2_POOL_INDEX = 1,
    TIMER_3_POOL_INDEX = 2,
    TIMER_4_POOL_INDEX = 3,
    TIMER_5_POOL_INDEX = 4,
};

#if 0
void TIM3_IRQHandler(void) {
}

void TIM4_IRQHandler(void) {
}

void TIM5_IRQHandler(void) {
}
#endif


timer_handle_t timer_get_handle(uint8_t timer_number) {
    switch (timer_number) {
        case 1:
            // @todo: add support for advanced timer 1
            return NULL;
        case 2:
        case 3:
        case 4:
        case 5:
            return &timer_ctx_pool[timer_number - 1];
        default:
            return NULL;
    };

    return NULL;
}

void timer_init(timer_handle_t ctx, const timer_cfg_t cfg) {
    if (ctx == NULL) {
        return;
    }

    ctx->reg = TIM2;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM2RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);

    uint32_t sys_freq = plat_system_clock_freq();

    {
        /* From Gemini */
        /* * Strategy: Set PSC so the timer increments every 1 microsecond.
         * This makes the ARR value equal to the period in microseconds.
         */
        uint32_t psc_val = (sys_freq / 1000000) - 1;
        uint32_t arr_val = (1000000 / cfg.hz) - 1;

        // Safety checks for 16-bit registers (like TIM2 on some chips, or TIM3/4)
        if (psc_val > 0xFFFF) psc_val = 0xFFFF;
        // @todo: handle 16 vs 32 bit timer
        // if (arr_val > 0xFFFF) arr_val = 0xFFFF;

        ctx->reg->PSC = (uint16_t)psc_val;
        ctx->reg->ARR = arr_val;
    }

    // Force a shadow register update so PSC/ARR take effect immediately
    ctx->reg->EGR |= TIM_EGR_UG;
}

void timer_start(timer_handle_t ctx) {
    if (ctx) {
        ctx->reg->CR1 |= TIM_CR1_CEN;
    }
}

void timer_stop(timer_handle_t ctx) {
    if (ctx) {
        ctx->reg->CR1 &= ~TIM_CR1_CEN;
    }
}

void timer_int_enable(timer_handle_t ctx) {
    if (ctx) {
        NVIC_EnableIRQ(TIM2_IRQn);
        ctx->reg->DIER |= TIM_DIER_UIE;
    }
}

void timer_int_disable(timer_handle_t ctx) {
    if (ctx) {
        NVIC_DisableIRQ(TIM2_IRQn);
        ctx->reg->DIER &= ~TIM_DIER_UIE;
    }
}

uint32_t timer_read(timer_handle_t ctx) {
    if (ctx) {
        return ctx->reg->CNT;
    }

    return 0;
}

void timer_register_callback(timer_handle_t ctx, timer_callback_t cb, void *user_data) {
    if (ctx) {
        ctx->cb = cb;
        ctx->user_data = user_data;
    }
}

inline bool timer_int_update_flag_check(timer_handle_t ctx) {
    return (ctx->reg->SR) & TIM_SR_UIF;
}

void timer_int_update_flag_clear(timer_handle_t ctx) {
    ctx->reg->SR &= ~TIM_SR_UIF;
}

void timer_int_callback_exec(timer_handle_t ctx) {
    if (ctx->cb) {
        ctx->cb(ctx->user_data);
    }
}
