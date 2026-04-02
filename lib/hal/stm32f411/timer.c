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
} timer_ctx_t;

static timer_ctx_t _tim2;

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        if (_tim2.cb) {
            _tim2.cb(_tim2.user_data);
        }

        TIM2->SR &= ~TIM_SR_UIF;
    }
};

timer_handle_t timer_get_handle(uint8_t timer_index) {
    // @todo: implement timer pool logic to get TIM1/2/3 etc
    // like how we do for I2C/GPIO
    return &_tim2;
}

void timer_init(timer_handle_t ctx, const timer_cfg_t cfg) {
    // @todo: temporary hardcode to only support TIM2
    ctx = &_tim2;
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
    ctx->reg->CR1 |= TIM_CR1_CEN;
}

void timer_stop(timer_handle_t ctx) {
    ctx->reg->CR1 &= ~TIM_CR1_CEN;
}

void timer_int_enable(timer_handle_t ctx) {
    NVIC_EnableIRQ(TIM2_IRQn);
    ctx->reg->DIER |= TIM_DIER_UIE;
}

void timer_int_disable(timer_handle_t ctx) {
    NVIC_DisableIRQ(TIM2_IRQn);
    ctx->reg->DIER &= ~TIM_DIER_UIE;
}

uint32_t timer_read(timer_handle_t ctx) {
    return ctx->reg->CNT;
}

void timer_register_callback(timer_handle_t ctx, timer_callback_t cb, void *user_data) {
    ctx->cb = cb;
    ctx->user_data = user_data;
}
