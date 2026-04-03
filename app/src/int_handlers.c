#include "antos.h"
#include "timer.h"

void TIM2_IRQHandler(void) {
    const timer_handle_t timer = ant_get_system_timer();
    if (timer_int_update_flag_check(timer)) {
        ant_tick_handler();
        timer_int_update_flag_clear(timer);
    }
};

