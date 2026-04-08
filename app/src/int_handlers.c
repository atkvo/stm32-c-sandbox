#include "timer.h"
#include "antos.h"
#include "app.h"

timer_handle_t app_os_timer = NULL;

void TIM2_IRQHandler(void) {
    if (app_os_timer) {
        if (timer_int_update_flag_check(app_os_timer)) {
            ant_tick_handler();
            timer_int_update_flag_clear(app_os_timer);
        }
    }
};

