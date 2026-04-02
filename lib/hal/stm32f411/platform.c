#include "platform.h"
#include "system_stm32f4xx.h"

void plat_init() {
    SystemInit();
}

void plat_system_core_clock_update() {
    SystemCoreClockUpdate();
}

uint32_t plat_system_clock_freq() {
    return SystemCoreClock;
}
