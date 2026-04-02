#pragma once

#ifndef HAL_PLATFORM_H
#define HAL_PLATFORM_H

#include <stdint.h>

void plat_init();

void plat_system_core_clock_update();
uint32_t plat_system_clock_freq();

#endif
