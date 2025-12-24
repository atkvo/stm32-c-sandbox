# set(MCU stm32f411)

set(MCU_HAL ${MCU}_hal)
set(MCU_PLATFORM ${MCU}_plat)

add_library(${MCU_HAL} STATIC
    hal/${MCU}/gpio.c
)

target_link_libraries(${MCU_HAL} PRIVATE
    cmsis
    ${MCU_PLATFORM}
)

target_include_directories(${MCU_HAL} PUBLIC
    hal/inc
)


add_library(hal INTERFACE)

target_link_libraries(hal INTERFACE
    ${MCU_HAL}
)
