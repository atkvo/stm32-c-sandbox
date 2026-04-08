add_library(hal INTERFACE)

target_include_directories(hal INTERFACE
    inc/
)

target_link_libraries(hal INTERFACE
    common
)

add_library(stm32f411 STATIC
    port/stm32f411/dma.c
    port/stm32f411/gpio.c
    port/stm32f411/i2c.c
    port/stm32f411/platform.c
    port/stm32f411/timer.c
)

target_link_libraries(stm32f411 PUBLIC
    cmsis
    common
    hal
    plat_stm32f411
)
