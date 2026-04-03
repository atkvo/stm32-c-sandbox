# set(MCU stm32f411)

set(MCU_HAL ${MCU}_hal)
set(MCU_PLATFORM ${MCU}_plat)

add_library(hal INTERFACE)

add_library(${MCU_HAL} STATIC
    hal/${MCU}/gpio.c
    hal/${MCU}/i2c.c
    hal/${MCU}/platform.c
    hal/${MCU}/timer.c
)

add_library(driver STATIC
    driver/ssd1306/src/ssd1306.c
    driver/ssd1306/src/framebuffer.c
    driver/ssd1306/src/font.c
)

add_library(util STATIC
    src/pool_mgr.c
)

add_library(antos STATIC
    antos/src/antos.c
)

target_include_directories(${MCU_HAL} PUBLIC
    inc/
    hal/inc
)

target_include_directories(driver PUBLIC
    inc/
    driver/ssd1306/inc
)

target_include_directories(util PUBLIC
    inc/
)

target_include_directories(antos PUBLIC
    inc/
    antos/inc/
)

target_link_libraries(${MCU_HAL} PRIVATE
    cmsis
    ${MCU_PLATFORM}
    util
)

target_link_libraries(hal INTERFACE
    ${MCU_HAL}
)

target_link_libraries(driver PUBLIC
    hal
)

target_link_libraries(antos PUBLIC
    ${MCU_HAL}
)
