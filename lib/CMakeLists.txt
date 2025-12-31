# set(MCU stm32f411)

set(MCU_HAL ${MCU}_hal)
set(MCU_PLATFORM ${MCU}_plat)

add_library(hal INTERFACE)

add_library(${MCU_HAL} STATIC
    hal/${MCU}/gpio.c
    hal/${MCU}/i2c.c
)

add_library(driver STATIC
    driver/ssd1306/src/ssd1306.c
)

target_include_directories(${MCU_HAL} PUBLIC
    inc/
    hal/inc
)

target_include_directories(driver PUBLIC
    inc/
    driver/ssd1306/inc
)

target_link_libraries(${MCU_HAL} PRIVATE
    cmsis
    ${MCU_PLATFORM}
)

target_link_libraries(hal INTERFACE
    ${MCU_HAL}
)

target_link_libraries(driver PUBLIC
    hal
)
