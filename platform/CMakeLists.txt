add_library(cmsis INTERFACE)

set(MCU_PLATFORM ${MCU}_plat)

target_include_directories(cmsis INTERFACE
    cmsis/inc
)

add_library(${MCU_PLATFORM} STATIC
    ${MCU}/src/startup_stm32f411xe.s
    ${MCU}/src/system_stm32f4xx.c
)

target_include_directories(${MCU_PLATFORM} PUBLIC
    ${MCU}/inc)

target_link_libraries(${MCU_PLATFORM} PUBLIC cmsis)

target_link_options(${MCU_PLATFORM} PUBLIC
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f411/STM32F411CEUx_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    # -specs=nano.specs
    -specs=nosys.specs
    # -specs=rdimon.specs
    -lc
    -lm
    -lnosys
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -g
)
