add_library(cmsis INTERFACE)

target_include_directories(cmsis INTERFACE
    cmsis/inc
)

add_library(plat_stm32f411 STATIC
    stm32f411/src/startup_stm32f411xe.s
    stm32f411/src/system_stm32f4xx.c
)

target_include_directories(plat_stm32f411 PUBLIC
    stm32f411/inc)

target_link_libraries(plat_stm32f411 PUBLIC cmsis)

target_link_options(plat_stm32f411 PUBLIC
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
