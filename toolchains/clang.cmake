set(CMAKE_SYSTEM_PROCESSOR ARM)

if(MINGW OR CYGWIN OR WIN32)
  set(UTIL_SEARCH_CMD where)
elseif(UNIX OR APPLE)
  set(UTIL_SEARCH_CMD which)
endif()

set(TOOLCHAIN clang)

execute_process(
  COMMAND ${UTIL_SEARCH_CMD} ${TOOLCHAIN}
  OUTPUT_VARIABLE BINUTILS_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CUBE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/STM32CubeF4)

set(CUBE_INCLUDES
        ${CUBE_ROOT_DIR}/Inc
        ${CUBE_ROOT_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
        ${CUBE_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ${CUBE_ROOT_DIR}/Drivers/CMSIS/Include
)

include_directories(
  ${CUBE_INCLUDES}
)

set(CMSIS_STM32F4XX_TEMPLATE ${CUBE_ROOT_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates)

set(STM32CUBEMX_GENERATED_FILES
    ${CMSIS_STM32F4XX_TEMPLATE}/gcc/startup_stm32f411xe.s
    ${CMSIS_STM32F4XX_TEMPLATE}/system_stm32f4xx.c
)

set(SOURCES
    ${SOURCES}
    ${STM32CUBEMX_GENERATED_FILES}
)

get_filename_component(ARM_TOOLCHAIN_DIR ${BINUTILS_PATH} DIRECTORY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER ${TOOLCHAIN})
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN})

set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_DIR}/arm-none-eabi-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_SIZE_UTIL ${ARM_TOOLCHAIN_DIR}/arm-none-eabi-size CACHE INTERNAL "size tool")

set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


include_directories(
  ${CUBE_INCLUDES}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_compile_options(
  -march=armv7-m
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard

  # --specs=nano.specs
  # --specs=nosys.specs
  --specs=rdimon.specs

  -fdata-sections
  -ffunction-sections
  -fno-rtti
  -Wall
  -g

  $<$<CONFIG:Debug>:-Og>
)

add_link_options(
  -T${CMAKE_SOURCE_DIR}/STM32F411CEUx_FLASH.ld
  -march=armv7-m
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  # -specs=nano.specs
  # -specs=nosys.specs
  -specs=rdimon.specs
  -lstdc++
  -lc
  -lm
  -lnosys
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
  -g
)
