cmake_minimum_required(VERSION 3.10)

project(
    stm32-test
    VERSION 1.0
    LANGUAGES C ASM
)

set(EXECUTABLE ${PROJECT_NAME}.elf)

set(MCU stm32f411)

add_subdirectory(platform)
add_subdirectory(lib)
# add_subdirectory(app)

set(SOURCES
    ${SOURCES}
    app/main.c
)

# set(CXXFLAGS  " -Wall -std=c++17")
# set(CFLAGS  " -Wall -std=c23")
# set(CMAKE_C_STANDARD 23)
# set(CMAKE_CXX_FLAGS "${CXXFLAGS}")
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${EXECUTABLE} ${SOURCES})

# target_compile_definitions(${EXECUTABLE} PRIVATE
#     # -DUSE_HAL_DRIVER
#     -DSTM32F411xE
# )

target_link_libraries(${EXECUTABLE} PUBLIC
    cmsis
    ${MCU}_plat
    hal
)

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE})

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
