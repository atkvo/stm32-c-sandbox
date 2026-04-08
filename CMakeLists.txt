cmake_minimum_required(VERSION 3.10)

project(
    stm32-test
    VERSION 1.0
    LANGUAGES C ASM
)

set(EXECUTABLE ${PROJECT_NAME}.elf)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(platform)
add_subdirectory(lib)
add_subdirectory(app)

