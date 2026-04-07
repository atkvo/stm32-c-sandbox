
add_executable(${EXECUTABLE}
    main.c
    src/task_button.c
    src/task_display.c
    src/int_handlers.c
)

target_include_directories(${EXECUTABLE} PUBLIC
    inc/
)

target_link_libraries(${EXECUTABLE} PUBLIC
    cmsis
    ${MCU}_plat
    hal
    driver
    antos
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

add_custom_target(
    flash
    COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin 0x08000000
    COMMENT "Flashing the custom executable to the target device..."
    DEPENDS ${EXECUTABLE}
)

