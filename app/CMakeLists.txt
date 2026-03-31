
add_executable(${EXECUTABLE}
    main.c
    src/task_display.c
)

target_include_directories(${EXECUTABLE} PUBLIC
    inc/
)

target_link_libraries(${EXECUTABLE} PUBLIC
    cmsis
    ${MCU}_plat
    hal
    driver
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
