
add_executable(sim_test
    src/main.c
)

target_include_directories(sim_test PUBLIC
    inc/
)

target_link_libraries(sim_test PUBLIC
    antos
    common
    pthread
)

