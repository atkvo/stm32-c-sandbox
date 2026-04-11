add_library(si7021 STATIC
    src/si7021.c
)

target_include_directories(si7021 PUBLIC
    inc/
)

target_link_libraries(si7021 PUBLIC
    hal
)

