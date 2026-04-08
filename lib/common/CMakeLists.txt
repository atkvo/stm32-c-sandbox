add_library(common STATIC
    src/pool_mgr.c
)

target_include_directories(common PUBLIC
    inc/
)

