add_library(antos STATIC
    src/antos.c
)

target_include_directories(antos PUBLIC
    inc/
)

target_link_libraries(antos PRIVATE common)

