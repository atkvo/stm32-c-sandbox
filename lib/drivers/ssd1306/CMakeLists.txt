add_library(ssd1306 STATIC
    src/framebuffer.c
    src/font.c
    src/ssd1306.c
)

target_include_directories(ssd1306 PUBLIC
    inc/
)

target_link_libraries(ssd1306 PUBLIC
    hal
)

