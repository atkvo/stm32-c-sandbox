#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "i2c.h"
#include "slice.h"
#include "ssd1306.h"
#include "font.h"
#include "string.h"
#include "framebuffer.h"

// @todo: move this out of main.c
// maybe simple "platform_init()" to call these two?
// platform_clock_update()
#include "system_stm32f4xx.h"

enum {
    DISP_COL = 128,
    DISP_ROW = 64,
    DISP_PAGES = DISP_ROW / SSD1306_PAGE_HEIGHT,
    DISP_ADDR_TWO_TONE = 0x3c,
    DISP_ADDR_ONE_TONE = 0x3d,
};

typedef struct i2c_app_config_t {
    i2c_handle_t handle;
    gpio_pin_handle_t sda;
    gpio_pin_handle_t scl;
} i2c_app_config_t;

static void delay(const uint32_t count) {
    for (uint32_t i = 0; i < count; i++);
}

static void FATAL() {
    while (1) {}
};

static i2c_app_config_t configure_i2c() {
    i2c_app_config_t cfg;
    cfg.sda = gpio_pin_acquire(GPIO_PORT_B, 6);
    cfg.scl = gpio_pin_acquire(GPIO_PORT_B, 7);
    cfg.handle = i2c_acquire(0);

    i2c_init(cfg.handle, cfg.sda, cfg.scl);

    return cfg;
}

static void heartbeat(gpio_pin_handle_t led_pin, const uint32_t delay_count) {
    if (led_pin == NULL) {
        return;
    }

    gpio_pin_write(led_pin, 1);
    delay(delay_count);
    gpio_pin_write(led_pin, 0);
    delay(delay_count);
}

static void write_hello(slice_mutable_t s, size_t col, size_t row) {
    const uint8_t msg[] = "HELLO";
    fb_write_ascii(s, col, row, slice_view(msg, strlen(msg)));
}

static void write_number(slice_mutable_t s, size_t col, size_t row, size_t num) {
    const uint8_t ascii = '0' + num;
    fb_write_ascii(s, col, row, slice_view(&ascii, 1));
}

static void splash(ssd1306_handle_t oled, uint32_t cycle_count) {
    const uint8_t msg[] = "(^_^)";
    fb_clear(oled->ram);
    fb_write_ascii(oled->ram,
            oled->disp_info.columns / 2,
            DISP_PAGES / 2,
            slice_view(msg, strlen(msg)));

    ssd1306_update(oled);
    delay(cycle_count);
}

int main()
{
    SystemInit();
    SystemCoreClockUpdate();

    gpio_pin_handle_t led_pin = gpio_pin_acquire(GPIO_PORT_C, 13);
    if (led_pin == NULL) {
        FATAL();
    }

    gpio_pin_configure(
            led_pin,
            (gpio_pin_config_t) {
                .mode = GPIO_MODE_OUTPUT,
                .pupd = GPIO_PUPD_PULLUP,
                .output = {
                    GPIO_SPEED_LOW,
                    GPIO_OUTPUT_MODE_OPEN_DRAIN,
                },
            });

    i2c_app_config_t i2c_prop = configure_i2c();
    ssd1306_ctx_t oled_ctx;

    uint8_t display_ram[SSD1306_GET_RAM_SIZE(DISP_COL, DISP_ROW)];

    ssd1306_handle_t oled_handle = ssd1306_handle_create(
            &oled_ctx,
            i2c_prop.handle,
            DISP_ADDR_ONE_TONE,
            (ssd1306_display_info_t) {
                .columns = DISP_COL,
                .rows = DISP_ROW,
                },
            (slice_mutable_t){
                .ptr = display_ram,
                .len = sizeof(display_ram)
                }
            );

    if (oled_handle == NULL) {
        FATAL();
    }

    fb_clear(oled_handle->ram);

    enum {
        PULSE_FAST = 100000,
        PULSE_SLOW = 800000,

        PULSE_INIT = PULSE_FAST,
    };

    uint32_t pulse_duration = PULSE_INIT;

    bool screen_init_flag = false;
    const uint8_t heartbeat_pulses_before_init = 10;
    uint8_t count = 0;
    const bool ssd1306_disp_state = true;

    ssd1306_display_state_set(oled_handle, ssd1306_disp_state);
    ssd1306_scroll_state_set(oled_handle, false);
    while (1) {
        heartbeat(led_pin, pulse_duration);

        if (screen_init_flag == false) {
            count++;
        }

        if (count == heartbeat_pulses_before_init) {
            if (screen_init_flag) {
                pulse_duration =  pulse_duration == PULSE_FAST ? PULSE_SLOW : PULSE_FAST;
            }
            else {
                ssd1306_init(oled_handle);
                pulse_duration = PULSE_SLOW;
                screen_init_flag = true;

                splash(oled_handle, 800000 * 4);

                fb_clear(oled_handle->ram);
                uint8_t col = 0;
                for (uint8_t page = 0; page < DISP_PAGES; page++) {
                    write_number(oled_handle->ram, col, page, page);
                    col += FONT_PX_WIDTH + 1;
                }

                write_hello(oled_handle->ram, DISP_COL - (5 * (FONT_PX_WIDTH + FONT_PX_KERNING)), 0);

                ssd1306_update(oled_handle);
            }
        }
    }
}
