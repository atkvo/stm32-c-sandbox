#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include "antos.h"
#include "gpio.h"
#include "i2c.h"
#include "slice.h"
#include "ssd1306.h"
#include "string.h"
#include "framebuffer.h"
#include "task_display.h"

// @todo: move this out of main.c
// maybe simple "platform_init()" to call these two?
// platform_clock_update()
#include "system_stm32f4xx.h"

enum {
    DISP_COL = FB_COLUMNS,
    DISP_ROW = 64,
    DISP_PAGES = FB_PAGES,
    DISP_ADDR_TWO_TONE = 0x3c,
    DISP_ADDR_ONE_TONE = 0x3d,
};

static uint8_t task_mem[ANT_REQUIRED_MEM(4)];

static_assert(DISP_ROW == (FB_PAGES * SSD1306_PAGE_HEIGHT), "Framebuffer page/col not expected");

typedef struct i2c_app_config_t {
    i2c_handle_t handle;
    gpio_pin_handle_t sda;
    gpio_pin_handle_t scl;
} i2c_app_config_t;

void delay(const uint32_t count);
static void FATAL();

static i2c_app_config_t configure_i2c() {
    i2c_app_config_t cfg;
    cfg.sda = gpio_pin_acquire(GPIO_PORT_B, 6);
    cfg.scl = gpio_pin_acquire(GPIO_PORT_B, 7);
    cfg.handle = i2c_acquire(0);

    i2c_init(cfg.handle, cfg.sda, cfg.scl);

    return cfg;
}

static void heartbeat(gpio_pin_handle_t led_pin, const uint32_t cycles, const uint32_t delay_count) {
    if (led_pin == NULL) {
        return;
    }

    for (size_t i = 0; i < cycles; i++) {
        gpio_pin_write(led_pin, 1);
        delay(delay_count);
        gpio_pin_write(led_pin, 0);
        delay(delay_count);
    }
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

    uint8_t display_ram[SSD1306_GET_RAM_SIZE(DISP_COL, DISP_ROW)];
    ssd1306_ctx_t oled_ctx;

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

    heartbeat(led_pin, 5, pulse_duration);

    task_display_ctx_t ctx_display = task_display_create_ctx(oled_handle);

    if (ant_init(slice_mut_view(task_mem, sizeof(task_mem)), 3) != ANT_STATUS_OK) {
        heartbeat(led_pin, 20, PULSE_FAST);
        FATAL();
    }

    ant_register((ant_task_t)&task_display, &ctx_display);

    ant_run();

    heartbeat(led_pin, 20, PULSE_FAST);
    FATAL();
}

void delay(const uint32_t count) {
    for (uint32_t i = 0; i < count; i++);
}

static void FATAL() {
    while (1) {}
};

