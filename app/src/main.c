#include <assert.h>
#include <stdint.h>
#include <stddef.h>

#include "antos.h"
#include "app.h"
#include "framebuffer.h"
#include "gpio.h"
#include "i2c.h"
#include "platform.h"
#include "slice.h"
#include "ssd1306.h"
#include "string.h"
#include "task_button.h"
#include "task_display.h"
#include "task_heartbeat.h"
#include "timer.h"

enum {
    DISP_COL = FB_COLUMNS,
    DISP_ROW = 64,
    DISP_PAGES = FB_PAGES,
    DISP_ADDR_TWO_TONE = 0x3c,
    DISP_ADDR_ONE_TONE = 0x3d,
};

static_assert(DISP_ROW == (FB_PAGES * SSD1306_PAGE_HEIGHT), "Framebuffer page/col not expected");

enum {
    ANT_MAX_TASKS = 5,
};

static uint8_t task_mem[ANT_REQUIRED_MEM(ANT_MAX_TASKS)];

typedef struct i2c_app_config_t {
    i2c_handle_t handle;
    gpio_pin_handle_t sda;
    gpio_pin_handle_t scl;
} i2c_app_config_t;

void delay(const uint32_t count);
static void FATAL();

static i2c_app_config_t configure_i2c() {
    i2c_app_config_t cfg;
    cfg.sda = gpio_pin_take(GPIO_PORT_B, 6);
    cfg.scl = gpio_pin_take(GPIO_PORT_B, 7);
    cfg.handle = i2c_take(0);

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
    plat_init();
    plat_system_core_clock_update();

    gpio_pin_handle_t led_pin = gpio_pin_take(GPIO_PORT_C, 13);
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


    gpio_pin_handle_t button_pin = gpio_pin_take(GPIO_PORT_A, 0);
    if (button_pin == NULL) {
        FATAL();
    }

    gpio_pin_configure(
            button_pin,
            (gpio_pin_config_t) {
                .mode = GPIO_MODE_INPUT,
                .pupd = GPIO_PUPD_PULLUP,
            });

    heartbeat_task_ctx_t hb_ctx = task_heartbeat_create_ctx(led_pin, 500);
    button_task_ctx_t button_ctx = task_button_create_ctx(button_pin);
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
    ctx_display.press_count = &button_ctx.press_count;

    app_os_timer = timer_take(APP_OS_TIMER_NUM);
    if (app_os_timer == NULL) {
        heartbeat(led_pin, 10, PULSE_FAST);
        FATAL();
    }

    timer_cfg_t timer_cfg = {
        .hz = APP_OS_TICK_RATE_HZ,
        .periodic = true,
    };

    timer_init(app_os_timer, timer_cfg);
    timer_int_enable(app_os_timer);
    timer_start(app_os_timer);

    if (ant_init(
            slice_mut_view(task_mem, sizeof(task_mem)),
            ANT_MAX_TASKS,
            APP_OS_TICK_RATE_HZ) != ANT_STATUS_OK) {
        heartbeat(led_pin, 20, PULSE_FAST);
        FATAL();
    }

    ant_register_task((ant_task_t)&task_display, &ctx_display);
    ant_register_task((ant_task_t)&task_heartbeat, &hb_ctx);
    ant_register_task((ant_task_t)&task_read_button, &button_ctx);

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

