#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "i2c.h"
#include "ssd1306.h"

// @todo: move this out of main.c
// maybe simple "platform_init()" to call these two?
// platform_clock_update()
#include "system_stm32f4xx.h"

static void sleep(const uint32_t count) {
    for (uint32_t i = 0; i < count; i++);
}

typedef struct i2c_app_config_t {
    i2c_handle_t handle;
    gpio_pin_handle_t sda;
    gpio_pin_handle_t scl;
} i2c_app_config_t;

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
    sleep(delay_count);
    gpio_pin_write(led_pin, 0);
    sleep(delay_count);
}

static void FATAL() {
    while (1) {}
};

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

    enum {
        PULSE_FAST = 100000,
        PULSE_SLOW = 800000,

        PULSE_INIT = PULSE_FAST,
    };

    uint32_t pulse_rate = PULSE_INIT;

    bool screen_init = false;
    const uint8_t heartbeat_pulses_before_init = 5;
    uint8_t count = 0;

    while (1) {
        heartbeat(led_pin, pulse_rate);

        if (screen_init == false) {
            count++;
        }

        if ((count == heartbeat_pulses_before_init) && (screen_init == false)) {
            ssd1306_init(i2c_prop.handle, 0x3d);
            pulse_rate = PULSE_SLOW;
            screen_init = true;
        }
    }
}
