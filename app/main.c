#include <stdint.h>
#include <stddef.h>
#include "system_stm32f4xx.h"
#include "gpio.h"

static void sleep_ms(const uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++);
}

int main()
{
    SystemInit();
    SystemCoreClockUpdate();

    gpio_pin_handle_t led_pin = gpio_pin_acquire(GPIO_PORT_C, 13);
    if (led_pin == NULL) {
        while (1) {}
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

    while (1)
    {
        gpio_pin_write(led_pin, 1);
        sleep_ms(200000);
        gpio_pin_write(led_pin, 0);
        sleep_ms(200000);
    }
}
