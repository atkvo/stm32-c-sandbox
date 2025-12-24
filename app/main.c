#include <stddef.h>
#include "system_stm32f4xx.h"
#include "gpio.h"

int main()
{
    SystemInit();
    SystemCoreClockUpdate();

    while (1)
    {
        gpio_port_handle_t port_c = gpio_handle_acquire(GPIO_PORT_C);
        if (port_c == NULL) {
            break;
        }

        gpio_init(port_c);
    }
}
