// Simple LED blink on nRF52832 breakout board

#include "nrf_gpio.h"
#include "nrf_delay.h"

// LED definitions
#define LED_PIN      7
#define LED_BIT      (1 << LED_PIN)

int main(void)
{
    nrf_gpio_cfg_output(LED_PIN);

    while (1)
    {
        nrf_delay_ms(500);
        NRF_GPIO->OUT ^= LED_BIT;
    }
}

// EOF
