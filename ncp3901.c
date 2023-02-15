#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ncp3901.h"
#include "hardware/adc.h"

// on wireless power available
void on_wireless_enabled(uint gpio, ulong events)
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);

    // send to Android to inform that wireless power available.
}

// Power mux initialization
void ncp3901_init(uint gpio_wireless_available, uint gpio_otg)
{
    gpio_set_irq_enabled_with_callback(gpio_wireless_available,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, &on_wireless_enabled);

    // OTG Disabled by default
    gpio_init(gpio_otg);
    gpio_set_dir(gpio_otg, GPIO_OUT);
    gpio_put(gpio_otg, 0);

    // init ADC0 as analog input (sampling should be done in main loop)
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
}


void ncp3901_adc0()
{
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    uint16_t result = adc_read();
    // Save this value, add to a buffer, or merge with some moving avg.
    // printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
}
