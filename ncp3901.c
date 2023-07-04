#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ncp3901.h"
#include "hardware/adc.h"

static int8_t _gpio_wireless_charger;
static int8_t _gpio_otg;

// on wireless power available
void ncp3901_on_wireless_charger_interrupt(uint gpio, uint32_t event_mask)
{
    if (event_mask & GPIO_IRQ_EDGE_RISE){
	gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_RISE);
	//printf("Wireless power available\n");
	// send to Android to inform that wireless power available.
	// remember this should only add to the call_queue, not actually call the function.
    } else if (gpio_get_irq_event_mask(gpio) & GPIO_IRQ_EDGE_FALL){
	gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_FALL);
	//printf("Wireless power unavailable\n");
	// send to Android to inform that wireless power unavailable.
	// remember this should only add to the call_queue, not actually call the function.
    }	
}

// Power mux initialization
void ncp3901_init(uint gpio_wireless_charger, uint gpio_otg)
{
    _gpio_wireless_charger = gpio_wireless_charger;
    _gpio_otg = gpio_otg;

    // flag pin is normally low, but when wireless charger present will go high
    // so rising edge indicates charger present, while falling edge indicates charger removed
    gpio_set_irq_enabled(_gpio_wireless_charger, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

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

void ncp3901_shutdown(){
}
