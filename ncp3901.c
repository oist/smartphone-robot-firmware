#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ncp3901.h"
#include "hardware/adc.h"
#include "robot.h"
#include <inttypes.h>
#include "custom_printf.h"

static int32_t ncp3901_on_wireless_charger_attached(int32_t test);
static int32_t ncp3901_on_wireless_charger_detached(int32_t test);
static int32_t ncp3901_test_response();
static int8_t _gpio_wireless_charger;
static int8_t _gpio_otg;
static bool test_ncp3901_started = false;
static bool test_ncp3901_completed = false;
static bool wireless_charger_attached = false;

// on wireless power available
void ncp3901_on_wireless_charger_interrupt(uint gpio, uint32_t event_mask)
{
    if (event_mask & GPIO_IRQ_EDGE_RISE){
        gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_RISE);
        call_queue_try_add(&ncp3901_on_wireless_charger_attached, 0);
        if (test_ncp3901_started){
            call_queue_try_add(&ncp3901_test_response, 1);
        }
    } else if (event_mask & GPIO_IRQ_EDGE_FALL){
        gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_FALL);
        call_queue_try_add(&ncp3901_on_wireless_charger_detached, 0);
        if (test_ncp3901_started){
            call_queue_try_add(&ncp3901_test_response, 1);
        }
    }	
}

static int32_t ncp3901_on_wireless_charger_attached(int32_t test){
    rp2040_log("Wireless power available\n");
    // send to Android to inform that wireless power available.
    wireless_charger_attached = true;
    return 0;
}

static int32_t ncp3901_on_wireless_charger_detached(int32_t test){
    rp2040_log("Wireless power unavailable\n");
    // send to Android to inform that wireless power unavailable.
    wireless_charger_attached = false;
    return 0;
}

bool ncp3901_wireless_charger_attached(){
    return wireless_charger_attached;
}

// Power mux initialization
void ncp3901_init(uint gpio_wireless_charger, uint gpio_otg)
{
    _gpio_wireless_charger = gpio_wireless_charger;
    _gpio_otg = gpio_otg;

    // flag pin is normally low, but when wireless charger present will go high
    // that being said, with the voltage translation, it was reversed (i.e. when FLAG is
    // LOW VIN_B_EN/FLAG is HIGH and vice versa. 
    // so rising edge indicates charger was removed, while falling edge indicates charger attached
    gpio_init(_gpio_wireless_charger);
    gpio_put(_gpio_wireless_charger, 0);
    gpio_set_dir(_gpio_wireless_charger, GPIO_IN);
    gpio_pull_up(_gpio_wireless_charger);
    gpio_set_irq_enabled(_gpio_wireless_charger, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // OTG Disabled by default
    gpio_init(gpio_otg);
    gpio_put(gpio_otg, 0);
    gpio_set_dir(gpio_otg, GPIO_OUT);

    // init ADC0 as analog input (sampling should be done in main loop)
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
}

uint16_t ncp3901_adc0()
{
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    uint16_t result = adc_read();
    return result;
    // Save this value, add to a buffer, or merge with some moving avg.
    // rp2040_log("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
}

void ncp3901_shutdown(){
}

void test_ncp3901_interrupt(){
    rp2040_log("test_ncp3901_interrupt: starting test of wireless connection...\n");
    test_ncp3901_started = true;
    rp2040_log("test_ncp3901_interrupt: prior to driving low GPIO%d. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
    gpio_set_dir(_gpio_wireless_charger, GPIO_OUT);
    if (gpio_get(_gpio_wireless_charger) != 0){
	rp2040_log("test_ncp3901_interrupt: GPIO%d was not driven low. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
	assert(false);
    }
    rp2040_log("test_ncp3901_interrupt: after driving low GPIO%d. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
    uint32_t i = 0;
    while (!test_ncp3901_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    rp2040_log("test_ncp3901_interrupt wireless connect timed out\n");
	    assert(false);
	}
    }
    test_ncp3901_started = false;
    test_ncp3901_completed = false;
    rp2040_log("test_ncp3901_interrupt_wireless_connect: PASSED after %" PRIu32 " milliseconds.\n", i*10);

    // This should trigger the opposite interrupt EGDE_RISE
    rp2040_log("test_ncp3901_interrupt: starting test of wireless disconnection...\n");
    test_ncp3901_started = true;
    rp2040_log("test_ncp3901_interrupt: prior to pulling up GPIO%d. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
    gpio_set_dir(_gpio_wireless_charger, GPIO_IN);
    gpio_pull_up(_gpio_wireless_charger);
    if (gpio_get(_gpio_wireless_charger) != 1){
	rp2040_log("test_ncp3901_interrupt: GPIO%d was pulled up. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
	assert(false);
    }
    rp2040_log("test_ncp3901_interrupt: after pulling up GPIO%d. Current Value:%d\n", _gpio_wireless_charger, gpio_get(_gpio_wireless_charger));
    i = 0;
    while (!test_ncp3901_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    rp2040_log("test_ncp3901_interrupt wireless disconnect timed out\n");
	    assert(false);
	}
    }
    rp2040_log("test_ncp3901_interrupt_wireless_disconnect: PASSED after %" PRIu32 " milliseconds.\n", i*10);
}

static int32_t ncp3901_test_response(){
    test_ncp3901_completed = true;
    return 0;
}

