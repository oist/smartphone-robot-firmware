#ifndef NCP3901_
#define NCP3901_

#include "pico/types.h"

void ncp3901_init(uint gpio_wireless_available, uint gpio_otg);
void ncp3901_on_wireless_charger_interrupt(uint gpio, uint32_t event_mask);
uint16_t ncp3901_adc0();
void ncp3901_shutdown();
void test_ncp3901_interrupt();

#endif
