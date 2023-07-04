#ifndef NCP3901_
#define NCP3901_

#include "pico/types.h"

void ncp3901_init(uint gpio_wireless_available, uint gpio_otg);

void ncp3901_on_wireless_charger_interrupt(uint gpio, uint32_t event_mask);

void ncp3901_adc0();

void ncp3901_shutdown();

#endif
