#ifndef NCP3901_
#define NCP3901_

void ncp3901_init(uint gpio_wireless_available, uint gpio_otg);

void on_wireless_enabled(uint gpio, ulong events);

void ncp3901_adc0();

#endif