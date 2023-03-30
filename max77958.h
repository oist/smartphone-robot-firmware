#ifndef MAX77958_
#define MAX77958_

#include "pico/types.h"

#define FPF1048BUCX_EN _u(4) // GPIO4
#define TPS61253_EN _u(5) // GPIO5

// gpio_interrupt being the gpio pin on the mcu attached to the INTB pin of max77958
void max77958_init(uint gpio_interrupt);

#endif
