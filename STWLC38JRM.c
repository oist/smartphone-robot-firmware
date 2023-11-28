#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "STWLC38JRM.h"

// Enables the external wireless charging module
void STWLC38JRM_init(uint enable_pin){
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);
    gpio_put(enable_pin, 1);
}

void STWLC38JRM_shutdown(uint enable_pin) {
    gpio_put(enable_pin, 0);
}
