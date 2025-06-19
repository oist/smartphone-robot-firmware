#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "wrm483265_10f5_12v_g.h"

// Enables the external wireless charging module
void wrm483265_10f5_12v_g_init(uint enable_pin){
    gpio_init(enable_pin);
    gpio_set_dir(enable_pin, GPIO_OUT);
    gpio_put(enable_pin, 1);
}

void wrm483265_10f5_12v_g_shutdown(uint enable_pin) {
    gpio_put(enable_pin, 0);
}
