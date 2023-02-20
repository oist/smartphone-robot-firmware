#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77958.h"
#include "bit_ops.h"

void max7795_init(uint FPF1048BUCX_EN, uint TPS61253_EN){
    // Disable Fpf1048bucx
    gpio_init(FPF1048BUCX_EN);
    gpio_set_dir(FPF1048BUCX_EN, GPIO_OUT);
    gpio_put(FPF1048BUCX_EN, 0);

    bool state = gpio_get(FPF1048BUCX_EN); // Not sure if this bool is necessary, or if better to use some interrupt
    if (!state){
	    // Disable Fpf1048bucx
	    gpio_init(TPS61253_EN);
	    gpio_set_dir(TPS61253_EN, GPIO_OUT);
	    gpio_put(TPS61253_EN, 1);
    }
}
