#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sn74ahc125rgyr.h"
#include "bit_ops.h"

void sn74ahc125rgyr_init(uint GPIO){
    // Hold EN pin HIGH during startup
    gpio_init(GPIO);
    gpio_set_dir(GPIO, GPIO_OUT);
    gpio_put(GPIO, 1);
}

void sn74ahc125rgyr_on_end_of_start(uint GPIO){
    gpio_put(GPIO, 0);
}

void sn74ahc125rgyr_shutdown(uint GPIO){
    gpio_deinit(GPIO);
}
