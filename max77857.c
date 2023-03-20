#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77857.h"
#include "bit_ops.h"

void max77857_init() {
    uint8_t buf[2];

    // CONT1
    buf[0] = MAX77857_REG_REG_CONT1_ADDR;
    // Change switching current limit threshold ILIM to 2.8A
    buf[1] = bit_set_range(MAX77857_REG_REG_CONT1_RESET,
                  MAX77857_REG_REG_CONT1_ILM_LSB,
                  MAX77857_REG_REG_CONT1_ILM_MSB,
                  MAX77857_REG_REG_CONT1_ILM_DEFAULT);
    // Change internal compensation (COMP)
    buf[1] = bit_set_range(buf[1],
                  MAX77857_REG_REG_CONT1_COMP_LSB,
                  MAX77857_REG_REG_CONT1_COMP_MSB,
                  MAX77857_REG_REG_CONT1_COMP_DEFAULT);
    // Change switching frequency (FREQ)
    buf[1] = bit_set_range(buf[1],
                  MAX77857_REG_REG_CONT1_FREQ_LSB,
                  MAX77857_REG_REG_CONT1_FREQ_MSB,
                  MAX77857_REG_REG_CONT1_FREQ_DEFAULT);
    i2c_write_blocking(i2c1, MAX77857_ADDR, buf, 2, false);

    //// CONT2 and VREF
    //buf[0] = MAX77857_REG_REG_CONT2_ADDR;
    //// Change Vref, to indirectly change Vout. This is just overwriting the reset value, so uncomment and change if necessary 
    //buf[1] = bit_set_range(MAX77857_REG_REG_CONT2_RESET,
    //              MAX77857_REG_REG_CONT2_VREF_LSB,
    //              MAX77857_REG_REG_CONT2_VREF_MSB,
    //              MAX77857_REG_REG_CONT2_VREF_DEFAULT);
    
    //// CONT3 and FPWM
    //buf[0] = MAX77857_REG_REG_CONT3_ADDR;
    //// Change FRWM. This is just overwriting the reset value, so uncomment and change if necessary 
    //buf[1] = bit_set_range(MAX77857_REG_REG_CONT3_RESET,
    //              MAX77857_REG_REG_CONT3_FPWM_LSB,
    //              MAX77857_REG_REG_CONT3_FPWM_MSB,
    //              MAX77857_REG_REG_CONT3_FPWM_DEFAULT);}
}
