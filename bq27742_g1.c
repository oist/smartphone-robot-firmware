#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bq27742_g1.h"
#include "bit_ops.h"
#include <string.h>

static uint8_t send_buf[2];
static uint8_t return_buf[2];
static uint32_t voltage = 0;

void bq27742_g1_init() {
    // uint8_t buf[2];

    // ToDo Implement Key Daya Flash Parameters somehow.
    // buf[0] = BQ227742_G1_REG_REG_CONT1_ADDR;
    // // Change switching current limit threshold ILIM to 2.8A
    // buf[1] = bit_set_range(BQ227742_G1_REG_REG_CONT1_RESET,
    //               BQ227742_G1_REG_REG_CONT1_ILM_LSB,
    //               BQ227742_G1_REG_REG_CONT1_ILM_MSB,
    //               BQ227742_G1_REG_REG_CONT1_ILM_DEFAULT);
    // Change internal compensation (COMP)

    // Test reading the voltage
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x08;
    send_buf[1] = 0x09;
    while (1){
        memset(return_buf, 0, sizeof return_buf);
	memset(&voltage, 0, sizeof(uint32_t));
	i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
        i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

	voltage = (return_buf[1] << 8) | return_buf[0];

        printf("Voltage: %d\n", voltage);
        sleep_ms(2000); 
    }
}
