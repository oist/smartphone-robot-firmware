#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bq27742_g1.h"
#include "bit_ops.h"
#include <string.h>

static uint8_t send_buf[2];
static uint8_t return_buf[2];
static uint32_t voltage = 0;
static uint32_t safety_status = 0;

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
    // Test reading the safety_status
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x08;
    send_buf[1] = 0x09;
    send_buf[0] = 0x1A;
    send_buf[1] = 0x1B;
    while (1){
        memset(return_buf, 0, sizeof return_buf);
	memset(&voltage, 0, sizeof(uint32_t));
	memset(&safety_status, 0, sizeof(uint32_t));
	i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
        i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

	voltage = (return_buf[1] << 8) | return_buf[0];
	// this might need to be return_buf[1]...not sure what Low and High byte refer to in the datasheet context
	uint8_t low_byte = return_buf[0];

	if (low_byte & ISD_MASK){
	    printf("Internal Short condition detected\n");
	}else if (low_byte & TDD_MASK){
	    printf("Tab Disconnect condition detected\n");
	}else if (low_byte & OTC_MASK){
	    printf("Overtemperature in charge condition detected\n");
	}else if (low_byte & OVP_MASK){
	    printf("Overvoltage condition detected\n");
	}else if (low_byte & UVP_MASK){
	    printf("Undervoltage condition detected\n");
	}else{
	    printf("No error detected in battery protection\n");
	}

        printf("Voltage: %d\n", voltage);
        sleep_ms(2000); 
    }
}
