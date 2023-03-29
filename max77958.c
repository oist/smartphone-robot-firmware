#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77958.h"
#include "max77958_driver.h"
#include "bit_ops.h"

void max77958_init(uint FPF1048BUCX_EN, uint TPS61253_EN){
    //// Disable Fpf1048bucx
    //gpio_init(FPF1048BUCX_EN);
    //gpio_set_dir(FPF1048BUCX_EN, GPIO_OUT);
    //gpio_put(FPF1048BUCX_EN, 0);

    //bool state = gpio_get(FPF1048BUCX_EN); // Not sure if this bool is necessary, or if better to use some interrupt
    //if (!state){
    //        // Disable Fpf1048bucx
    //        gpio_init(TPS61253_EN);
    //        gpio_set_dir(TPS61253_EN, GPIO_OUT);
    //        gpio_put(TPS61253_EN, 1);
    //}
    // Testing for just DEVICE_ID
    uint8_t buf[2];
    buf[0] = 0x00;
    buf[1] = 0x00;
    // Write the register 0x00 to set the pointer there before reading its value
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, buf, 1, false);
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, buf, 2, false);
    printf("DEVICE_ID = %x", buf[0]);
    printf("DEVICE_REV = %x", buf[1]);

    // Testing opCode by reading Customer Configuration Read (0x55 opCode)
    buf[0] = OPCODE_WRITE;
    buf[1] = OPCODE_SET_CSTM_INFORMATION_R;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, buf, 2, false);
    // End of OpCodecommand Requires a write to 0x41 to know the opcode command is finished
    buf[0] = OPCODE_WRITE_END;
    buf[1] = 0x00;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, buf, 2, false);
    // Normally would wait until interrupt occurs on GPIO but will just sleep for testing now
    sleep_ms(1000);
    buf[0] = OPCODE_READ;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, buf, 1, false);
    uint8_t return_buf[3];
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 3, false);
    printf("VID_LSB = %x", return_buf[1]);
    printf("VID_MSB = %x", return_buf[2]);
}
