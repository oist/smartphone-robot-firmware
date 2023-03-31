#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77958.h"
#include "max77958_driver.h"
#include "bit_ops.h"

static void on_interrupt(uint gpio, long unsigned int events);
static uint8_t send_buf[33] = {0};
static uint8_t return_buf[33] = {0}; // Will read full buffer from registers 0x52 to 0x71
static void get_interrupt_vals();
static void get_interrupt_masks();
static void set_interrupt_masks();
static int opcode_read();
static int opcode_write(uint8_t *send_buf, uint8_t *return_buf);

static void on_interrupt(unsigned int gpio, long unsigned int events){
    // You can now READ back the OpCommand return registers
    opcode_read();
}

static void get_interrupt_vals(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT; // 0x04 Register
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
}

static void get_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
}

static void set_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b10011111;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 2, false);
}

static int opcode_write(uint8_t *send_buf, uint8_t *return_buf){
    // send_buf should always be 32 bytes long since the register values from 0x22 to 0x41 are never overwritten, 
    // so you may send wrong data if you don't directly specify them for ALL registers. Note the defaults are NOT always 0x00, 
    // so you should send all values everytime. What a pain...
    if (send_buf[0] != 0x21){
	// buffer should always start with the 0x21 register
	return -1;
    }
    if (send_buf[32] != 0x00){
	// OPCODE_WRITE_END (0x41) should be set to 0x00 to signify the end of the OpCode write
	return -2;
    }
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 33, false);

    // For whatever reason, this is necessary for the interrupt to fire. Even though I already write 0x00 to it in the line above.
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x41;
    send_buf[1] = 0x00;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 2, false);
    return 1;
}

static int opcode_read(){
    // Set the current register pointer to 0x51 to you can read the return values from the OpCode Command
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_READ_COMMAND;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 33, false);
    // Set breakpoint before clearing the registers via the following command. 
    // I'm commenting this out since I won't use the output in the code, but you can copy/paste this into gdb if you want to
    // inspect the results before clearing them

    // Clear registers 0x21 - 0x41 to ensure you don't write wrong values to opCode Commands later
    memset(return_buf, 0, sizeof return_buf);
    return_buf[0] = 0x21;
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 32, false);
}

void max77958_init(uint gpio_interrupt){

    // Testing for just DEVICE_ID
    // Write the register 0x00 to set the pointer there before reading its value
    i2c_write_blocking(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_blocking(i2c0, MAX77958_SLAVE_P1, return_buf, 2, false);
    printf("DEVICE_ID = %x", return_buf[0]);
    printf("DEVICE_REV = %x", return_buf[1]);
    // clear the buffer
    memset(return_buf, 0, sizeof return_buf);

    get_interrupt_masks();
    get_interrupt_vals();
    set_interrupt_masks();
    get_interrupt_masks();
    
    // max77958 sends active LOW on INTB connected to GPIO7 on the rp2040. Setup interrupt callback here
    gpio_init(gpio_interrupt);
    gpio_set_dir(gpio_interrupt, GPIO_IN);
    gpio_get(gpio_interrupt);
    gpio_pull_up(gpio_interrupt);
    gpio_set_irq_enabled_with_callback(gpio_interrupt, GPIO_IRQ_EDGE_FALL, true, &on_interrupt);

    // Disable TPS61253_EN and Fpf1048bucx by turning off GPIO4 and GPIO5 on the MAX77958.
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_GPIO; 
    send_buf[2] = 0x00; //Reg 0x22 by default should be all 0s
    //Reg 0x23 GPIO7Output,GPIO7Direction,GPIO6Output,GPIO6Direction,GPIO5Output,GPIO5Direction,GPIO4Output,GPIODirection
    //To disable TPS61253_EN, We need to set GPIO5Output Low (bit b3 of 0x23 to 0) and set it to output (b2 of 0x23 to 1)
    //To disable Fpf1048bucx We also need GPIO4 to be low (b1=0) and GPIO4Direction to be output (b0=1)
    send_buf[3] = 0b00000101; 
    opcode_write(send_buf, return_buf);
    sleep_ms(1000);
    opcode_read();

    // Enable TPS61253_EN via GPIO5 on the MAX77958.
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_GPIO; 
    send_buf[2] = 0x00; //Reg 0x22 by default should be all 0s
    //Reg 0x23 GPIO7Output,GPIO7Direction,GPIO6Output,GPIO6Direction,GPIO5Output,GPIO5Direction,GPIO4Output,GPIODirection
    //To disable TPS61253_EN, We need to set GPIO5Output Low (bit b3 of 0x23 to 0) and set it to output (b2 of 0x23 to 1)
    //To disable Fpf1048bucx We also need GPIO4 to be low (b1=0) and GPIO4Direction to be output (b0=1)
    send_buf[3] = 0b00001101; 
    opcode_write(send_buf, return_buf);

}

