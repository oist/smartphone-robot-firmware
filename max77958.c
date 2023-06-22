#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77958.h"
#include "max77958_driver.h"
#include "bit_ops.h"
#include "robot.h"
#include <assert.h>

static uint8_t send_buf[33] = {0};
static uint8_t return_buf[33] = {0}; // Will read full buffer from registers 0x52 to 0x71
static int parse_interrupt_vals();
static void on_interrupt(uint gpio, long unsigned int events);
static void get_interrupt_vals();
static void get_interrupt_masks();
static void set_interrupt_masks();
static void opcode_read();
static int opcode_write(uint8_t *send_buf);
static queue_t* call_queue_ptr;
static queue_t* return_queue_ptr;
static queue_entry_t parse_interrupt_vals_entry = {&parse_interrupt_vals, 0};
static bool opcode_cmd_finished = false;
static bool power_swap_enabled = true;
static queue_t opcode_queue;
static void power_swap_request();
static void set_snk_pdos();
static void pd_msg_response();
static void customer_config_write();
static bool opcode_queue_pop();
bool opcodes_finished = false;
static void opcode_queue_add(void (*opcode_func)(), int32_t opcode_data);
static int32_t gpio_bool_to_int32(bool _GPIO4, bool _GPIO5);
static void gpio_set(int32_t gpio_val);
static void set_src_pdos();

static void on_interrupt(unsigned int gpio, long unsigned int events){
    if(!queue_try_add(call_queue_ptr, &parse_interrupt_vals_entry)){
	printf("call_queue is full");
        assert(false);
    }
}

static int on_pd_msg_received(){
    queue_entry_t on_pd_msg_received_entry = {&pd_msg_response, 0};
    if (!queue_try_add(call_queue_ptr, &on_pd_msg_received_entry)){
	printf("call_queue is full");
	assert(false);
    }
    return 0;
}

static int on_opcode_cmd_response(){
    // You can now READ back the OpCommand return registers
    opcode_read();
    // opcode_queue_pop will return false if the opcode queue is empty
    if (!opcode_queue_pop()){
        opcodes_finished = true;
    }
    return 0;
}

static void opcode_queue_add(void (*opcode_func)(), int32_t opcode_data){
    opcodes_finished = false;
    queue_entry_t opcode_entry = {opcode_func, opcode_data};
    if(!queue_try_add(&opcode_queue, &opcode_entry)){
	printf("opcode_queue is full");
	assert(false);
    }
} 

static int parse_interrupt_vals(){
    get_interrupt_vals();
    // don't really need these, but makes it easier to understand what each entry to the return_buf represents
    uint8_t* UIC_INT = &return_buf[0]; 
    uint8_t* CC_INT = &return_buf[1]; 
    uint8_t* PD_INT = &return_buf[2]; 
    // Check if the APCmdResI interrupt is on (AP command response pending)
    uint APCmdResI_mask = 1 << 7;
    uint PSRDYI_mask = 1 << 6;
    uint PDMsgI = 1 << 7;
    if (*UIC_INT & APCmdResI_mask){
	on_opcode_cmd_response();
    }else if (*PD_INT & PSRDYI_mask){
	printf("Power source ready\n");
	//on_power_source_ready();
    }else if (*PD_INT & PDMsgI){
        printf("Rec PD message\n");
	on_pd_msg_received();
    }
    return -1;
    
    // Check for other relevant interrupts here and do something with that info...
}

static void get_interrupt_vals(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT; // 0x04 Register
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
    printf("interrupts vals: 0x4: 0x%02x, 0x5: 0x%02x, 0x6: 0x%02x, 0x7: 0x%02x\n", return_buf[0], return_buf[1], return_buf[2], return_buf[3]);
}

static void get_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
}

static void set_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b01111111; // UIC_INT 0x4
    send_buf[2] = 0b11111111; // CC_INT 0x5 all masked by default
    send_buf[3] = 0b00111111; // PD_INT 0x6 unmasking PSRDYI
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 4, false);
}

static int opcode_write(uint8_t *buf){
    // buf should always be 32 bytes long since the register values from 0x22 to 0x41 are never overwritten, 
    // so you may send wrong data if you don't directly specify them for ALL registers. Note the defaults are NOT always 0x00, 
    // so you should send all values everytime. What a pain...
    if (buf[0] != 0x21){
	printf("buffer should always start with the 0x21 register");
	assert(false);
    }

    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, buf, sizeof(send_buf), false);
    printf("opcode_write: 0x%02x 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3]);

    // For whatever reason, this is necessary for the interrupt to fire. Even though I already write 0x00 to it in the line above.
    memset(send_buf, 0, sizeof &send_buf);
    send_buf[0] = 0x41;
    send_buf[1] = 0x00;
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 2, false);
    return 1;
}

static void opcode_read(){
    // Set the current register pointer to 0x51 to you can read the return values from the OpCode Command
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_READ_COMMAND;
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 33, false);
    printf("opcode_read: 0x%02x 0x%02x 0x%02x 0x%02x\n", return_buf[0], return_buf[1], return_buf[2], return_buf[3]);
    // Set breakpoint before clearing the registers via the following command. 
    // I'm commenting this out since I won't use the output in the code, but you can copy/paste this into gdb if you want to
    // inspect the results before clearing them

    // Clear registers 0x21 - 0x41 to ensure you don't write wrong values to opCode Commands later
    //memset(return_buf, 0, sizeof return_buf);
    //return_buf[0] = 0x21;
    //i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 32, false);
}

void max77958_init(uint gpio_interrupt, queue_t* cq, queue_t* rq){

    call_queue_ptr = cq;
    return_queue_ptr = rq;
    queue_init(&opcode_queue, sizeof(queue_entry_t), 16);

    // Testing for just DEVICE_ID
    // Write the register 0x00 to set the pointer there before reading its value
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 2, false);
    printf("DEVICE_ID = %x\n", return_buf[0]);
    printf("DEVICE_REV = %x\n", return_buf[1]);

    set_interrupt_masks();
    
    // max77958 sends active LOW on INTB connected to GPIO7 on the rp2040. Setup interrupt callback here
    gpio_init(gpio_interrupt);
    gpio_set_dir(gpio_interrupt, GPIO_IN);
    gpio_get(gpio_interrupt);
    gpio_pull_up(gpio_interrupt);
    gpio_set_irq_enabled_with_callback(gpio_interrupt, GPIO_IRQ_EDGE_FALL, true, &on_interrupt);
    // clear interupts
    get_interrupt_vals();

    // Add all opcode commands in order to a queue. These will be called sequentially from core1 via the call_queue
    opcode_queue_add(&customer_config_write, 0);
    //opcode_queue_add(&set_snk_pdos, 0);
    //opcode_queue_add(&set_src_pdos, 0);
    // Set GPIO5 and GPIO4 to LOW
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(false, false));
    // Set GPIO5 to HIGH and GPIO4 to LOW
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(true, false));

    opcode_queue_pop();
}

// check if opcode_queue has entries remaining
// if so remove an entry from the opcode_queue and run in on core1 via the call_queue
// return true if an entry was removed and added to the call_queue
// return false if opccode_queue was empty 
static bool opcode_queue_pop(){
    queue_entry_t entry;
    // if there is an entry in the opcode_queue, remove it and add it to the call_queue
    if (queue_try_remove(&opcode_queue, &entry)){
	// if the call_queue is full, assert
	if(queue_try_add(call_queue_ptr, &entry)){
	    return true;
	}else{
	    printf("opcode_queue_pop: call_queue full\n");
	    assert(false);
	}
    }
    // if there is no entry in the opcode_queue, return false
    else {
	printf("opcode_queue_pop: opcode_queue empty\n");
    	return false;
    }
}

static void customer_config_write(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x56; // Customer Configuration Write 
    send_buf[2] = 0b00101000; // All defaults values other than adding CC Try SNK Mode 
    send_buf[3] = 0x0B; // default VID
    send_buf[4] = 0x6A; // default VID
    send_buf[5] = 0x68; // default PID
    send_buf[6] = 0x60; // default PID
    send_buf[7] = 0x00; // RSVD
    send_buf[8] = 0x64; // default SRC_PDO_V
    send_buf[9] = 0x00; // default SRC_PDO_V of 5.0V (0x64= 100, and 50mA*100). 
    send_buf[10] = 0x14; // SRC_PDO_MaxI
    send_buf[11] = 0x00; // SRC_PDO_MaxI = 1.0A (0x64=100, and 100*10mA)
    opcode_write(send_buf);
}

// A function to make turning on/off GPIO4 and 5 more readable
static int32_t gpio_bool_to_int32(bool _GPIO4, bool _GPIO5){
    //Reg 0x23 GPIO7Output,GPIO7Direction,GPIO6Output,GPIO6Direction,GPIO5Output,GPIO5Direction,GPIO4Output,GPIODirection
    return (_GPIO5 << 3) | (1 << 2) | (_GPIO4 << 1) | (1 << 0);
}

// A function to set the GPIO of the max77958 taking as input two bool values setting GPIO4 and GPIO5
static void gpio_set(int32_t gpio_val){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_GPIO; 
    send_buf[2] = 0x00; //Reg 0x22 by default should be all 0s
    send_buf[3] = gpio_val;
    opcode_write(send_buf);
}

static void power_swap_request(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x37; // Send Swap Request 
    send_buf[2] = 0x02; // PR SWAP
    opcode_write(send_buf);
}

static void set_src_pdos(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_SOURCE_CAP;
    send_buf[2] = 0b00000001; // specify only 1 PDO
    // You can verify this by comparing against Table 6-9 of the USB-PD spec 
    // This represents:
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 1 Dual-Role Power ON
    // 0 Suspend Supported OFF
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 1 Dual-Role Data ON
    // 0 Unchunked Messages OFF
    // 0 ERP Mode Incable
    // 0 RSVD
    // 00  Peak current
    // 0001100100 = 100x 50mV = 5.0V
    // 0000101100 = 100x 10mA = 1.0A
    // In summary 00100110000000011001000000101100 or 0x2601902C
    send_buf[3] = 0x2C;
    send_buf[4] = 0x90;
    send_buf[5] = 0x01;
    send_buf[6] = 0x26;
    opcode_write(send_buf);
}
static void set_snk_pdos(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SNK_PDO_SET; 
    send_buf[2] = 0b00000001; // Write to RAM only and specify only 1 PDO 
    // Next four are specified by Analog Support via 0x1401912C LSB first
    // You can verify this by comparing against Table 6-16 of the USB-PD spec
    // I convert this to binary 00010100000000011001000100101100
    // This represents:
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 0 Dual-Role Power Off
    // 1 Higher Capability ON
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 0 Dual-Role Data OFF
    // 00 Fast Role Swap Not Supported
    // 000 RSVD
    // 0001100100 = 100x 50mV = 5.0V
    // 0100101100 = 300x 10mA = 3.0A
    // I then want to update this to 
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 1 Dual-Role Power ON
    // 0 Higher Capability OFF
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 1 Dual-Role Data ON
    // 01 Fast Role Swap Default USB Power (can update to 10b for 1.5A@5V but not sure if this is supported by chips)
    // 000 RSVD
    // 0001100100 = 100x 50mV = 5.0V
    // 0100101100 = 300x 10mA = 3.0A
    // In summary 00100110100000011001000100101100 or 0x2681912C
    send_buf[3] = 0x2C;
    send_buf[4] = 0x91;
    send_buf[5] = 0x81;
    send_buf[6] = 0x26;
    // default values 
    //send_buf[3] = 0x2C;
    //send_buf[4] = 0x91;
    //send_buf[5] = 0x01;
    //send_buf[6] = 0x14;    
    //Trying with MSB first (didn't work)
    //send_buf[3] = 0x14;    
    //send_buf[4] = 0x01;
    //send_buf[5] = 0x91;
    //send_buf[6] = 0x2C;
    opcode_write(send_buf);
}

static void pd_msg_response(){
    // Read the 0xE PD_STATUS0 register as it contains the PD message Type recieved 
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_PD_STATUS0; // 0xE PD_STATUS0 Register 
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    printf("PD_STATUS0: 0x%02x\n", return_buf[0]);
    if (return_buf[0] == PDMSG_PRSWAP_SRCTOSWAP){
	    //TODO implement later
    }else if (return_buf[0] == PDMSG_PRSWAP_SWAPTOSNK){
	    // turn off VBus
	    opcode_queue_add(&gpio_set, gpio_bool_to_int32(false, true));
            opcode_queue_pop();
    }else if (return_buf[0] == PDMSG_PRSWAP_SNKTOSWAP){
	    //TODO implement later
    }else if (return_buf[0] == PDMSG_PRSWAP_SWAPTOSRC){
	    // turn on Vbus
	    opcode_queue_add(&gpio_set, gpio_bool_to_int32(true, true));
	    opcode_queue_pop();
    }
}

void max77958_shutdown(uint gpio_interrupt){
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(false, false));
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished){
	sleep_ms(100);
	i++;
	if (i > 10){
	    printf("Error: Timed out waiting for GPIO to finish\n");
	    assert(false);
	}
    }
}
