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
#include <inttypes.h>

static uint8_t send_buf[33] = {0};
static uint8_t return_buf[33] = {0}; // Will read full buffer from registers 0x52 to 0x71
static int parse_interrupt_vals();
static void on_interrupt();
static void get_interrupt_vals();
static void get_interrupt_masks();
static void set_interrupt_masks();
static void opcode_read();
static int opcode_write(uint8_t *send_buf);
static void max77958_test_response();
static queue_t* call_queue_ptr;
static queue_t* return_queue_ptr;
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
static void on_ccstat_change();
static void on_chgtype_change();
static void on_ccvcnstat_change();
static void on_ccistat_change();
static void on_ccpinstat_change();
static void vbus_turn_off();
static void vbus_turn_on();
static void customer_config_read();
static uint8_t _gpio_interrupt;
static uint8_t interrupt_mask = GPIO_IRQ_EDGE_FALL;
static bool test_max77958_interrupt_bool = false;
static bool test_max77958_started = false;
static bool test_max77958_completed = false;

void max77958_on_interrupt(uint gpio, uint32_t event_mask){
    if (event_mask & interrupt_mask){
        gpio_acknowledge_irq(_gpio_interrupt, interrupt_mask);	
	call_queue_try_add(&parse_interrupt_vals, 0);
        if (test_max77958_started){
            call_queue_try_add(&max77958_test_response, 1);
        }
    }
}

static int on_pd_msg_received(){
    printf("Rec PD message\n");
    queue_entry_t on_pd_msg_received_entry = {&pd_msg_response, 0};
    call_queue_try_add(&on_pd_msg_received_entry, 0);
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

static void on_ccstat_change(){
    printf("ccstat changed\n");
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0xC; // CC_STATUS0 
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    uint8_t CCStat = return_buf[0] & 0b111;
    switch (CCStat){
	case 0b000:
	    printf("ccstat changed to no connection\n");
	    vbus_turn_off();
	    break;
	case 0b001:
	    printf("ccstat changed to SINK\n");
	    vbus_turn_off();
	    break;
	case 0b010:
	    printf("ccstat changed to SOURCE\n");
	    //vbus_turn_on();
	    break;
	default: 
	    printf("ccstat changed to %d\n", CCStat);
	    break;
	}
}

static void on_chgtype_change(){
    printf("chgtype changed\n");
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0xA; // BC_STATUS
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    uint8_t ChgType = return_buf[0] & 0b11;
    switch (ChgType){
	case 0b000:
	    printf("ChgTyp changed to nothing attached\n");
	    break;
	case 0b001:
	    printf("ChgTyp changed to SDP, USB cable attached\n");
	    break;
	case 0b010:
	    printf("ChgTyp changed to CDP, Charging Downstream Port\n");
	    break;
	default: 
	    printf("ChgTyp changed to DCP, Dedicated charger\n");
	    break;
	}
}

static void opcode_queue_add(void (*opcode_func)(), int32_t opcode_data){
    opcodes_finished = false;
    queue_entry_t opcode_entry = {opcode_func, opcode_data};
    if(!queue_try_add(&opcode_queue, &opcode_entry)){
	printf("opcode_queue is full");
	assert(false);
    }
    printf("Added to opcode_queue. %d entries remaining\n", queue_get_level(&opcode_queue));
} 

static int parse_interrupt_vals(){
    uint16_t return_val = 0;
    get_interrupt_vals();
    // don't really need these, but makes it easier to understand what each entry to the return_buf represents
    uint8_t* UIC_INT = &return_buf[0]; 
    uint8_t* CC_INT = &return_buf[1]; 
    uint8_t* PD_INT = &return_buf[2]; 
    // Check if the APCmdResI interrupt is on (AP command response pending)
    uint APCmdResI_mask = 1 << 7;
    uint PSRDYI_mask = 1 << 6;
    uint PDMsgI = 1 << 7;
    uint CCStat = 1 << 0;
    uint ChgType = 1 << 1;
    uint CCVcnStatI = 1 << 1;
    uint CCIStatI = 1 << 2;
    uint CCPinStatI = 1 << 3;
    if (*UIC_INT & APCmdResI_mask){
	on_opcode_cmd_response();
	return_val |= 1 << 0;
    }
    if (*PD_INT & PSRDYI_mask){
	printf("Power source ready\n");
	//on_power_source_ready();
	return_val |= 1 << 1;
    }
    if (*PD_INT & PDMsgI){
	on_pd_msg_received();
	return_val |= 1 << 2;
    }
    if (*UIC_INT & ChgType){
	on_chgtype_change();
	return_val |= 1 << 3;
    }
    if (*CC_INT & CCStat){
	on_ccstat_change();
	return_val |= 1 << 4;
    }
    if (*CC_INT & CCVcnStatI){
	on_ccvcnstat_change();
	return_val |= 1 << 5;
    }
    if (*CC_INT & CCIStatI){
	on_ccistat_change();
	return_val |= 1 << 6;
    }
    if (*CC_INT & CCPinStatI){
	on_ccpinstat_change();
	return_val |= 1 << 7;
    }
    if (return_val == 0){
	test_max77958_interrupt_bool = true;
    }
    return return_val;
    
    // Check for other relevant interrupts here and do something with that info...
}

static void on_ccvcnstat_change(){
    printf("CCStat changed\n");
}

static void on_ccistat_change(){
    printf("CCIStat changed\n");
}

static void on_ccpinstat_change(){
    printf("CCPinStat changed\n");
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

// Mask all interrupts for unit test purposes
static void set_interrupt_masks_all_masked(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b11111111; // UIC_INT_M 0x10 values
    send_buf[2] = 0b11111111; // CC_INT_M 0x11 values
    send_buf[3] = 0b11111111; // PD_INT_M 0x12 unmasking PSRDYI
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 4, false);
}

static void set_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b01111101; // UIC_INT_M 0x10 values
    send_buf[2] = 0b11110000; // CC_INT_M 0x11 values
    send_buf[3] = 0b00111111; // PD_INT_M 0x12 unmasking PSRDYI
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
    //printf("opcode_write: 0x%02x 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3]);

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

void test_max77958_get_id(){
    printf("test_max77958_get_id started...\n");
    // Testing for just DEVICE_ID
    // Write the register 0x00 to set the pointer there before reading its value
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 2, false);
    if (return_buf[0] != 0x58){
	printf("DEVICE_ID should be 0x58");
	assert(false);
    }
    if (return_buf[1] != 0x02){
	printf("DEVICE_REV should be 0x02");
	assert(false);
    }
    printf("test_max77958_get_id PASSED: DEVICE_ID = %x\n", return_buf[0]);
    printf("test_max77958_get_id PASSED: DEVICE_REV = %x\n", return_buf[1]);
}

void test_max77958_get_customer_config_id(){
    printf("test_max77958_get_customer_config_id started...\n");
    opcode_queue_add(&customer_config_read, 0);
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
    if (return_buf[0] != 0x55){
	printf("OPCODE should be 0x55");
	assert(false);
    }
    if ((return_buf[2] | (return_buf[3] << 8)) != 0x0B6A){
        printf("CUSTOMER_CONFIG_ID should be 0x0B6A");	
	assert(false);
    }
    if ((return_buf[4] | (return_buf[5] << 8)) != 0x6860){
	printf("CUSTOMER_CONFIG_REV should be 0x6860");
	assert(false);
    }
    printf("test_max77958_get_customer_config_id PASSED: CUSTOMER_CONFIG_ID = 0x%04x\n", (return_buf[2] | (return_buf[3] << 8)));
    printf("test_max77958_get_customer_config_id PASSED: CUSTOMER_CONFIG_REV = 0x%04x\n", (return_buf[4] | (return_buf[5] << 8)));
}

void test_max77958_interrupt(){
    printf("test_max77958_interrupt started...\n");
    set_interrupt_masks_all_masked();

    test_max77958_started = true;
    printf("test_max77958_interrupt: prior to driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    gpio_set_dir(_gpio_interrupt, GPIO_OUT);
    if (gpio_get(_gpio_interrupt) != 0){
	printf("test_max77958_interrupt: GPIO%d was not driven low. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
	assert(false);
    }
    printf("test_max77958_interrupt: after driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    uint32_t i = 0;
    while (!test_max77958_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    printf("test_max77958_interrupt timed out\n");
	    assert(false);
	}
    }
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    test_max77958_started = false;
    test_max77958_completed = false;
    set_interrupt_masks();
    test_max77958_interrupt_bool = false;
    printf("test_max77958_interrupt: PASSED after %" PRIu32 " milliseconds.\n", i*10);
}

static void max77958_test_response(){
    test_max77958_completed = true;
}

void max77958_init(uint gpio_interrupt, queue_t* cq, queue_t* rq){
    _gpio_interrupt = gpio_interrupt;

    printf("max77958 init started\n");
    call_queue_ptr = cq;
    return_queue_ptr = rq;
    queue_init(&opcode_queue, sizeof(queue_entry_t), 16);

    test_max77958_get_id();

    set_interrupt_masks();
    
    // max77958 sends active LOW on INTB connected to GPIO7 on the rp2040. Setup interrupt callback here
    gpio_init(_gpio_interrupt);
    gpio_put(_gpio_interrupt, 0);
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    gpio_set_irq_enabled(_gpio_interrupt, GPIO_IRQ_EDGE_FALL, true); 

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
    printf("max77958 init finished\n");
}

// check if opcode_queue has entries remaining
// if so remove an entry from the opcode_queue and run in on core1 via the call_queue
// return true if an entry was removed and added to the call_queue
// return false if opccode_queue was empty 
static bool opcode_queue_pop(){
    queue_entry_t entry;
    // if there is an entry in the opcode_queue, remove it and add it to the call_queue
    if (queue_try_remove(&opcode_queue, &entry)){
	printf("Removed entry from opcode_queue. %d entries remaining\n", queue_get_level(&opcode_queue));
	// if the call_queue is full, assert
	if(queue_try_add(call_queue_ptr, &entry)){
	    printf("added opcode entry to call_queue\n");
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

static void customer_config_read(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x55; // Customer Configuration Write 
    opcode_write(send_buf);
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

static void vbus_turn_off(){
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(false, true));
    opcode_queue_pop();
}

static void vbus_turn_on(){
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(true, true));
    opcode_queue_pop();
}

static void pd_msg_response(){
    // Read the 0xE PD_STATUS0 register as it contains the PD message Type recieved 
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_PD_STATUS0; // 0xE PD_STATUS0 Register 
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    printf("PD_STATUS0: 0x%02x\n", return_buf[0]);
    switch (return_buf[0]){
        case PDMSG_PRSWAP_SRCTOSWAP:
	    printf("PD Message: PRSWAP_SRCTOSWAP\n");
	    break;
	case PDMSG_PRSWAP_SWAPTOSNK:
	    printf("PD Message: PRSWAP_SWAPTOSNK\n");
	    vbus_turn_off();
	    break;
	case PDMSG_PRSWAP_SNKTOSWAP:
	    printf("PD Message: PRSWAP_SNKTOSWAP\n");
	    break;
	case PDMSG_PRSWAP_SWAPTOSRC:
	    printf("PD Message: PRSWAP_SWAPTOSRC\n");
	    vbus_turn_on();
	    break;
	case PDMSG_VDM_NAK_RECEIVED:
	    printf("PD Message: VDM_NAK Received\n");
	case PDMSG_VDM_BUSY_RECEIVED:
	    printf("PD Message: VDM_BUSY_RECEIVED\n");
	case PDMSG_VDM_ACK_RECEIVED:
	    printf("PD Message: VDM_ACK_RECEIVED\n");
	case PDMSG_VDM_REQ_RECEIVED:
	    printf("PD Message: VDM_REQ_RECEIVED\n");
	    break;
	default:
	    printf("PD Message: Unknown\n");
	    break;
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
