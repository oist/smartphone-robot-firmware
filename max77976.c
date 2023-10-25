#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77976.h"
#include "bit_ops.h"
#include <string.h>
#include "robot.h"
#include <inttypes.h>
#include "custom_printf.h"

static void max77976_onEXTUSBCHG_connect();
static void max77976_onEXTUSBCHG_disconnect();
static void max77976_onHardwareInterrupt();
static int32_t max77976_parse_interrupt_vals();
static void max77976_set_interrupt_masks();
static void max77976_set_interrupt_masks_all_masked();
static int32_t max77976_test_response();
static void max77976_get_interrupt_vals(uint8_t* buf_ptr) ;
static uint8_t send_buf[4];
static uint8_t return_buf[2];
static uint8_t _gpio_interrupt;
static bool test_max77976_interrupt_bool = false;
static queue_t* call_queue_ptr;
static queue_t* return_queue_ptr;
static uint8_t interrupt_mask = GPIO_IRQ_EDGE_FALL;
static bool test_max77976_started = false;
static bool test_max77976_completed = false;

void max77976_factory_ship_mode_check(){
    // Check if CONNECTION_ANDROID or CONNECTION_PC occured in last hour

    // Check if CONNECTION_ANDROID or CONNECTION_PC still exists and if so last time command sent

    // If CONNECTION_EXTUSBCHG exists don't enable

    // If not, enable factory ship mode
}

// on interrupt from MAX77976
void max77976_on_battery_charger_interrupt(uint gpio, uint32_t event_mask){
    if (event_mask & interrupt_mask){
        gpio_acknowledge_irq(_gpio_interrupt, interrupt_mask);	
	call_queue_try_add(&max77976_parse_interrupt_vals, 0);
	//synchronized_printf("MAX77976 added max77976_parse_interrupt_vals to call_queue.\n");
        if (test_max77976_started){
            call_queue_try_add(&max77976_test_response, 1);
        }
    }
}

static int32_t max77976_parse_interrupt_vals(){
    uint8_t buf[MAX77976_INT_BUF_LEN];
    max77976_get_interrupt_vals(buf);
    uint8_t AICL_I = 1 << 7;
    uint8_t CHGIN_I = 1 << 6;
    uint8_t INLIM_I = 1 << 5;
    uint8_t CHG_I = 1 << 4;
    uint8_t BAT_I = 1 << 3;
    uint8_t DISQBAT_I = 1 << 1;
    uint8_t BYP_I = 1 << 0;

    if (buf[0] & AICL_I){
	//synchronized_printf("MAX77976: AICL_I interrupt detected.\n");
        if (buf[2] & AICL_I){
	    //synchronized_printf("MAX77976: AICL mode.\n");
	}else {
	    //synchronized_printf("MAX77976: AICL mode not detected.\n");
        }
    }
    if (buf[0] & CHGIN_I){
	//synchronized_printf("MAX77976: CHGIN_I interrupt detected.\n");
        if (buf[2] & CHGIN_I){
	    //synchronized_printf("MAX77976: CHGIN input is valid.\n");
            // Set Charge mode to default mode to Charge Buck while charging
            //buf[0] = MAX77976_REG_CHG_CNFG_00_ADDR;
            //buf[1] = MAX77976_REG_CHG_CNFG_00_MODE_CHARGE_BUCK;
            //i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);
	}else {
	    //synchronized_printf("MAX77976: CHGIN input is not valid.\n");
            // Set mode to Battery-boot (flash) while no charger present
            //buf[0] = MAX77976_REG_CHG_CNFG_00_ADDR;
            //buf[1] = MAX77976_REG_CHG_CNFG_00_MODE_BATTERY_BOOST_FLASH;
            //i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);
        }
    }
    if (buf[0] & INLIM_I){
	//synchronized_printf("MAX77976: INLIM_I interrupt detected.\n");
        if (buf[2] & INLIM_I){
	    //synchronized_printf("MAX77976: The CHGIN input current has been reaching the current limit for at least 30ms.\n");
        }else {
	    //synchronized_printf("MAX77976: The CHGIN input current has not reached the current limit.\n");
	}
    }
    if (buf[0] & CHG_I){
	//synchronized_printf("MAX77976: CHG_I interrupt detected.\n");
        if (buf[2] & CHG_I){
	    //synchronized_printf("MAX77976: The charger has suspended charging or TREG = 1.\n");
        }else {
	    //synchronized_printf("MAX77976: The charger is okay or the charger is off.\n");
	}
    }
    if (buf[0] & BAT_I){
	//synchronized_printf("MAX77976: BAT_I interrupt detected.\n");
        if (buf[2] & BAT_I){
	    //synchronized_printf("MAX77976: The battery has an issue or the charger has been suspended.\n");
        }else {
	    //synchronized_printf("MAX77976: The battery is okay.\n");
	}
    }
    if (buf[0] & DISQBAT_I){
	//synchronized_printf("MAX77976: DISQBAT_I interrupt detected.\n");
        if (buf[2] & DISQBAT_I){
	    //synchronized_printf("MAX77976: DISQBAT is high and QBATT is disabled.\n");
        }else {
	    //synchronized_printf("MAX77976: DISQBAT is low and QBATT is not disabled.\n");
	}
    }
    if (buf[0] & BYP_I){
	//synchronized_printf("MAX77976: BYP_I interrupt detected.\n");
        if (buf[2] & BYP_I){
	    //synchronized_printf("MAX77976: Something powered by the bypass node has hit current limit.\n");
        }else {
	    //synchronized_printf("MAX77976: The bypass node is okay.\n");
	}
    }
    if (buf[0] == 0){
        test_max77976_interrupt_bool = true;  
    }
    return buf[1] << 8 | buf[0];
}

static void max77976_get_interrupt_vals(uint8_t* buf_ptr) {
    memset(buf_ptr, 0, sizeof MAX77976_INT_BUF_LEN);
    uint8_t addr = MAX77976_REG_CHG_INT; // 0x04 Register
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &addr, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, buf_ptr, 3, false);
    //synchronized_printf("MAX77976 interrupts vals: 0x10: 0x%02x, 0x11: 0x%02x, 0x12: 0x%02x\n", buf_ptr[0], buf_ptr[1], buf_ptr[2]);
}


int max77976_init(uint gpio_interrupt, queue_t* cq, queue_t* rq){
    _gpio_interrupt = gpio_interrupt;

    //synchronized_printf("max77958 init started\n");
    call_queue_ptr = cq;
    return_queue_ptr = rq;

    // max77976 sends active LOW on IRQB connected to GPIO6 on the rp2040. Setup interrupt callback here
    gpio_init(_gpio_interrupt);
    gpio_put(_gpio_interrupt, 0);
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    gpio_set_irq_enabled(_gpio_interrupt, GPIO_IRQ_EDGE_FALL, true);
    uint8_t buf[MAX77976_INT_BUF_LEN];

    max77976_set_interrupt_masks();

    // clear interupts
    max77976_get_interrupt_vals(buf);
    memset(buf, 0, MAX77976_INT_BUF_LEN);

    // Unlock the write capability of CHGPROT
    buf[0] = 0x1C; // CHAG_CNFG_06 
    buf[1] = 0x0C;   
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set default mode to Battery-boot (flash)
    buf[0] = MAX77976_REG_CHG_CNFG_00_ADDR;
    //buf[1] = MAX77976_REG_CHG_CNFG_00_MODE_BATTERY_BOOST_FLASH;
    buf[1] = MAX77976_REG_CHG_CNFG_00_MODE_CHARGE_BUCK;
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set Fast-Charge Current limit to 1200mA
    buf[0] = MAX77976_REG_CHG_CNFG_02_ADDR;
    buf[1] = MAX77976_REG_CHG_CNFG_02_CHG_CC_1250;
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set switching frequency to 2.6 MHz
    buf[0] = MAX77976_REG_CHG_CNFG_08_ADDR;
    buf[1] = bit_assign(MAX77976_REG_CHG_CNFG_08_RESET,
                        MAX77976_REG_CHG_CNFG_08_FSW_2P6, 
                        MAX77976_REG_CHG_CNFG_08_FSW_LSB); // All reset values are 0 other than FSW. Seeting this also to 0 results in 2.6MHz
    //buf[1] = MAX77976_REG_CHG_CNFG_08_RESET;
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set charge input current limit to 3000mA and leave Input Current Limit Soft Start Clock as default value (1024 usec)
    buf[0] = MAX77976_REG_CHG_CNFG_09_ADDR;
    buf[1] = bit_set_range(MAX77976_REG_CHG_CNFG_09_RESET,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_LSB,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_MSB,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_3000);
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Lock the write capability of CHGPROT
    buf[0] = 0x1C; // CHAG_CNFG_06 
    buf[1] = 0x00;   
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);
    return 0;
}

void max77976_log_current_limit(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x1F; //CHG_CNFG_09
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);

    uint8_t CHGIN_ILIM = return_buf[0] & 0x3F;
    
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x18; //CHG_CNFG_02
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);

    uint8_t CHG_CC = return_buf[0] & 0x7F;

    //synchronized_printf("CHGIN_ILIM: 0x%02x\n", CHGIN_ILIM);
    //synchronized_printf("CHG_CC: 0x%02x\n", CHG_CC);
}

void max77976_toggle_led(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x24; //STAT_CNFG
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);

    uint8_t STAT_EN = (return_buf[0] & (1<<7)) >> 7;
    if (STAT_EN){
        send_buf[1] = 0x00; // Turn off
        i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 2, false);
    }else{
        send_buf[1] = 0x80; // Turn on
        i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 2, false);
    }
}

void max77976_get_chg_details(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x13; //CHG_DETAILS_00
    send_buf[1] = 0x14; //CHG_DETAILS_01
    send_buf[2] = 0x15; //CHG_DETAILS_02
    send_buf[3] = 0x16; //CHG_CNFG_00
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[0], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_00 = return_buf[0];
    uint8_t CHGIN_DTLS = (CHG_DETAILS_00 & 0x60) >> 5;
    //synchronized_printf("CHG_DETAILS_00: 0x%02x\n CHGIN_DTLS: 0x%02x\n", CHG_DETAILS_00, CHGIN_DTLS);
    //synchronized_printf("CHG_DETAILS_00_CHGIN_DTLS: ");
    switch (CHGIN_DTLS){
        case 0b00:
	    //synchronized_printf("VBUS is invalid. VCHGIN rising: VCHGIN < VCHGIN_UVLO. VCHGIN falling: VCHGIN < VCHGIN_REG (AICL)");
	    break;
        case 0b01:
	    //synchronized_printf("VBUS is invalid. VCHGIN < VBATT + VCHGIN2SYS and VCHGIN > VCHGIN_UVLO");
	    break;
        case 0b10:
	    //synchronized_printf("VBUS is invalid. VCHGIN > VCHGIN_OVLO");
	    break;
        case 0b11:
	    //synchronized_printf("VBUS is valid. VCHGIN > VCHGIN_UVLO and VCHGIN > VBATT + VCHGIN2SYS and VCHGIN < VCHGIN_OVLO");
	    break;
    }//synchronized_printf("\n"); 
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[1], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_01 = return_buf[0];
    uint8_t TREG = (CHG_DETAILS_01 & (1 << 7) ) >> 7;
    uint8_t BAT_DTLS = (CHG_DETAILS_01 & 0x70) >> 4;
    uint8_t CHG_DTLS = (CHG_DETAILS_01 & 0xF);
    //synchronized_printf("CHG_DETAILS_01: 0x%02x\n TREG: 0x%02x\n BAT_DTLS: 0x%02x\n CHG_DTLS: 0x%02x\n", CHG_DETAILS_01, TREG, BAT_DTLS, CHG_DTLS);
    //synchronized_printf("CHG_DETAILS_01_TREG: ");
    switch (TREG){
        case 0b0:
	    //synchronized_printf("The junction temperature is less than the threshold set by REGTEMP and the full charge current limit is available");
	    break;
	case 0b1:
	    //synchronized_printf("The junction temperature is greater than the threshold set by REGTEMP and the charge current limit may be folding back to reduce power dissipation.");
	    break;
    }
    //synchronized_printf("\n"); 

    //synchronized_printf("CHG_DETAILS_01_BAT_DTLS: ");
    switch (BAT_DTLS){
        case 0b000:
	    //synchronized_printf("Battery Removal");
	    break;
        case 0b001:
	    //synchronized_printf("Battery Prequalification Voltage");
	    break;
        case 0b010:
	    //synchronized_printf("Battery Timer Fault");
	    break;
        case 0b011:
	    //synchronized_printf("Battery Regular Voltage");
	    break;
        case 0b100:
	    //synchronized_printf("Battery Low Voltage");
	    break;
        case 0b101:
	    //synchronized_printf("Battery Overvoltage");
	    break;
        case 0b110:
	    //synchronized_printf("Reserved");
	    break;
        case 0b111:
	    //synchronized_printf("Battery Only");
	    break;
    }//synchronized_printf("\n"); 
    
    //synchronized_printf("CHG_DETAILS_01_CHG_DTLS: ");
    switch (CHG_DTLS){
        case 0x00:
	    //synchronized_printf("Charger is in dead-battery prequalification or low-battery prequalification mode.");
	    break;
        case 0x01:
	    //synchronized_printf("Charger is in fast-charge constant current mode.");
	    break;
        case 0x02:
	    //synchronized_printf("Charger is in fast-charge constant voltage mode.");
	    break;
        case 0x03:
	    //synchronized_printf("Charger is in top-off mode.");
	    break;
        case 0x04:
	    //synchronized_printf("Charger is in done mode.");
	    break;
        case 0x05:
	    //synchronized_printf("Reserved");
	    break;
        case 0x06:
	    //synchronized_printf("Charger is in timer-fault mode.");
	    break;
        case 0x07:
	    //synchronized_printf("Charger is suspended because QBATT is disabled");
	    break;
        case 0x08:
	    //synchronized_printf("Charger is off, charger input invalid and/or charger is disabled.");
	    break;
        case 0x09:
	    //synchronized_printf("Reserved");
	    break;
        case 0x0A:
	    //synchronized_printf("Charger is off and the junction temperature is > TSHDN.");
	    break;
        case 0x0B:
	    //synchronized_printf("Charger is off because the watchdog timer expired");
	    break;
        case 0x0C:
	    //synchronized_printf("Charger is suspended or charge current or voltage is reduced based on JEITA control.");
	    break;
        case 0x0D:
	    //synchronized_printf("Charger is suspended because battery removal is detected on THM pin.");
	    break;
        case 0x0E:
	    //synchronized_printf("Charger is suspended because SUSPEND pin is high.");
	    break;
        case 0x0F:
	    //synchronized_printf("Reserved");
	    break;
    }//synchronized_printf("\n"); 
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[2], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_02 = return_buf[0];
    uint8_t THM_DTLS = (CHG_DETAILS_02 & 0x70) >> 4;
    uint8_t BYP_DTLS = (CHG_DETAILS_02 & 0x0F);
    //synchronized_printf("CHG_DETAILS_02_BYP_DTLS: ");
    switch (BYP_DTLS){
        case 0x00:
	    //synchronized_printf("The bypass node is okay.");
	    break;
        case 0x01:
	    //synchronized_printf("OTG_ILIM when CHG_CNFG_00.MODE=0xA or 0xE or 0xF");
	    break;
        case 0x02:
	    //synchronized_printf("BSTILIM");
	    break;
        case 0x04:
	    //synchronized_printf("BCKNegILIM");
	    break;
        case 0x08:
	    //synchronized_printf("BST_SWON_DONE");
	    break;
    }//synchronized_printf("\n"); 

    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[3], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_CNFG_00 = return_buf[0];
    uint8_t _MODE = (CHG_CNFG_00 & 0x0F);
    //synchronized_printf("CHG_CNFG_00: 0x%x\n", _MODE);

}

void max77976_shutdown(){
}

// Mask all interrupts for unit test purposes
static void max77976_set_interrupt_masks_all_masked(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = MAX77976_REG_CHG_INT_MASK;
    send_buf[1] = 0b11111111;
    i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 4, false);
}

// Note this also masks everything, but that is the defualt reset values.
// Will need to change this later when implementing the interrupts for detecting
// charger interrupt
static void max77976_set_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = MAX77976_REG_CHG_INT_MASK; 
    uint8_t AICL_M = 1 << 7;
    uint8_t CHGIN_M = 0 << 6; // Turning off this mask
    uint8_t INLIM_M = 1 << 5;
    uint8_t CHG_M = 1 << 4;
    uint8_t BAT_M = 1 << 3;
    uint8_t SPR2 = 1 << 2;
    uint8_t DISQBAT_M = 1 << 1;
    uint8_t BYP_M = 1 << 0;

    send_buf[1] = AICL_M | CHGIN_M | INLIM_M | CHG_M | BAT_M | SPR2 | DISQBAT_M | BYP_M;;  
    i2c_write_error_handling(i2c1, MAX77976_ADDR, send_buf, 4, false);
}

void test_max77976_get_id(){
    //synchronized_printf("test_max77976_get_id started...\n"); 
    max77976_set_interrupt_masks_all_masked();
    // Check if responding as i2c slave before trying to write to it
    uint8_t rxdata;

    i2c_write_error_handling(i2c1, MAX77976_ADDR, 0x0, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, &rxdata, 1, false);
    if (rxdata != 0x76){
	//synchronized_printf("MAX77976 not responding. Exiting.\n");
	assert(false);
    }
    max77976_set_interrupt_masks();
    //synchronized_printf("test_max77976_get_id PASSED. Read CHIP_ID %x.\n", rxdata);
}

void test_max77976_get_FSW(){
    //synchronized_printf("test_max77976_get_FSW started...\n"); 
    max77976_set_interrupt_masks_all_masked();
    uint8_t rxdata;

    uint8_t reg = MAX77976_REG_CHG_CNFG_08_ADDR;
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &reg, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, &rxdata, 1, false);
    if (rxdata != 0x0){
	//synchronized_printf("MAX77976 FSW Not 0x0. Exiting.\n");
	assert(false);
    }
    max77976_set_interrupt_masks();
    //synchronized_printf("test_max77976_get_FSW PASSED. Read register 0x1E to be %x.\n", rxdata);
}

static int32_t max77976_test_response(){
    test_max77976_completed = true;
    return 0;
}

void test_max77976_interrupt(){
    //synchronized_printf("test_max77976_interrupt starting...\n");
    max77976_set_interrupt_masks_all_masked();
    test_max77976_started = true;
    //synchronized_printf("test_max77976_interrupt: prior to driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    gpio_set_dir(_gpio_interrupt, GPIO_OUT);
    if (gpio_get(_gpio_interrupt) != 0){
	//synchronized_printf("test_max77976_interrupt: GPIO%d was not driven low. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
	assert(false);
    }
    //synchronized_printf("test_max77976_interrupt: after driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    uint32_t i = 0;
    while (!test_max77976_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    //synchronized_printf("test_max77976_interrupt timed out\n");
	    assert(false);
	}
    }
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    test_max77976_started = false;
    test_max77976_completed = false;
    max77976_set_interrupt_masks();

    //synchronized_printf("test_max77976_interrupt: Passed after %" PRIu32 " milliseconds.\n", i*10);
}
