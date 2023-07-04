#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77976.h"
#include "bit_ops.h"
#include <string.h>
#include "robot.h"

static void max77976_onEXTUSBCHG_connect();
static void max77976_onEXTUSBCHG_disconnect();
static void max77976_onHardwareInterrupt();
static uint8_t send_buf[3];
static uint8_t return_buf[2];
static uint8_t _gpio;

void max77976_factory_ship_mode_check(){
    // Check if CONNECTION_ANDROID or CONNECTION_PC occured in last hour

    // Check if CONNECTION_ANDROID or CONNECTION_PC still exists and if so last time command sent

    // If CONNECTION_EXTUSBCHG exists don't enable

    // If not, enable factory ship mode
}

// on interrupt from MAX77976
void max77976_on_battery_charger_interrupt(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE){
	gpio_acknowledge_irq(gpio, GPIO_IRQ_EDGE_RISE);
	// printf("Rising edge detected on max77976.\n");
	// remember this should only add to the call_queue, not execute the function
    }
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);

    // do something if necessary. Currently don't know a use for this
}


int max77976_init(uint GPIO){
    _gpio = GPIO;

    gpio_set_irq_enabled(_gpio, GPIO_IRQ_EDGE_RISE, true);
    
    // Check if responding as i2c slave before trying to write to it
    uint8_t rxdata;
    int ret;
    i2c_read_error_handling(i2c1,MAX77976_ADDR, &rxdata, 1, false);

    uint8_t buf[2];
    i2c_write_error_handling(i2c1, MAX77976_ADDR, 0x0, 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, &rxdata, 1, false);
    printf("Read CHIP_ID %x.\n", rxdata);
    if (rxdata != 0x76){
	printf("MAX77976 not responding. Exiting.\n");
	assert(false);
    }

    // Unlock the write capability of CHGPROT
    buf[0] = 0x1C; // CHAG_CNFG_06 
    buf[1] = 0x0C;   
    i2c_write_error_handling(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set default mode to CHARGE-BUCK
    buf[0] = MAX77976_REG_CHG_CNFG_00_ADDR;
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

    printf("CHGIN_ILIM: 0x%02x\n", CHGIN_ILIM);
    printf("CHG_CC: 0x%02x\n", CHG_CC);
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
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[0], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_00 = return_buf[0];
    uint8_t CHGIN_DTLS = (CHG_DETAILS_00 & 0x60) >> 5;
    printf("CHG_DETAILS_00: 0x%02x\n CHGIN_DTLS: 0x%02x\n", CHG_DETAILS_00, CHGIN_DTLS);
    printf("CHG_DETAILS_00_CHGIN_DTLS: ");
    switch (CHGIN_DTLS){
        case 0b00:
	    printf("VBUS is invalid. VCHGIN rising: VCHGIN < VCHGIN_UVLO. VCHGIN falling: VCHGIN < VCHGIN_REG (AICL)");
	    break;
        case 0b01:
	    printf("VBUS is invalid. VCHGIN < VBATT + VCHGIN2SYS and VCHGIN > VCHGIN_UVLO");
	    break;
        case 0b10:
	    printf("VBUS is invalid. VCHGIN > VCHGIN_OVLO");
	    break;
        case 0b11:
	    printf("VBUS is valid. VCHGIN > VCHGIN_UVLO and VCHGIN > VBATT + VCHGIN2SYS and VCHGIN < VCHGIN_OVLO");
	    break;
    }printf("\n"); 
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[1], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_01 = return_buf[0];
    uint8_t TREG = (CHG_DETAILS_01 & (1 << 7) ) >> 7;
    uint8_t BAT_DTLS = (CHG_DETAILS_01 & 0x70) >> 4;
    uint8_t CHG_DTLS = (CHG_DETAILS_01 & 0xF);
    printf("CHG_DETAILS_01: 0x%02x\n TREG: 0x%02x\n BAT_DTLS: 0x%02x\n CHG_DTLS: 0x%02x\n", CHG_DETAILS_01, TREG, BAT_DTLS, CHG_DTLS);
    printf("CHG_DETAILS_01_TREG: ");
    switch (TREG){
        case 0b0:
	    printf("The junction temperature is less than the threshold set by REGTEMP and the full charge current limit is available");
	    break;
	case 0b1:
	    printf("The junction temperature is greater than the threshold set by REGTEMP and the charge current limit may be folding back to reduce power dissipation.");
    }printf("\n"); 

    printf("CHG_DETAILS_01_BAT_DTLS: ");
    switch (BAT_DTLS){
        case 0b000:
	    printf("Battery Removal");
	    break;
        case 0b001:
	    printf("Battery Prequalification Voltage");
	    break;
        case 0b010:
	    printf("Battery Timer Fault");
	    break;
        case 0b011:
	    printf("Battery Regular Voltage");
	    break;
        case 0b100:
	    printf("Battery Low Voltage");
	    break;
        case 0b101:
	    printf("Battery Overvoltage");
	    break;
        case 0b110:
	    printf("Reserved");
	    break;
        case 0b111:
	    printf("Battery Only");
	    break;
    }printf("\n"); 
    
    printf("CHG_DETAILS_01_CHG_DTLS: ");
    switch (CHG_DTLS){
        case 0x00:
	    printf("Charger is in dead-battery prequalification or low-battery prequalification mode.");
	    break;
        case 0x01:
	    printf("Charger is in fast-charge constant current mode.");
	    break;
        case 0x02:
	    printf("Charger is in fast-charge constant voltage mode.");
	    break;
        case 0x03:
	    printf("Charger is in top-off mode.");
	    break;
        case 0x04:
	    printf("Charger is in done mode.");
	    break;
        case 0x05:
	    printf("Reserved");
	    break;
        case 0x06:
	    printf("Charger is in timer-fault mode.");
	    break;
        case 0x07:
	    printf("Charger is suspended because QBATT is disabled");
	    break;
        case 0x08:
	    printf("Charger is off, charger input invalid and/or charger is disabled.");
	    break;
        case 0x09:
	    printf("Reserved");
	    break;
        case 0x0A:
	    printf("Charger is off and the junction temperature is > TSHDN.");
	    break;
        case 0x0B:
	    printf("Charger is off because the watchdog timer expired");
	    break;
        case 0x0C:
	    printf("Charger is suspended or charge current or voltage is reduced based on JEITA control.");
	    break;
        case 0x0D:
	    printf("Charger is suspended because battery removal is detected on THM pin.");
	    break;
        case 0x0E:
	    printf("Charger is suspended because SUSPEND pin is high.");
	    break;
        case 0x0F:
	    printf("Reserved");
	    break;
    }printf("\n"); 
    
    i2c_write_error_handling(i2c1, MAX77976_ADDR, &send_buf[2], 1, true);
    i2c_read_error_handling(i2c1, MAX77976_ADDR, return_buf, 1, false);
    uint8_t CHG_DETAILS_02 = return_buf[0];
    uint8_t THM_DTLS = (CHG_DETAILS_02 & 0x70) >> 4;
    uint8_t BYP_DTLS = (CHG_DETAILS_02 & 0x0F);
    printf("CHG_DETAILS_02_BYP_DTLS: ");
    switch (BYP_DTLS){
        case 0x00:
	    printf("The bypass node is okay.");
	    break;
        case 0x01:
	    printf("OTG_ILIM when CHG_CNFG_00.MODE=0xA or 0xE or 0xF");
	    break;
        case 0x02:
	    printf("BSTILIM");
	    break;
        case 0x04:
	    printf("BCKNegILIM");
	    break;
        case 0x08:
	    printf("BST_SWON_DONE");
	    break;
    }printf("\n"); 

}

void max77976_shutdown(){
}
