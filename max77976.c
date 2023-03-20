#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77976.h"
#include "bit_ops.h"

void max77976_onEXTUSBCHG_connect(){}
void max77976_onEXTUSBCHG_disconnect(){}
void max77976_onHardwareInterrupt(){
}
void max77976_factory_ship_mode_check(){
    // Check if CONNECTION_ANDROID or CONNECTION_PC occured in last hour

    // Check if CONNECTION_ANDROID or CONNECTION_PC still exists and if so last time command sent

    // If CONNECTION_EXTUSBCHG exists don't enable

    // If not, enable factory ship mode
}

// on interrupt from MAX77976
void on_interrupt(uint gpio, ulong events)
{
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);

    // do something if necessary. Currently don't know a use for this
}

void max77976_init(uint GPIO)
{
    gpio_set_irq_enabled_with_callback(GPIO, GPIO_IRQ_EDGE_RISE, true, &on_interrupt);

    // Set default mode to CHARGE-BUCK
    uint8_t buf[2];
    buf[0] = MAX77976_REG_CHG_CNFG_00_ADDR;
    buf[1] = MAX77976_REG_CHG_CNFG_00_MODE_CHARGE_BUCK;
    i2c_write_blocking(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set Fast-Charge Current limit to 1200mA

    buf[0] = MAX77976_REG_CHG_CNFG_02_ADDR;
    buf[1] = MAX77976_REG_CHG_CNFG_02_CHG_CC_1250;
    i2c_write_blocking(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set switching frequency to 2.6 MHz
    buf[0] = MAX77976_REG_CHG_CNFG_08_ADDR;
    buf[1] = bit_assign(MAX77976_REG_CHG_CNFG_08_RESET,
                        MAX77976_REG_CHG_CNFG_08_FSW_2P6, 
                        MAX77976_REG_CHG_CNFG_08_FSW_LSB); // All reset values are 0 other than FSW. Seeting this also to 0 results in 2.6MHz
    i2c_write_blocking(i2c1, MAX77976_ADDR, buf, 2, false);

    // Set charge input current limit to 3000mA and leave Input Current Limit Soft Start Clock as default value (1024 usec)
    buf[0] = MAX77976_REG_CHG_CNFG_09_ADDR;
    buf[1] = bit_set_range(MAX77976_REG_CHG_CNFG_09_RESET,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_LSB,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_MSB,
                  MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_3000);
    i2c_write_blocking(i2c1, MAX77976_ADDR, buf, 2, false);
}
