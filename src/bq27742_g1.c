#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bq27742_g1.h"
#include "bit_ops.h"
#include <string.h>
#include "robot.h"
#include "custom_printf.h"

static uint8_t send_buf[4];
static uint8_t return_buf[4];
static uint16_t voltage = 0;
static uint16_t temperature = 0;
static uint32_t soh = 0;
static void bq27742_g1_clear_shutdown();

uint16_t bq27742_g1_get_voltage(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&voltage, 0, sizeof(voltage));

    // Test reading the voltage
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x08;
    send_buf[1] = 0x09;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    voltage = (return_buf[1] << 8) | return_buf[0];
    rp2040_log("Voltage: %d\n", (int) voltage);
    return voltage;
}

uint8_t bq27742_g1_get_safety_stats(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x1A;
    send_buf[1] = 0x1B;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);
    
    uint8_t low_byte = return_buf[0];
    bool error = false;
    rp2040_log("SafetyStats: ");
    if (low_byte & ISD_MASK){
        rp2040_log("Internal Short condition detected, ");
        error = true;
    }
    if (low_byte & TDD_MASK){
        rp2040_log("Tab Disconnect condition detected, ");
        error = true;
    }
    if (low_byte & OTC_MASK){
        rp2040_log("Overtemperature in charge condition detected, ");
        error = true;
    }
    if (low_byte & OTD_MASK){
        rp2040_log("Overtemperature in discharge condition detected, ");
        error = true;
    }
    if (low_byte & OVP_MASK){
        rp2040_log("Overvoltage condition detected, ");
        error = true;
    }
    if (low_byte & UVP_MASK){
        rp2040_log("Undervoltage condition detected, ");
        error = true;
    }
    if (!error){
        rp2040_log("No error detected in battery protection\n");
    }
  return low_byte;  
}

uint16_t bq27742_g1_get_temp(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&temperature, 0, sizeof(temperature));

    // Test reading the temperature
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x06;
    send_buf[1] = 0x07;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    // temperature in 0.1 deg Kelvin, so convert to deg C
    float temperature_ = (float)((return_buf[1] << 8) | return_buf[0]);
    temperature_ = (temperature_ - 2731.5);
    temperature_ = temperature_ / 10.0;
    temperature = (uint16_t)temperature;
    rp2040_log("Temperature: %d\n", (int)temperature);
    return temperature; 
}

uint8_t bq27742_g1_get_soh(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&soh, 0, sizeof(uint32_t));
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x2e;
    send_buf[1] = 0x2f;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    rp2040_log("SOH: 0x2e=%02x, 0x2f=%02x\n", return_buf[0], return_buf[1]);
    //float soh = (float)return_buf[0] / 100;
    rp2040_log("SOH: %02f\n", soh);
    // Note in the user guide Section 4.1.24 the range of values is only from 0x00 to 0x64
    return return_buf[0];
}

uint16_t bq27742_g1_get_flags(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x0A;
    send_buf[1] = 0x0B;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);
    
    uint8_t flags = (return_buf[1] << 8) | return_buf[0];
    bool error = false;

    rp2040_log("Tags: ");
    if (flags & BATHI_MASK){
        rp2040_log("High battery voltage condition BATHI detected, ");
        error = true;
    }
    if (flags & BATLOW_MASK){
        rp2040_log("Low battery voltage condition BATLOW detected, ");
        error = true;
    }
    if (flags & CHG_INH_MASK){
        rp2040_log("Temperature is < T1 Temp or > T4 Temp while charging is not active. CHG_INH detected, ");
        error = true;
    }
    if (flags & FC_MASK){
        rp2040_log("Charge termination reached and FC Set Percent = -1. Or SOC > FC Percent is not -1. FC detected, ");
        error = true;
    }
    if (flags & CHG_SUS_MASK){
        rp2040_log("Temp < T1 Temp or > T5 Temp while charging active. CHG_SUS detected, ");
        error = true;
    }
    if (flags & IMAX_MASK){
        rp2040_log("Imax value has changed enough to interrupt. IMAX detected, ");
        error = true;
    }
    if (flags & CHG_MASK){
        rp2040_log("Fast charging allowed. CHG detected, ");
        error = true;
    }
    if (flags & SOC1_MASK){
        rp2040_log("SOC1 reached.");
        error = true;
    }
    if (flags & SOCF_MASK){  
        rp2040_log("SOCF Set Percent reached. SOCF detected, ");
        error = true;
    }
    if (flags & DSG_MASK){
        rp2040_log("Discharging detected. DSG detected, ");
        error = true;
    }
    if (!error){
        rp2040_log("No SystemStat errors detected");
    }
    rp2040_log("\n");
    return flags;
}

void bq27742_g1_init() {
    rp2040_log("bq27742_g1 init started\n");
    bq27742_g1_clear_shutdown();
    rp2040_log("bq27742_g1 init finished\n");
    // uint8_t buf[2];

    // ToDo Implement Key Daya Flash Parameters somehow.
    // buf[0] = BQ227742_G1_REG_REG_CONT1_ADDR;
    // // Change switching current limit threshold ILIM to 2.8A
    // buf[1] = bit_set_range(BQ227742_G1_REG_REG_CONT1_RESET,
    //               BQ227742_G1_REG_REG_CONT1_ILM_LSB,
    //               BQ227742_G1_REG_REG_CONT1_ILM_MSB,
    //               BQ227742_G1_REG_REG_CONT1_ILM_DEFAULT);
    // Change internal compensation (COMP)
}

static void bq27742_g1_control(uint16_t subcommand_code){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x00;
    //send_buf[1] = 0x01; // Apparently this is interpreted as the LSB for the SUBCOMMAND.
    // LSB first (i.e. send 0x02 then 0x00)
    send_buf[1] = (subcommand_code & 0x00FF);
    send_buf[2] = (subcommand_code & 0xFF00) >> 8;
    // This is wrong as it is MSB first
    //send_buf[2] = (subcommand_code & 0xFF00) >> 8;
    //send_buf[3] = (subcommand_code & 0x00FF);
    //
    // void i2c_write_error_handling(i2c_inst_t *i2c, 
    // uint8_t addr, 
    // const uint8_t *src, 
    // size_t len, 
    // bool nostop);
    // This does not work for some unknown reason. I need to write a 0x00 as a 4th byte?
    //i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 3, true);
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 4, true);
    // Move pointer back to Control register for reading
    send_buf[0] = 0x00;
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);
}

static void bq27742_g1_set_shutdown(){
    bq27742_g1_control(0x0013); 
    rp2040_log("Shutting Down bq27742_g1\n");
}

static void bq27742_g1_clear_shutdown(){
    bq27742_g1_control(0x0014); 
    rp2040_log("Clearing Shutdown on bq27742_g1\n");
}

void bq27742_g1_fw_version_check(){
    bq27742_g1_control(0x0002); // Read FW Version
    rp2040_log("FW Version: 0x%02x%02x\n", return_buf[1], return_buf[0]);
}

void bq27742_g1_shutdown(){
    bq27742_g1_set_shutdown();
}
