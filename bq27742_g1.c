#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bq27742_g1.h"
#include "bit_ops.h"
#include <string.h>

static uint8_t send_buf[2];
static uint8_t return_buf[2];
static uint32_t voltage = 0;
static uint32_t temperature = 0;
static uint32_t soh = 0;

void bq27742_g1_get_voltage(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&voltage, 0, sizeof(uint32_t));

    // Test reading the voltage
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x08;
    send_buf[1] = 0x09;
    i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    voltage = (return_buf[1] << 8) | return_buf[0];
    printf("Voltage: %d\n", voltage);
}

void bq27742_g1_get_safety_stats(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x1A;
    send_buf[1] = 0x1B;
    i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);
    
    uint8_t low_byte = return_buf[0];
    bool error = false;
    printf("SafetyStats: ");
    if (low_byte & ISD_MASK){
        printf("Internal Short condition detected, ");
        error = true;
    }
    if (low_byte & TDD_MASK){
        printf("Tab Disconnect condition detected, ");
        error = true;
    }
    if (low_byte & OTC_MASK){
        printf("Overtemperature in charge condition detected, ");
        error = true;
    }
    if (low_byte & OTD_MASK){
        printf("Overtemperature in discharge condition detected, ");
        error = true;
    }
    if (low_byte & OVP_MASK){
        printf("Overvoltage condition detected, ");
        error = true;
    }
    if (low_byte & UVP_MASK){
        printf("Undervoltage condition detected, ");
        error = true;
    }
    if (!error){
        printf("No error detected in battery protection\n");
    }
    
}

void bq27742_g1_get_temp(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&temperature, 0, sizeof(uint32_t));

    // Test reading the temperature
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x06;
    send_buf[1] = 0x07;
    i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    uint32_t temperature_k = ((return_buf[1] << 8) | return_buf[0]) / 10;
    temperature = temperature_k - 273;
    printf("Temperature: %d\n", temperature);
}

void bq27742_g1_get_soh(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&soh, 0, sizeof(uint32_t));
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = 0x2e;
    send_buf[1] = 0x2f;
    i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);

    printf("SOH: 0x2e=%02x, 0x2f=%02x\n", return_buf[0], return_buf[1]);
    float soh = (float)return_buf[0] / 100;
    printf("SOH: %02f\n", soh);
}

void bq27742_g1_get_flags(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x0A;
    send_buf[1] = 0x0B;
    i2c_write_blocking(i2c0, BQ27742_G1_ADDR, send_buf, 1, true);
    i2c_read_blocking(i2c0, BQ27742_G1_ADDR, return_buf, 2, false);
    
    uint8_t flags = (return_buf[1] << 8) | return_buf[0];
    bool error = false;

    printf("Tags: ");
    if (flags & BATHI_MASK){
        printf("High battery voltage condition BATHI detected, ");
        error = true;
    }
    if (flags & BATLOW_MASK){
        printf("Low battery voltage condition BATLOW detected, ");
        error = true;
    }
    if (flags & CHG_INH_MASK){
        printf("Temperature is < T1 Temp or > T4 Temp while charging is not active. CHG_INH detected, ");
        error = true;
    }
    if (flags & FC_MASK){
        printf("Charge termination reached and FC Set Percent = -1. Or SOC > FC Percent is not -1. FC detected, ");
        error = true;
    }
    if (flags & CHG_SUS_MASK){
        printf("Temp < T1 Temp or > T5 Temp while charging active. CHG_SUS detected, ");
        error = true;
    }
    if (flags & IMAX_MASK){
        printf("Imax value has changed enough to interrupt. IMAX detected, ");
        error = true;
    }
    if (flags & CHG_MASK){
        printf("Fast charging allowed. CHG detected, ");
        error = true;
    }
    if (flags & SOC1_MASK){
        printf("SOC1 reached.");
        error = true;
    }
    if (flags & SOCF_MASK){  
        printf("SOCF Set Percent reached. SOCF detected, ");
        error = true;
    }
    if (flags & DSG_MASK){
        printf("Discharging detected. DSG detected, ");
        error = true;
    }
    if (!error){
        printf("No SystemStat errors detected");
    }
    printf("\n");
}

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
}
