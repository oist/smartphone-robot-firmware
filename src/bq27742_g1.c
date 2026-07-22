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
static uint8_t _gpio_interrupt;
static volatile bool bq27742_g1_interrupt_pending = false;
static const uint32_t bq27742_g1_irq_mask = GPIO_IRQ_EDGE_FALL;
static const uint32_t bq27742_g1_deassert_irq_mask = GPIO_IRQ_EDGE_RISE;
static void bq27742_g1_clear_shutdown();
static void bq27742_g1_control(uint16_t subcommand_code);
static uint16_t bq27742_g1_get_pack_configuration();
static int32_t bq27742_g1_parse_interrupt_vals(int32_t unused);
static void bq27742_g1_queue_parse_interrupt();
static void bq27742_g1_rearm_assert_irq();
static void bq27742_g1_configure_host_interrupts();
static bool bq27742_g1_read_data_flash_block(uint8_t subclass, uint8_t block, uint8_t *block_data);
static void bq27742_g1_write_data_flash_block(uint8_t subclass, uint8_t block, const uint8_t *block_data);
static uint8_t bq27742_g1_data_flash_checksum(const uint8_t *block_data);
static void bq27742_g1_write_byte(uint8_t command, uint8_t value);
static void bq27742_g1_read_bytes(uint8_t command, uint8_t *buf, size_t len);

#define BQ27742_G1_REG_FLAGS 0x0A
#define BQ27742_G1_REG_SAFETY_STATUS 0x1A
#define BQ27742_G1_REG_PACK_CONFIGURATION 0x3A
#define BQ27742_G1_REG_DATA_FLASH_CLASS 0x3E
#define BQ27742_G1_REG_DATA_FLASH_BLOCK 0x3F
#define BQ27742_G1_REG_BLOCK_DATA 0x40
#define BQ27742_G1_REG_BLOCK_DATA_CHECKSUM 0x60
#define BQ27742_G1_REG_BLOCK_DATA_CONTROL 0x61

#define BQ27742_G1_DF_SUBCLASS_REGISTERS 64
#define BQ27742_G1_DF_BLOCK_REGISTERS 0
#define BQ27742_G1_DF_BLOCK_LEN 32
#define BQ27742_G1_DF_PACK_CONFIG_OFFSET 0
#define BQ27742_G1_DF_PACK_CONFIG_C_OFFSET 3

#define BQ27742_G1_PACK_CONFIG_HOSTIPU_MASK (1 << 14)
#define BQ27742_G1_PACK_CONFIG_HOSTIPOL_MASK (1 << 13)
#define BQ27742_G1_PACK_CONFIG_HOSTIE_MASK (1 << 12)
#define BQ27742_G1_PACK_CONFIG_C_BTP_EN_MASK (1 << 0)
#define BQ27742_G1_CONTROL_IMAX_INT_CLEAR 0x0023

uint16_t bq27742_g1_get_voltage(){
    memset(return_buf, 0, sizeof return_buf);
    memset(&voltage, 0, sizeof(voltage));

    // Test reading the voltage
    memset(send_buf, 0, sizeof send_buf);
    bq27742_g1_read_bytes(0x08, return_buf, 2);

    voltage = (return_buf[1] << 8) | return_buf[0];
    rp2040_log("Voltage: %d\n", (int) voltage);
    return voltage;
}

uint8_t bq27742_g1_get_safety_stats(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    bq27742_g1_read_bytes(BQ27742_G1_REG_SAFETY_STATUS, return_buf, 2);
    
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
    bq27742_g1_read_bytes(0x06, return_buf, 2);

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
    bq27742_g1_read_bytes(0x2e, return_buf, 2);

    rp2040_log("SOH: 0x2e=%02x, 0x2f=%02x\n", return_buf[0], return_buf[1]);
    //float soh = (float)return_buf[0] / 100;
    rp2040_log("SOH: %02f\n", soh);
    // Note in the user guide Section 4.1.24 the range of values is only from 0x00 to 0x64
    return return_buf[0];
}

uint16_t bq27742_g1_get_flags(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    bq27742_g1_read_bytes(BQ27742_G1_REG_FLAGS, return_buf, 2);
    
    uint16_t flags = (return_buf[1] << 8) | return_buf[0];
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

void bq27742_g1_init(uint gpio_interrupt) {
    _gpio_interrupt = gpio_interrupt;

    rp2040_log("bq27742_g1 init started\n");
    bq27742_g1_clear_shutdown();
    bq27742_g1_configure_host_interrupts();

    gpio_init(_gpio_interrupt);
    gpio_put(_gpio_interrupt, 0);
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_disable_pulls(_gpio_interrupt);
    gpio_set_irq_enabled(_gpio_interrupt, bq27742_g1_irq_mask, true);
    if (!gpio_get(_gpio_interrupt)){
        rp2040_log("BQ27742-G1 RC2_3V3 already asserted on GPIO%d\n", _gpio_interrupt);
        bq27742_g1_queue_parse_interrupt();
    }
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

void bq27742_g1_on_interrupt(uint gpio, uint32_t event_mask){
    if (gpio != _gpio_interrupt){
        return;
    }

    if (event_mask & bq27742_g1_irq_mask){
        gpio_acknowledge_irq(_gpio_interrupt, bq27742_g1_irq_mask);
        bq27742_g1_queue_parse_interrupt();
    }
    if (event_mask & bq27742_g1_deassert_irq_mask){
        gpio_acknowledge_irq(_gpio_interrupt, bq27742_g1_deassert_irq_mask);
        if (gpio_get(_gpio_interrupt)){
            bq27742_g1_rearm_assert_irq();
        }
    }
}

static void bq27742_g1_queue_parse_interrupt(){
    if (bq27742_g1_interrupt_pending){
        return;
    }

    bq27742_g1_interrupt_pending = true;
    gpio_set_irq_enabled(_gpio_interrupt, bq27742_g1_irq_mask, false);
    gpio_set_irq_enabled(_gpio_interrupt, bq27742_g1_deassert_irq_mask, true);
    if (!call_queue_try_add_nonblocking(&bq27742_g1_parse_interrupt_vals, 0)){
        if (gpio_get(_gpio_interrupt)){
            bq27742_g1_rearm_assert_irq();
        }
    }
}

static int32_t bq27742_g1_parse_interrupt_vals(int32_t unused){
    (void)unused;

    uint8_t flags_buf[2];
    uint8_t safety_buf[2];
    bq27742_g1_read_bytes(BQ27742_G1_REG_FLAGS, flags_buf, 2);
    bq27742_g1_read_bytes(BQ27742_G1_REG_SAFETY_STATUS, safety_buf, 2);

    uint16_t flags = (flags_buf[1] << 8) | flags_buf[0];
    uint16_t safety = (safety_buf[1] << 8) | safety_buf[0];
    rp2040_log("BQ27742-G1 interrupt: GPIO%d=%d Flags=0x%04x SafetyStatus=0x%04x\n",
               _gpio_interrupt,
               gpio_get(_gpio_interrupt),
               flags,
               safety);

    if (flags & SOC1_MASK){
        rp2040_log("BQ27742-G1 interrupt: SOC1 set\n");
    }
    if (flags & BATHI_MASK){
        rp2040_log("BQ27742-G1 interrupt: BATHI set\n");
    }
    if (flags & BATLOW_MASK){
        rp2040_log("BQ27742-G1 interrupt: BATLOW set\n");
    }
    if (flags & IMAX_MASK){
        rp2040_log("BQ27742-G1 interrupt: IMAX set, clearing latched interrupt\n");
        bq27742_g1_control(BQ27742_G1_CONTROL_IMAX_INT_CLEAR);
    }
    if (safety & ISD_MASK){
        rp2040_log("BQ27742-G1 interrupt: ISD set\n");
    }
    if (safety & TDD_MASK){
        rp2040_log("BQ27742-G1 interrupt: TDD set\n");
    }
    if (safety & OTC_MASK){
        rp2040_log("BQ27742-G1 interrupt: OTC set\n");
    }
    if (safety & OTD_MASK){
        rp2040_log("BQ27742-G1 interrupt: OTD set\n");
    }
    if (safety & OVP_MASK){
        rp2040_log("BQ27742-G1 interrupt: OVP set\n");
    }
    if (safety & UVP_MASK){
        rp2040_log("BQ27742-G1 interrupt: UVP set\n");
    }
    if ((flags & (SOC1_MASK | BATHI_MASK | BATLOW_MASK | IMAX_MASK)) == 0 &&
        (safety & (ISD_MASK | TDD_MASK | OTC_MASK | OTD_MASK | OVP_MASK | UVP_MASK)) == 0){
        rp2040_log("BQ27742-G1 interrupt: no enabled status bits currently set\n");
    }

    if (gpio_get(_gpio_interrupt)){
        bq27742_g1_rearm_assert_irq();
    }
    return flags;
}

static void bq27742_g1_rearm_assert_irq(){
    bq27742_g1_interrupt_pending = false;
    gpio_set_irq_enabled(_gpio_interrupt, bq27742_g1_deassert_irq_mask, false);
    gpio_set_irq_enabled(_gpio_interrupt, bq27742_g1_irq_mask, true);
}

static void bq27742_g1_configure_host_interrupts(){
    uint16_t pack_config = bq27742_g1_get_pack_configuration();
    rp2040_log("BQ27742-G1 PackConfiguration before RC2 init: 0x%04x\n", pack_config);

    uint8_t block_data[BQ27742_G1_DF_BLOCK_LEN];
    if (!bq27742_g1_read_data_flash_block(BQ27742_G1_DF_SUBCLASS_REGISTERS,
                                          BQ27742_G1_DF_BLOCK_REGISTERS,
                                          block_data)){
        rp2040_log("ERROR: BQ27742-G1 could not read data flash for RC2 init\n");
        return;
    }

    uint8_t pack_config_high_index = BQ27742_G1_DF_PACK_CONFIG_OFFSET;
    uint8_t pack_config_low_index = BQ27742_G1_DF_PACK_CONFIG_OFFSET + 1;
    uint16_t data_flash_pack_config_be =
        (block_data[BQ27742_G1_DF_PACK_CONFIG_OFFSET] << 8) |
        block_data[BQ27742_G1_DF_PACK_CONFIG_OFFSET + 1];
    uint16_t data_flash_pack_config_le =
        (block_data[BQ27742_G1_DF_PACK_CONFIG_OFFSET + 1] << 8) |
        block_data[BQ27742_G1_DF_PACK_CONFIG_OFFSET];
    if (data_flash_pack_config_le == pack_config && data_flash_pack_config_be != pack_config){
        pack_config_low_index = BQ27742_G1_DF_PACK_CONFIG_OFFSET;
        pack_config_high_index = BQ27742_G1_DF_PACK_CONFIG_OFFSET + 1;
    } else if (data_flash_pack_config_be != pack_config){
        rp2040_log("BQ27742-G1 PackConfiguration byte order not verified by command read; using TRM order\n");
    }

    uint16_t old_pack_config =
        (block_data[pack_config_high_index] << 8) | block_data[pack_config_low_index];
    uint16_t new_pack_config = old_pack_config;
    new_pack_config |= BQ27742_G1_PACK_CONFIG_HOSTIE_MASK;
    new_pack_config &= ~BQ27742_G1_PACK_CONFIG_HOSTIPU_MASK;
    new_pack_config &= ~BQ27742_G1_PACK_CONFIG_HOSTIPOL_MASK;

    uint8_t old_pack_config_c = block_data[BQ27742_G1_DF_PACK_CONFIG_C_OFFSET];
    uint8_t new_pack_config_c =
        old_pack_config_c & ~BQ27742_G1_PACK_CONFIG_C_BTP_EN_MASK;

    if (old_pack_config == new_pack_config && old_pack_config_c == new_pack_config_c){
        rp2040_log("BQ27742-G1 RC2 host interrupts already configured\n");
        return;
    }

    block_data[pack_config_high_index] = (new_pack_config >> 8) & 0xFF;
    block_data[pack_config_low_index] = new_pack_config & 0xFF;
    block_data[BQ27742_G1_DF_PACK_CONFIG_C_OFFSET] = new_pack_config_c;

    bq27742_g1_write_data_flash_block(BQ27742_G1_DF_SUBCLASS_REGISTERS,
                                      BQ27742_G1_DF_BLOCK_REGISTERS,
                                      block_data);
    sleep_ms(10);

    uint8_t verify_block[BQ27742_G1_DF_BLOCK_LEN];
    if (!bq27742_g1_read_data_flash_block(BQ27742_G1_DF_SUBCLASS_REGISTERS,
                                          BQ27742_G1_DF_BLOCK_REGISTERS,
                                          verify_block)){
        rp2040_log("ERROR: BQ27742-G1 could not verify RC2 data flash write\n");
        return;
    }

    uint16_t verify_pack_config =
        (verify_block[pack_config_high_index] << 8) | verify_block[pack_config_low_index];
    uint8_t verify_pack_config_c = verify_block[BQ27742_G1_DF_PACK_CONFIG_C_OFFSET];
    if (verify_pack_config != new_pack_config || verify_pack_config_c != new_pack_config_c){
        rp2040_log("ERROR: BQ27742-G1 RC2 data flash write did not verify. PackConfig=0x%04x expected=0x%04x PackConfigC=0x%02x expected=0x%02x\n",
                   verify_pack_config,
                   new_pack_config,
                   verify_pack_config_c,
                   new_pack_config_c);
        return;
    }

    rp2040_log("BQ27742-G1 RC2 host interrupts configured. PackConfiguration 0x%04x -> 0x%04x, PackConfigC 0x%02x -> 0x%02x\n",
               old_pack_config,
               new_pack_config,
               old_pack_config_c,
               new_pack_config_c);
}

static uint16_t bq27742_g1_get_pack_configuration(){
    uint8_t buf[2];
    bq27742_g1_read_bytes(BQ27742_G1_REG_PACK_CONFIGURATION, buf, 2);
    return (buf[1] << 8) | buf[0];
}

static bool bq27742_g1_read_data_flash_block(uint8_t subclass, uint8_t block, uint8_t *block_data){
    bq27742_g1_write_byte(BQ27742_G1_REG_BLOCK_DATA_CONTROL, 0x00);
    bq27742_g1_write_byte(BQ27742_G1_REG_DATA_FLASH_CLASS, subclass);
    bq27742_g1_write_byte(BQ27742_G1_REG_DATA_FLASH_BLOCK, block);
    sleep_ms(1);
    bq27742_g1_read_bytes(BQ27742_G1_REG_BLOCK_DATA, block_data, BQ27742_G1_DF_BLOCK_LEN);

    uint8_t checksum = 0;
    bq27742_g1_read_bytes(BQ27742_G1_REG_BLOCK_DATA_CHECKSUM, &checksum, 1);
    uint8_t calculated_checksum = bq27742_g1_data_flash_checksum(block_data);
    if (checksum != calculated_checksum){
        rp2040_log("ERROR: BQ27742-G1 data flash checksum mismatch read=0x%02x calculated=0x%02x\n",
                   checksum,
                   calculated_checksum);
        return false;
    }
    return true;
}

static void bq27742_g1_write_data_flash_block(uint8_t subclass, uint8_t block, const uint8_t *block_data){
    uint8_t write_buf[BQ27742_G1_DF_BLOCK_LEN + 1];
    write_buf[0] = BQ27742_G1_REG_BLOCK_DATA;
    memcpy(&write_buf[1], block_data, BQ27742_G1_DF_BLOCK_LEN);

    bq27742_g1_write_byte(BQ27742_G1_REG_BLOCK_DATA_CONTROL, 0x00);
    bq27742_g1_write_byte(BQ27742_G1_REG_DATA_FLASH_CLASS, subclass);
    bq27742_g1_write_byte(BQ27742_G1_REG_DATA_FLASH_BLOCK, block);
    sleep_ms(1);
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, write_buf, sizeof write_buf, false);
    bq27742_g1_write_byte(BQ27742_G1_REG_BLOCK_DATA_CHECKSUM,
                          bq27742_g1_data_flash_checksum(block_data));
}

static uint8_t bq27742_g1_data_flash_checksum(const uint8_t *block_data){
    uint8_t sum = 0;
    for (uint8_t i = 0; i < BQ27742_G1_DF_BLOCK_LEN; i++){
        sum += block_data[i];
    }
    return 0xFF - sum;
}

static void bq27742_g1_write_byte(uint8_t command, uint8_t value){
    uint8_t buf[2] = {command, value};
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, buf, sizeof buf, false);
}

static void bq27742_g1_read_bytes(uint8_t command, uint8_t *buf, size_t len){
    i2c_write_error_handling(i2c0, BQ27742_G1_ADDR, &command, 1, true);
    i2c_read_error_handling(i2c0, BQ27742_G1_ADDR, buf, len, false);
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
