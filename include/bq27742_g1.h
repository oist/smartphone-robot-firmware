#ifndef BQ27742_G1_
#define BQ27742_G1_

// -----------------------------------------------------------------------------
// Device Slave Addresses
// -----------------------------------------------------------------------------
#define BQ27742_G1_ADDR _u(0x55)       // device has 7-bit address of 0b1010101
#define BQ27742_G1_ADDR_WRITE _u(0xAA) // Write address (8-bit)
#define BQ27742_G1_ADDR_READ _u(0xAB)  // Read address (8-bit)
// -----------------------------------------------------------------------------
// Registers
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Masks
// -----------------------------------------------------------------------------
// ---Safety Status---
#define ISD_MASK 1<<5
#define TDD_MASK 1<<4
#define OTC_MASK 1<<3
#define OTD_MASK 1<<2
#define OVP_MASK 1<<1
#define UVP_MASK 1

// ---Flags---
#define BATHI_MASK 1<<(5+8)
#define BATLOW_MASK 1<<(4+8)
#define CHG_INH_MASK 1<<(3+8)
#define FC_MASK 1<<(1+8)
#define CHG_SUS_MASK 1<<7
#define IMAX_MASK 1<<4
#define CHG_MASK 1<<3
#define SOC1_MASK 1<<2
#define SOCF_MASK 1<<1
#define DSG_MASK 1


void bq27742_g1_init();
void bq27742_g1_get_voltage();
void bq27742_g1_get_safety_stats();
void bq27742_g1_get_temp();
void bq27742_g1_get_soh();
void bq27742_g1_get_flags();
void bq27742_g1_shutdown();
void bq27742_g1_fw_version_check();

#endif
