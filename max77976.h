#ifndef MAX77976_ /* Include guard */
#define MAX77976_

// -----------------------------------------------------------------------------
// Device Slave Addresses
// -----------------------------------------------------------------------------
#include "pico/types.h"
#include "pico/util/queue.h"
#define MAX77976_ADDR _u(0x6B)       // device has 7-bit address of 0x6B
#define MAX77976_ADDR_WRITE _u(0xD6) // Write address (8-bit)
#define MAX77976_ADDR_READ _u(0xD7)  // Read address (8-bit)
// -----------------------------------------------------------------------------
// Registers
// -----------------------------------------------------------------------------

#define MAX77976_REG_CHG_INT _u(0x10)
#define MAX77976_REG_CHG_INT_MASK _u(0x11)
#define MAX77976_REG_CHG_INT_OK _u(0x12)

#define MAX77976_REG_CHG_CNFG_00_ADDR _u(0x16)
#define MAX77976_REG_CHG_CNFG_00_SPR_7_4_LSB _u(4)
#define MAX77976_REG_CHG_CNFG_00_SPR_7_4_MSB _u(7)
#define MAX77976_REG_CHG_CNFG_00_SPR_7_4_RESET _u(0x0)
#define MAX77976_REG_CHG_CNFG_00_MODE_LSB _u(0)
#define MAX77976_REG_CHG_CNFG_00_MODE_MSB _u(3)
#define MAX77976_REG_CHG_CNFG_00_MODE_RESET _u(0X4)
#define MAX77976_REG_CHG_CNFG_00_MODE_CHARGE_BUCK _u(0x5)
#define MAX77976_REG_CHG_CNFG_00_MODE_BATTERY_BOOST_FLASH _u(0x9)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_02_ADDR _u(0x18)
#define MAX77976_REG_CHG_CNFG_02_SPR_7_LSB _u(7)
#define MAX77976_REG_CHG_CNFG_02_SPR_7_MSB _u(7)
#define MAX77976_REG_CHG_CNFG_02_CHG_CC_LSB _u(0)
#define MAX77976_REG_CHG_CNFG_02_CHG_CC_MSB _u(6)
#define MAX77976_REG_CHG_CNFG_02_CHG_CC_RESET _u(0x09)
#define MAX77976_REG_CHG_CNFG_02_CHG_CC_1250 _u(0x19)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_03_ADDR _u(0x19)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_04_ADDR _u(0x1A)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_06_ADDR _u(0x1C)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_08_ADDR _u(0x1E)
#define MAX77976_REG_CHG_CNFG_08_RESET (0x2)
#define MAX77976_REG_CHG_CNFG_08_RSVD_7_LSB _u(7)
#define MAX77976_REG_CHG_CNFG_08_RSVD_7_MSB _u(7)
#define MAX77976_REG_CHG_CNFG_08_RSVD_7_RESET _u(0)
#define MAX77976_REG_CHG_CNFG_08_SPR_6_5_LSB _u(5)
#define MAX77976_REG_CHG_CNFG_08_SPR_6_5_MSB _u(6)
#define MAX77976_REG_CHG_CNFG_08_SPR_6_5_RESET _u(0)
#define MAX77976_REG_CHG_CNFG_08_FMBST_LSB _u(4)
#define MAX77976_REG_CHG_CNFG_08_FMBST_MSB _u(4)
#define MAX77976_REG_CHG_CNFG_08_FMBST_RESET _u(0)
#define MAX77976_REG_CHG_CNFG_08_SPR_3_LSB _u(3)
#define MAX77976_REG_CHG_CNFG_08_SPR_3_MSB _u(3)
#define MAX77976_REG_CHG_CNFG_08_SPR_3_RESET _u(0)
#define MAX77976_REG_CHG_CNFG_08_SLOWLX_LSB _u(2)
#define MAX77976_REG_CHG_CNFG_08_SLOWLX_MSB _u(2)
#define MAX77976_REG_CHG_CNFG_08_SLOWLX_RESET _u(0)
#define MAX77976_REG_CHG_CNFG_08_FSW_LSB _u(1)
#define MAX77976_REG_CHG_CNFG_08_FSW_MSB _u(1)
#define MAX77976_REG_CHG_CNFG_08_FSW_RESET _u(1)
#define MAX77976_REG_CHG_CNFG_08_FSW_2P6 _u(0)
#define MAX77976_REG_CHG_CNFG_08_DISKIP_LSB _u(0)
#define MAX77976_REG_CHG_CNFG_08_DISKIP_MSB _u(0)
#define MAX77976_REG_CHG_CNFG_08_DISKIP_RESET _u(0)
// -----------------------------------------------------------------------------
#define MAX77976_REG_CHG_CNFG_09_ADDR _u(0x1F)
#define MAX77976_REG_CHG_CNFG_09_RESET _u(0x89)
#define MAX77976_REG_CHG_CNFG_09_INLIM_CLK_LSB _u(6)
#define MAX77976_REG_CHG_CNFG_09_INLIM_CLK_MSB _u(7)
#define MAX77976_REG_CHG_CNFG_09_INLIM_CLK_RESET _u(0x2) // 0b10: 1024
#define MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_LSB _u(0)
#define MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_MSB _u(5)
#define MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_RESET _u(0x9)
#define MAX77976_REG_CHG_CNFG_09_CHGIN_ILIM_3000 _u(0x3B)
// -----------------------------------------------------------------------------
// Variable defines
#define MAX77976_INT_BUF_LEN _u(3)


int max77976_init(uint gpio_interrupt, queue_t* cq, queue_t* rq);
void max77976_get_chg_details();
void max77976_toggle_led();
void max77976_log_current_limit();
void max77976_shutdown();
void max77976_on_battery_charger_interrupt(uint GPIO, uint32_t events);
void test_max77976_get_id();
void test_max77976_get_FSW();
void test_max77976_interrupt();

#endif // MAX77976_
