#ifndef MAX77857_
#define MAX77857_

// -----------------------------------------------------------------------------
// Device Slave Addresses
// -----------------------------------------------------------------------------
#define MAX77857_ADDR _u(0x66)       // device has 7-bit address of 0x66
#define MAX77857_ADDR_WRITE _u(0xCC) // Write address (8-bit)
#define MAX77857_ADDR_READ _u(0xCD)  // Read address (8-bit)
// -----------------------------------------------------------------------------
// Registers
// -----------------------------------------------------------------------------
#define MAX77857_REG_REG_CONT1_ADDR _u(0x12)
#define MAX77857_REG_REG_CONT1_RESET _u(0x50) // 0b01010000
#define MAX77857_REG_REG_CONT1_COMP_LSB _u(5)
#define MAX77857_REG_REG_CONT1_COMP_MSB _u(7)
#define MAX77857_REG_REG_CONT1_COMP_DEFAULT _u(0x2) // Currently the reset value
#define MAX77857_REG_REG_CONT1_FREQ_LSB _u(3)
#define MAX77857_REG_REG_CONT1_FREQ_MSB _u(3)
#define MAX77857_REG_REG_CONT1_FREQ_DEFAULT _u(0x2) // Currently the reset value
#define MAX77857_REG_REG_CONT1_ILM_LSB _u(0)
#define MAX77857_REG_REG_CONT1_ILM_MSB _u(2)
#define MAX77857_REG_REG_CONT1_ILM_DEFAULT _u(0x5)

#define MAX77857_REG_REG_CONT2_ADDR _u(0x13)
#define MAX77857_REG_REG_CONT2_RESET _u(0x44)
#define MAX77857_REG_REG_CONT2_VREF_LSB _u(0)
#define MAX77857_REG_REG_CONT2_VREF_MSB _u(7)
#define MAX77857_REG_REG_CONT2_VREF_DEFAULT _u(0x44) // Vout=Vref*15. Desired Vout=5V, thus Vref=0.333V, with default value of 0x44

#define MAX77857_REG_REG_CONT3_ADDR _u(0x14)
#define MAX77857_REG_REG_CONT3_RESET _u(0xC) // 0b00001100
#define MAX77857_REG_REG_CONT3_FPWM_LSB _u(5)
#define MAX77857_REG_REG_CONT3_FPWM_MSB _u(5)
#define MAX77857_REG_REG_CONT3_FPWM_DEFAULT _u(0x0) 

void max77857_init();


#endif
