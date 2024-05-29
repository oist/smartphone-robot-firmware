#ifndef DRV8830_
#define DRV8830_

// DRV8830 I2C addresses
#define MOTOR_LEFT_ADDRESS_W _u(0xC0)
#define MOTOR_RIGHT_ADDRESS_W _u(0xC4)
#define MOTOR_LEFT_ADDRESS_R _u(0xC1)
#define MOTOR_RIGHT_ADDRESS_R _u(0xC5)
#define MOTOR_LEFT_ADDRESS _u(0x60)
#define MOTOR_RIGHT_ADDRESS _u(0x62)

// Register address
#define DRV8830_REG_CONTROL _u(0x0)
#define DRV8830_REG_FAULT _u(0x1)

#define DRV8830_IN1_BIT 0
#define DRV8830_IN2_BIT 1

#define DRV8830_BUF_LEN _u(2)

#include "pico/types.h"

// Motor enum
typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

void drv8830_init(uint gpio_fault1, uint gpio_fault2) ;
void drv8830_on_interrupt(uint gpio, uint32_t event_mask);
void set_motor_control(Motor motor, uint8_t control_value);
void set_voltage(Motor motor, float voltage);
void test_drv8830_get_faults();
void test_drv8830_interrupt();
uint8_t* drv8830_get_faults();

#endif
