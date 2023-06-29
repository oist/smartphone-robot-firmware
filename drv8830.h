#ifndef DRV8830_
#define DRV8830_

// DRV8830 I2C addresses
#define MOTOR_LEFT_ADDRESS_W 0xC0
#define MOTOR_RIGHT_ADDRESS_W 0xC4
#define MOTOR_LEFT_ADDRESS_R 0xC1
#define MOTOR_RIGHT_ADDRESS_R 0xC5
#define MOTOR_LEFT_ADDRESS 0x60
#define MOTOR_RIGHT_ADDRESS 0x62

// Register address
#define DRV8830_REG_CONTROL 0x0
#define DRV8830_REG_FAULT 0x1

#define DRV8830_IN1_BIT 0
#define DRV8830_IN2_BIT 1

// Motor enum
typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor;

void drv8830_init();
void set_voltage(Motor motor, float voltage);

#endif
