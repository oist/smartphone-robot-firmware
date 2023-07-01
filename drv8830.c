#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "robot.h"
#include "drv8830.h"
#include <string.h>

static i2c_inst_t *i2c = i2c0;
static uint8_t send_buf[2] = {0};
static uint8_t return_buf[2] = {0}; // Will read full buffer from registers 0x52 to 0x71

void drv8830_init() {
    // read the fault register from both motors and assert false if no response
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = DRV8830_REG_FAULT;
    i2c_write_error_handling(i2c, MOTOR_LEFT_ADDRESS, send_buf, 1, true);
    i2c_read_error_handling(i2c, MOTOR_LEFT_ADDRESS, return_buf, 2, false);
    printf("Left motor fault register: %d\n", return_buf[0]);

    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = DRV8830_REG_FAULT;
    i2c_write_error_handling(i2c, MOTOR_RIGHT_ADDRESS, send_buf, 1, true);
    i2c_read_error_handling(i2c, MOTOR_RIGHT_ADDRESS, return_buf, 2, false);
    printf("Right motor fault register: %d\n", return_buf[0]);

    set_voltage(MOTOR_LEFT, 0);
    set_voltage(MOTOR_RIGHT, 0);
}

/*
 * Set the voltage of the specified motor.
 *
 * @param motor The motor to set the voltage of.
 * @param voltage The voltage to set the motor to. This should be between -5.06V and 5.06V.
 */
void set_voltage(Motor motor, float voltage) {
    // Exclude or truncate voltages between -0.48V and 0.48V to 0V
    if (voltage >= -0.48 && voltage <= 0.48) {
        voltage = 0.0;
    }

    // Clamp the voltage within the valid range
    if (voltage < -5.06) {
        voltage = -5.06;
    } else if (voltage > 5.06) {
        voltage = 5.06;
    }

    // Take the absolute value of voltage for control calculations
    float abs_voltage = fabs(voltage);

    // Convert voltage to control value (-0x3F to 0x3F)
    int16_t control_value = (int16_t)(((64 * abs_voltage) / (4 * 1.285)) - 1);
    // voltage is defined by bits 2-7. Shift the control value to the correct position
    control_value = control_value << 2;

    // Set the IN1 and IN2 bits based on the sign of the voltage
    if (voltage < 0) {
        control_value |= (1 << DRV8830_IN1_BIT);
        control_value &= ~(1 << DRV8830_IN2_BIT);
    } else if (voltage > 0) {
        control_value |= (1 << DRV8830_IN2_BIT);
        control_value &= ~(1 << DRV8830_IN1_BIT);
    } else {
        // Standby/Coast: Both IN1 and IN2 set to 0
        control_value = 0;
    }

    // Determine the appropriate I2C address based on the motor
    uint8_t i2c_address;
    if (motor == MOTOR_LEFT) {
        i2c_address = MOTOR_LEFT_ADDRESS;
    } else {
        i2c_address = MOTOR_RIGHT_ADDRESS;
    }

    uint8_t buffer[] = { DRV8830_REG_CONTROL, (uint8_t)control_value };
    i2c_write_blocking(i2c, i2c_address, buffer, sizeof(buffer), false);
}

