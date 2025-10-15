#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "robot.h"
#include "drv8830.h"
#include <string.h>
#include <inttypes.h>
#include "custom_printf.h"

static i2c_inst_t *i2c = i2c0;
static uint8_t send_buf[2] = {0};
static uint8_t return_buf[2] = {0}; // Will read full buffer from registers 0x52 to 0x71
static uint8_t fault_values[2] = {0};
				    
int32_t drv8830_fault_handler(int32_t gpio);
static int32_t drv8830_test_response();
void test_drv8830_interrupt();
static int8_t _gpio_fault1;
static int8_t _gpio_fault2;
static bool test_drv8830_started = false;
static bool test_drv8830_completed = false;
const uint32_t drv8830_irq_mask = GPIO_IRQ_EDGE_FALL;
static void drv8830_clear_faults();

void drv8830_on_interrupt(uint gpio, uint32_t event_mask){
    //rp2040_log("DRV8830 interrupt\n");
    if (event_mask & drv8830_irq_mask){
        gpio_acknowledge_irq(gpio, drv8830_irq_mask);
        //TODO remember to retrieve this gpio from the fault handler to interpret which motor has the fault
        call_queue_try_add(&drv8830_fault_handler, gpio);
        if (test_drv8830_started){
            call_queue_try_add(&drv8830_test_response, 1);
        }
    }
}

int32_t drv8830_fault_handler(int32_t gpio){
    uint8_t fault_values = 0; 
    uint8_t addr = 0;
    uint8_t reg = DRV8830_REG_FAULT;
    char *motor;
    if (gpio == _gpio_fault1){
	rp2040_log("Left motor fault\n");
	addr = MOTOR_LEFT_ADDRESS;
	motor = "Left";
    }else if (gpio == _gpio_fault2){
	rp2040_log("Right motor fault\n");
	addr = MOTOR_RIGHT_ADDRESS;
	motor = "Right";
    }

    if (addr == 0){
	rp2040_log("Error: invalid motor fault\n");
	return -1;
    }else{
        i2c_write_error_handling(i2c, addr, &reg, 1, true);
        i2c_read_error_handling(i2c, addr, &fault_values, 1, false);
        rp2040_log("Fault values for Motor %s: 0x%x\n", motor, fault_values);
	return 0;
    }
}

void drv8830_init(uint gpio_fault1, uint gpio_fault2) {
    rp2040_log("DRV8830 init\n");
    _gpio_fault1 = gpio_fault1;
    _gpio_fault2 = gpio_fault2;
    gpio_init(gpio_fault1);
    gpio_init(gpio_fault2);
    // Ensure the output value is low so when turning it on later, it doesn't start high and damage something. 
    gpio_put(gpio_fault1, 0);
    gpio_put(gpio_fault2, 0);
    gpio_set_dir(gpio_fault1, GPIO_IN);
    gpio_set_dir(gpio_fault2, GPIO_IN);
    gpio_pull_up(gpio_fault1);
    gpio_pull_up(gpio_fault2);
    gpio_set_irq_enabled(_gpio_fault1, drv8830_irq_mask, true);
    gpio_set_irq_enabled(_gpio_fault2, drv8830_irq_mask, true);

    set_voltage(MOTOR_LEFT, 0);
    set_voltage(MOTOR_RIGHT, 0);
    rp2040_log("DRV8830 init complete\n");
}

/*
 * Set the voltage of the specified motor.
 *
 * @param motor The motor to set the voltage of.
 * @param voltage The voltage to set the motor to. This should be between -5.06V and 5.06V.
 */
void set_motor_control(Motor motor, uint8_t control_value) {

    // Determine the appropriate I2C address based on the motor
    uint8_t i2c_address;
    if (motor == MOTOR_LEFT) {
        i2c_address = MOTOR_LEFT_ADDRESS;
    } else {
        i2c_address = MOTOR_RIGHT_ADDRESS;
    }

    uint8_t buffer[] = { DRV8830_REG_CONTROL, control_value};
    i2c_write_blocking(i2c, i2c_address, buffer, sizeof(buffer), false);
}

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

uint8_t* drv8830_get_faults(){
    uint8_t addr[2] = {MOTOR_LEFT_ADDRESS, MOTOR_RIGHT_ADDRESS};
    uint8_t buf[2] = {0};
    buf[0] = DRV8830_REG_FAULT;
    for (int i = 0; i < 2; i++){
        // Set the fault register for reading. 
        i2c_write_error_handling(i2c, addr[i], buf, 1, true);
        // Read the fault register
        i2c_read_error_handling(i2c, addr[i], &fault_values[i], 1, false);
    }
    drv8830_clear_faults();
    return fault_values;
}


static void drv8830_clear_faults(){
    uint8_t addr[2] = {MOTOR_LEFT_ADDRESS, MOTOR_RIGHT_ADDRESS};
    uint8_t buf[2] = {0};
    buf[0] = DRV8830_REG_FAULT;
    // Setting this to one ensures the assert function works only if the read value is in fact 0
    uint8_t fault_value = 1;
    for (int i = 0; i < 2; i++){
        // Clear the faults
        buf[1] = (1 << 7); // clear bit on D7
        i2c_write_error_handling(i2c, addr[i], buf, 2, false);
        // Set the fault register for reading. 
        i2c_write_error_handling(i2c, addr[i], buf, 1, true);
        // Read the fault register
        i2c_read_error_handling(i2c, addr[i], &fault_value, 1, false);
	// If the first bit is not 0, then the fault has not been cleared.
        if ((fault_value & 1) != 0){
            rp2040_log("ERROR: Motor %d cannot clear faults. Exiting.\n", i);
            assert(false);
        }
    }
}

void test_drv8830_get_faults(){
    rp2040_log("test_drv8830_get_faults starting...\n");
    // This already does what a test would otherwise do. 
    drv8830_clear_faults();
    rp2040_log("test_drv8830_get_faults: PASSED.\n");
}

void test_drv8830_interrupt(){
    rp2040_log("test_drv8830_interrupt starting...\n");
    test_drv8830_started = true;
    rp2040_log("test_drv8830_interrupt: prior to driving low GPIO%d. Current Value:%d\n", _gpio_fault1, gpio_get(_gpio_fault1));
    gpio_set_dir(_gpio_fault1, GPIO_OUT);
    if (gpio_get(_gpio_fault1) != 0){
	rp2040_log("ERROR: test_drv8830_interrupt: GPIO%d was not driven low. Current Value:%d\n", _gpio_fault1, gpio_get(_gpio_fault1));
	assert(false);
    }
    rp2040_log("test_drv8830_interrupt: after driving low GPIO%d. Current Value:%d\n", _gpio_fault1, gpio_get(_gpio_fault1));
    uint32_t i = 0;
    while (!test_drv8830_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    rp2040_log("ERROR: test_drv8830_interrupt timed out\n");
	    assert(false);
	}
    }
    gpio_set_dir(_gpio_fault1, GPIO_IN);
    gpio_pull_up(_gpio_fault1);
    test_drv8830_started = false;
    test_drv8830_completed = false;
    rp2040_log("test_drv8830_interrupt: Encoder 1 PASSED after %" PRIu32 " milliseconds.\n", i*10);
    test_drv8830_started = true;
    gpio_set_dir(_gpio_fault2, GPIO_OUT);
    while (!test_drv8830_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    rp2040_log("ERROR: test_drv8830_interrupt timed out\n");
	    assert(false);
	}
    }
    gpio_set_dir(_gpio_fault2, GPIO_IN);
    gpio_pull_up(_gpio_fault2);
    test_drv8830_started = false;
    rp2040_log("test_drv8830_interrupt: Encoder 2 PASSED after %" PRIu32 " milliseconds.\n", i*10);
}

static int32_t drv8830_test_response(){
    test_drv8830_completed = true;
    return 0;
}

