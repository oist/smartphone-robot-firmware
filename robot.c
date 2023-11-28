#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/types.h"
#include "robot.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "max77976.h"
#include "max77857.h"
#include "STWLC38JRM.h"
#include "ncp3901.h"
#include "sn74ahc125rgyr.h"
#include "max77958.h"
#include "bq27742_g1.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include <assert.h>
#include "CException.h"
#include "quad_encoders.h"
#include "drv8830.h"
#include "hardware/uart.h"
#include <string.h>
#include "rp2040_log.h"
#include "serial_comm_manager.h"
#include "hardware/pwm.h"

static queue_t call_queue;
static queue_t results_queue;
static bool core1_shutdown_requested = false;
static RP2040_STATE rp2040_state;

void blink_led(uint8_t blinkCnt, int onTime, int offTime);
void i2c_start();
void i2c_stop();
void adc_init();
void adc_shutdown();
void init_queues();
void free_queues();
void on_start();
void on_shutdown();
void results_queue_pop();
int32_t call_queue_pop();
static void signal_stop_core1();
static void robot_interrupt_handler(uint gpio, uint32_t event_mask);
void robot_unit_tests();
void get_encoder_counts(RP2040_STATE* rp2040_state);
void get_motor_faults(RP2040_STATE* state);
void get_charger_state(RP2040_STATE* state);
void turn_on_leds();

volatile CEXCEPTION_T e;

// core1 will be used to process all function calls requested by interrupt calls on core0
void core1_entry() {
    while (core1_shutdown_requested == false) {
        // Function pointer is passed to us via the queue_entry_t which also
        // contains the function parameter.
        // We provide an int32_t return value by simply pushing it back on the
        // return queue which also indicates the result is ready.

//sleep_ms(1);
	int32_t result = call_queue_pop();
	//results_queue_try_add(&tight_loop_contents, result);
        // as an alternative to polling the return queue, you can send an irq to core0 to add another entry to call_queue
    }
}

int32_t call_queue_pop(){
    queue_entry_t entry;
    queue_remove_blocking(&call_queue, &entry);
    rp2040_log("call_queue entry removed. call_queue has %d entries remaining to handle\n", queue_get_level(&call_queue));
    int32_t result = entry.func(entry.data);
    return result;
}

int32_t stop_core1(){
    core1_shutdown_requested = true;
    return 0;
}

void results_queue_pop(){
    // If the results_queue is not empty, take the first entry and call its function on core0
    if (!queue_is_empty(&results_queue)){
        queue_entry_t entry;
        //queue_try_remove(&results_queue, &entry);
        queue_remove_blocking(&results_queue, &entry);
        // TODO implement what to do with results_queue entries.
        rp2040_log("results_queue has %d entries remaining to handle\n", queue_get_level(&results_queue));
    }
}

void get_state(RP2040_STATE* state){
    // ChargeSideUSB: Mux MAX77976 and NCP3901 Data: Charger-side USB Voltage, and Wireless Coil State
    //uint16_t acdcValue = ncp3901_adc0();
    //memcpy(&response[2], &acdcValue, sizeof(uint16_t));
    // BatteryDetails: BQ27742-G1 Data: Battery Voltage, Current, and State of Charge

    // PhoneSideUSB: MAX77958 Phone-side USB Controller
    
    // MotorDetails: DRV8830DRCR Data: Includes MOTOR_FAULT, ENCODER_COUNTS, MOTOR_LEVELS, MOTOR_BRAKE
    //TODO this should not update response with static int values like this
    get_encoder_counts(state);
    get_motor_faults(state);
    get_charger_state(state);
    get_battery_state(state);
}

// Takes the response and add the quad encoder counts to it
void get_encoder_counts(RP2040_STATE* state){
    
    uint32_t left, right;
    left = quad_encoder_get_count(MOTOR_LEFT);
    right = quad_encoder_get_count(MOTOR_RIGHT);
   
    state->MotorsState.EncoderCounts.left = left;
    state->MotorsState.EncoderCounts.right = right;
}

void get_motor_faults(RP2040_STATE* state){
    uint8_t* motor_faults = drv8830_get_faults();
    state->MotorsState.Faults.left = motor_faults[0]; 
    state->MotorsState.Faults.right = motor_faults[1]; 
}

void get_battery_state(RP2040_STATE* state){
    state->BatteryDetails.voltage = bq27742_g1_get_voltage();
    state->BatteryDetails.safety_status = bq27742_g1_get_safety_stats();
    state->BatteryDetails.temperature = bq27742_g1_get_temp();
    state->BatteryDetails.state_of_health = bq27742_g1_get_soh();
    state->BatteryDetails.flags = bq27742_g1_get_flags();
}

void get_charger_state(RP2040_STATE* state){
    state->ChargeSideUSB.max77976_chg_details = max77976_get_chg_details();
    // Note the implied conversion from bool to uint8_t for the purpose of sending over the serial port via byte array
    state->ChargeSideUSB.wireless_charger_attached = ncp3901_wireless_charger_attached();
    state->ChargeSideUSB.usb_charger_voltage = ncp3901_adc0();
}

void set_motor_levels(RP2040_STATE* state){
    set_motor_control(MOTOR_LEFT, state->MotorsState.ControlValues.left);
    set_motor_control(MOTOR_RIGHT, state->MotorsState.ControlValues.right);
}

int main(){
    bool shutdown = false;
    on_start();
    sleep_ms(1000);
    //int i = 0;
    while (true)
    {
	//results_queue_pop();
        //sample_adc_inputs();
	//get_battery_state();
	//max77976_get_chg_details();
	//quad_encoder_update();
	//sleep_ms(100);
	//quad_encoder_update();
	//max77976_log_current_limit();
	//max77976_toggle_led();
        get_block();
	if (shutdown){
	    on_shutdown();
	    break;
	}else{
	    // This sleep or some other time consuming function must occur else can't reset from gdb as thread will be stuck in tight_loop_contents()
            sleep_ms(10);
	    tight_loop_contents();
	}
//	i++;
    }
    on_shutdown();

    return 0;
}

void on_start(){
    rp2040_log_init();
    rp2040_log("on_start\n");
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);
    gpio_set_irq_callback(&robot_interrupt_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    init_queues();
    multicore_launch_core1(core1_entry);
    i2c_start();
    adc_init();
    turn_on_leds();
    STWLC38JRM_init(WIRELESS_CHG_EN);
    ncp3901_init(GPIO_WIRELESS_AVAILABLE, GPIO_OTG);
    max77976_init(BATTERY_CHARGER_INTERRUPT_PIN, &call_queue, &results_queue);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO1);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO2);
    max77958_init(MAX77958_INTB, &call_queue, &results_queue);
    sleep_ms(1000);
    rp2040_log("done waiting 2\n");
    bq27742_g1_init();
    bq27742_g1_fw_version_check();
    // Be sure to do this last
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO1);
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO2);
    drv8830_init(DRV8830_FAULT1, DRV8830_FAULT2);
    sleep_ms(1000);
    encoder_init(&call_queue);
    rp2040_log("encoders initialize. Waiting 1 second\n");
    sleep_ms(1000);
    rp2040_log("done waiting, Running unit tests.\n");
    set_voltage(MOTOR_LEFT, 2.5);
    set_voltage(MOTOR_RIGHT, 2.5);
    int i = 0;
    while (i < 500){
       quad_encoder_update();
       i++;
       tight_loop_contents();
    }
    rp2040_log("done counting, turning off motors\n");
    set_voltage(MOTOR_LEFT, 0);
    set_voltage(MOTOR_RIGHT, 0);
    robot_unit_tests();
    serial_comm_manager_init(&rp2040_state);
    rp2040_log("on_start complete\n");
    //while(!stdio_usb_connected()){
    //    sleep_ms(100);
    //}
    //rp2040_log("USB connected\n"); 
}

void robot_unit_tests(){
    rp2040_log("----------Running robot unit tests-----------\n");
    test_max77958_get_id();
    test_max77958_get_customer_config_id();
    test_max77958_interrupt();
    test_max77976_get_id();
    test_max77976_get_FSW();
    test_max77976_interrupt();
    test_ncp3901_interrupt();
    test_drv8830_get_faults();
    test_drv8830_interrupt();
    rp2040_log("-----------robot unit tests complete-----------\n");
}

void on_shutdown(){
    rp2040_log("Shutting down\n");
    max77958_shutdown(MAX77958_INTB);
    //sn74ahc125rgyr_shutdown(SN74AHC125RGYR_GPIO);
    //max77976_shutdown();
    //ncp3901_shutdown();
    //wrm483265_10f5_12v_g_shutdown(WIRELESS_CHG_EN);
    adc_shutdown();
    // Note this will shut off the battery to the rp2040 so unless you're plugged in, everything will fail here.
    // TODO how do I wake from this if the rp2040 has no power to respond??

    signal_stop_core1();
    free_queues();
    bq27742_g1_shutdown();
    //i2c_stop();
}

void init_queues(){
    queue_init(&call_queue, sizeof(queue_entry_t), 256);
    queue_init(&results_queue, sizeof(int32_t), 256);
}

void free_queues(){
    while (!queue_is_empty(&results_queue)){
	results_queue_pop();
    }
    while (!queue_is_empty(&call_queue)){
    	rp2040_log("free_queues: call_queue not empty\n");
    	sleep_ms(500);
    }
    queue_free(&call_queue);
    queue_free(&results_queue);
}

static void signal_stop_core1(){
    queue_entry_t stop_entry = {stop_core1, 0};
    if(!queue_try_add(&call_queue, &stop_entry)){
	rp2040_log("call_queue is full");
        assert(false);
    }
}

void i2c_start(){
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA0);
    gpio_pull_up(I2C_SCL0);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA1);
    gpio_pull_up(I2C_SCL1);
}

void i2c_stop(){
    gpio_pull_down(I2C_SDA0);
    gpio_pull_down(I2C_SCL0);
    gpio_pull_down(I2C_SDA1);
    gpio_pull_down(I2C_SCL1);
    gpio_set_function(I2C_SDA0, GPIO_FUNC_SIO);
    gpio_set_function(I2C_SCL0, GPIO_FUNC_SIO);
    gpio_set_function(I2C_SDA1, GPIO_FUNC_SIO);
    gpio_set_function(I2C_SCL1, GPIO_FUNC_SIO);
    i2c_deinit(i2c0);
    i2c_deinit(i2c1);
}

void adc_shutdown(){
}

void turn_on_leds(){
    // Set GPIO to PWM with frequency 1kHz and 50% duty cycle
    gpio_set_function(LED_EN_PIN, GPIO_FUNC_PWM);
    // level between 0 and 255 inclusive
    pwm_set_gpio_level(LED_EN_PIN, 128);
}

//---------------------------------------------------------------------
// Initialization Methods
//---------------------------------------------------------------------

void max77642_init() {}

void quad_encoders_init() {}

//---------------------------------------------------------------------
// Run Loop Methods
//---------------------------------------------------------------------
void sample_adc_inputs(){
    ncp3901_adc0();
}

void drv8830drcr_set_moto_lvl(){
}

//---------------------------------------------------------------------
// Interrupt Callbacks
//---------------------------------------------------------------------

static void robot_interrupt_handler(uint gpio, uint32_t event_mask){
    switch (gpio){
	case GPIO_WIRELESS_AVAILABLE:
	    ncp3901_on_wireless_charger_interrupt(gpio, event_mask);
	    break;
	case BATTERY_CHARGER_INTERRUPT_PIN:
	    max77976_on_battery_charger_interrupt(gpio, event_mask);
	    break;
	case MAX77958_INTB:
	    max77958_on_interrupt(gpio, event_mask);
	    break;
	case DRV8830_FAULT1:
	    drv8830_on_interrupt(gpio, event_mask);
	    break;
	case DRV8830_FAULT2:
	    drv8830_on_interrupt(gpio, event_mask);
	    break;
    }
}

void results_queue_try_add(void *func, int32_t arg){
    queue_entry_t entry = {func, arg};
    //rp2040_log("call_queue currently has %i entries\n", queue_get_level(&call_queue));
    if(!queue_try_add(&results_queue, &entry)){
        rp2040_log("results_queue is full");
    	assert(false);
    }
}

void call_queue_try_add(entry_func func, int32_t arg){
    queue_entry_t entry = {func, arg};
    //rp2040_log("call_queue currently has %i entries\n", queue_get_level(&call_queue));
    if(!queue_try_add(&call_queue, &entry)){
        rp2040_log("call_queue is full");
    	assert(false);
    }
}

void quad_encoders_callback(){
    // GPIO12-15 monitor past and current states to determine counts
}

void i2c_write_error_handling(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop){
    int result;
    Try{
	    result = i2c_write_timeout_us(i2c, addr, src, len, nostop, I2C_TIMEOUT);
	    if (result < 0){
		    Throw(result);
	    }
	} 
    Catch(e){
	    rp2040_log("Error during i2c_write. Returned value of %i %i \n", result, e);
	    assert(false);
	}
}

void i2c_read_error_handling(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop){
    int result;
    Try{
	    result = i2c_read_timeout_us(i2c, addr, dst, len, nostop, I2C_TIMEOUT);
	    if (result < 0){
		    Throw(result);
	    }
	} 
    Catch(e){
	    rp2040_log("Error during i2c_read. Returned value of %i %i \n", result, e);
	    assert(false);
	}
}
