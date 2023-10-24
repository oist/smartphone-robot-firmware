#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/types.h"
#include "robot.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "max77976.h"
#include "max77857.h"
#include "wrm483265_10f5_12v_g.h"
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

static queue_t call_queue;
static queue_t results_queue;
static bool core1_shutdown_requested = false;

void bq27742_g1_poll();
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
void handle_packet(IncomingPacketFromAndroid *packet);
void send_block(uint8_t *buffer, uint8_t buffer_length);
void process_motor_level(uint8_t *buffer);
uint8_t response[RESPONSE_BUFFER_LENGTH];
static IncomingPacketFromAndroid incoming_packet_from_android;

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
    printf("call_queue entry removed. call_queue has %d entries remaining to handle\n", queue_get_level(&call_queue));
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
        printf("results_queue has %d entries remaining to handle\n", queue_get_level(&results_queue));
    }
}

// reads data from the UART and stores it in buffer. If no data is available, returns immediately.
// if new data is available, reads it until the buffer is full or both start and stop markers detected
// calls handle_block to process the data if both markers are detected
void get_block(IncomingPacketFromAndroid *packet) {
    // initialize as -1 as a way of detecting the absence of each marker in the buffer
    static int8_t start_idx = -1;
    static int8_t end_idx = -1;
    uint16_t buffer_index= 0;
    uint8_t i = 0;
    uint8_t MAX_SERIAL_GET_COUNT = 100;
    
    int c = getchar_timeout_us(100);
    // Only process data after finding the START_MARKER
    if (c != PICO_ERROR_TIMEOUT && c == START_MARKER){
        start_idx = buffer_index;
	// After finding the start marker get the rest of the packet or until MAX_SERIAL_GET_COUNT
	// to prevent an infinite loop
        while (buffer_index < sizeof(IncomingPacketFromAndroid) || i == MAX_SERIAL_GET_COUNT) {
            c = getchar_timeout_us(100);
    
    	    if (c != PICO_ERROR_TIMEOUT){
    	        if (buffer_index == 0){
    	    	    // First byte after start marker is the command
    	    	    packet->packet_type = (c & 0xFF);
    	    	    buffer_index++;
    	        }else{
    	    	    packet->data[buffer_index - 1] = (c & 0xFF);
    	    	    buffer_index++;
    	        }
    	    }else {
    	        assert(false);
    	    }
    	    i++;
        }
	
	c = getchar_timeout_us(100);

        if (c != PICO_ERROR_TIMEOUT && c == END_MARKER){
            // Calculate the length of the packet
            uint16_t packet_length = end_idx - start_idx;
            if (packet_length >= sizeof(IncomingPacketFromAndroid)) {
                // Call the handle_block function with the packet data
                handle_packet(packet);
                // Reset the values of start and end idx to detect the next block
                start_idx = -1;
                end_idx = -1;
                buffer_index = 0;
	        // Reset the packet
	        memset(packet, 0, sizeof(IncomingPacketFromAndroid));
            } else {
                printf("Received incomplete packet.\n");
		assert(false);
            }
        }else{
	    printf("Received packet with no end marker.\n");
	    assert(false);
	}

    }
}

void handle_packet(IncomingPacketFromAndroid *packet){
    // clear the response buffer
    memset(response, 0, RESPONSE_BUFFER_LENGTH);
    response[0] = START_MARKER;
    // Start with ACK response and only change to NACK if default case reached
    response[1] = ACK;
    uint8_t buffer_length = 3; // default to include START, ACK, and END markers
    switch (packet->packet_type){
    	case DO_NOTHING:
	        // TODO
	        break;
	case GET_CHARGE_DETAILS:
		// TODO
		break;
	case GET_LOG:
		// TODO
		break;
	case VARIOUS:
		// TODO
		break;
	case GET_ENCODER_COUNT:
		// TODO
		break;
	case RESET_ENCODER_COUNT:
		// TODO
		break;
	case SET_MOTOR_LEVEL:
		// TODO make this into a function
	        process_motor_level(packet->data);
		break;
	case SET_MOTOR_BRAKE:
		// TODO
		break;
	case GET_USB_VOLTAGE:
		// TODO
		break;
	default:
		response[1] = NACK;
		break;
    }
    response[RESPONSE_BUFFER_LENGTH - 1] = END_MARKER;
    for (int i = 0; i < RESPONSE_BUFFER_LENGTH; i++){
        putchar(response[i]);
    }
}

// Takes the response and add the quad encoder counts to it
void get_encoder_count(uint8_t *response){
	uint32_t left, right;
	left = quad_encoder_get_count(MOTOR_LEFT);
	right = quad_encoder_get_count(MOTOR_RIGHT);
	
	if (sizeof(uint32_t) == 4){
		memcpy(&left, &response[1], sizeof(uint32_t));
		memcpy(&right, &response[5], sizeof(uint32_t));
	}else{
		printf("uint32_t is not 4 bytes\n");
		assert (false);
	}
}

void process_motor_level(uint8_t *data){
    float left, right;
    if (sizeof(float) == 4){
        memcpy(&left, &data[0], sizeof(float));
        memcpy(&right, &data[4], sizeof(float));
    }else{
        printf("float is not 4 bytes\n");
        assert (false);
    }
    set_voltage(MOTOR_LEFT, left);
    set_voltage(MOTOR_RIGHT, right);
}


void send_block(uint8_t *buffer, uint8_t buffer_length){
    //printf("Testing");
    // check whether mResponse is large enough to hold all of response and the start/stop marks
    if (buffer_length > RESPONSE_BUFFER_LENGTH - 2){
	printf("buffer is too large to be sent");
	assert (false);
    }
    else{
	for (int i = 0; i < buffer_length; i++){
	    response[i+1] = buffer[i];
	}
	// print the response to the serial port including the start and end markers (+2)

    }
}

int main(){
    bool shutdown = false;
    on_start();
    //int i = 0;
    while (true)
    {
	//results_queue_pop();
        //sample_adc_inputs();
	//bq27742_g1_poll();
	//max77976_get_chg_details();
	//quad_encoder_update();
        //set_voltage(MOTOR_LEFT, 5.0);
        //set_voltage(MOTOR_RIGHT, 5.0);
	//sleep_ms(100);
	//set_voltage(MOTOR_LEFT, -5.0);
	//set_voltage(MOTOR_RIGHT, -5.0);
	//quad_encoder_update();
	//max77976_log_current_limit();
	//max77976_toggle_led();
        get_block(&incoming_packet_from_android);
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
    printf("on_start\n");
    stdio_init_all();
    gpio_set_irq_callback(&robot_interrupt_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    init_queues();
    multicore_launch_core1(core1_entry);
    // Waiting to make sure I can catch it within minicom
    sleep_ms(3000);
    printf("done waiting\n");
    i2c_start();
    adc_init();
    wrm483265_10f5_12v_g_init(WIRELESS_CHG_EN);
    ncp3901_init(GPIO_WIRELESS_AVAILABLE, GPIO_OTG);
    max77976_init(BATTERY_CHARGER_INTERRUPT_PIN, &call_queue, &results_queue);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO);
    max77958_init(MAX77958_INTB, &call_queue, &results_queue);
    sleep_ms(1000);
    printf("done waiting 2\n");
    bq27742_g1_init();
    bq27742_g1_fw_version_check();
    // Be sure to do this last
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO);
    drv8830_init(DRV8830_FAULT1, DRV8830_FAULT2);
    sleep_ms(1000);
    encoder_init(&call_queue);
    printf("encoders initialize. Waiting 1 second\n");
    sleep_ms(1000);
    printf("done waiting, turning on motors.\n");
    set_voltage(MOTOR_LEFT, 2.5);
    set_voltage(MOTOR_RIGHT, 2.5);
    int i = 0;
    while (i < 500){
	quad_encoder_update();
	i++;
	tight_loop_contents();
    }
    printf("done counting, turning off motors\n");
    set_voltage(MOTOR_LEFT, 0);
    set_voltage(MOTOR_RIGHT, 0);

    robot_unit_tests();
    printf("on_start complete\n");
    //while(!stdio_usb_connected()){
    //    sleep_ms(100);
    //}
    //printf("USB connected\n"); 
}

void robot_unit_tests(){
    printf("----------Running robot unit tests-----------\n");
    test_max77958_get_id();
    test_max77958_get_customer_config_id();
    test_max77958_interrupt();
    test_max77976_get_id();
    test_max77976_get_FSW();
    test_max77976_interrupt();
    test_ncp3901_interrupt();
    test_drv8830_get_faults();
    test_drv8830_interrupt();
    printf("-----------robot unit tests complete-----------\n");
}

void on_shutdown(){
    printf("Shutting down\n");
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
    	printf("free_queues: call_queue not empty\n");
    	sleep_ms(500);
    }
    queue_free(&call_queue);
    queue_free(&results_queue);
}

static void signal_stop_core1(){
    queue_entry_t stop_entry = {stop_core1, 0};
    if(!queue_try_add(&call_queue, &stop_entry)){
	printf("call_queue is full");
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

void bq27742_g1_poll(){
    bq27742_g1_get_voltage();
    bq27742_g1_get_temp();
    bq27742_g1_get_soh();
    bq27742_g1_get_flags();
    bq27742_g1_get_safety_stats();
}

void blink_led(uint8_t blinkCnt, int onTime, int offTime){
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    const uint8_t LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint8_t i = 0;
    while (i < blinkCnt)
    {
        printf("Hello blink\n");
        gpio_put(LED_PIN, 1);
        sleep_ms(onTime);
        gpio_put(LED_PIN, 0);
        sleep_ms(offTime);
        i++;
    }
#endif
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
    //printf("call_queue currently has %i entries\n", queue_get_level(&call_queue));
    if(!queue_try_add(&results_queue, &entry)){
        printf("results_queue is full");
    	assert(false);
    }
}

void call_queue_try_add(entry_func func, int32_t arg){
    queue_entry_t entry = {func, arg};
    //printf("call_queue currently has %i entries\n", queue_get_level(&call_queue));
    if(!queue_try_add(&call_queue, &entry)){
        printf("call_queue is full");
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
	    printf("Error during i2c_write. Returned value of %i %i \n", result, e);
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
	    printf("Error during i2c_read. Returned value of %i %i \n", result, e);
	    assert(false);
	}
}
