#include <stdio.h>
#include "pico/stdlib.h"
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

volatile CEXCEPTION_T e;

// core1 will be used to process all function calls requested by interrupt calls on core0
void core1_entry() {
    while (core1_shutdown_requested == false) {
        // Function pointer is passed to us via the queue_entry_t which also
        // contains the function parameter.
        // We provide an int32_t return value by simply pushing it back on the
        // return queue which also indicates the result is ready.

	sleep_ms(1);
	int32_t result = call_queue_pop();
        queue_add_blocking(&results_queue, &result);
        // as an alternative to polling the return queue, you can send an irq to core0 to add another entry to call_queue
    }
}

int32_t call_queue_pop(){
    queue_entry_t entry;
    queue_remove_blocking(&call_queue, &entry);
    int32_t (*func)() = (int32_t(*)())(entry.func);
    printf("core1_entry: calling function pointer from call_queue entry\n");
    int32_t result = (*func)(entry.data);
    printf("core1_entry: result from calling call_queue entry = %d\n", (int)result);
    return result;
}

void stop_core1(){
    core1_shutdown_requested = true;
}

void results_queue_pop(){
    // If the results_queue is not empty, take the first entry and call its function on core0
    if (!queue_is_empty(&results_queue)){
        queue_entry_t entry;
        //queue_try_remove(&results_queue, &entry);
        queue_remove_blocking(&results_queue, &entry);
        // TODO implement what to do with results_queue entries.
        printf("Handled an entry from the results queue\n");
    }
}

int main(){
    bool shutdown = false;
    on_start();
    int i = 0;
    while (i < 10000)
    {
	results_queue_pop();
        //sample_adc_inputs();
	//bq27742_g1_poll();
	//max77976_get_chg_details();
	//max77976_log_current_limit();
	//max77976_toggle_led();
	if (shutdown){
	    on_shutdown();
	    break;
	}else{
	    // This sleep or some other time consuming function must occur else can't reset from gdb as thread will be stuck in tight_loop_contents()
            sleep_ms(1);
	    tight_loop_contents();
	}
	i++;
    }
    on_shutdown();

    return 0;
}

void on_start(){
    printf("on_start\n");
    stdio_init_all();
    init_queues();
    multicore_launch_core1(core1_entry);
    // Waiting to make sure I can catch it within minicom
    sleep_ms(3000);
    printf("done waiting\n");
    i2c_start();
    adc_init();
    wrm483265_10f5_12v_g_init(WIRELESS_CHG_EN);
    ncp3901_init(GPIO_WIRELESS_AVAILABLE, GPIO_OTG);
    max77976_init(BATTERY_CHARGER_INTERRUPT_PIN);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO);
    max77958_init(MAX77958_INTB, &call_queue, &results_queue);
    sleep_ms(1000);
    printf("done waiting 2\n");
    bq27742_g1_init();
    bq27742_g1_fw_version_check();
    // Be sure to do this last
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO);
    printf("on_start complete\n");
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
    queue_init(&call_queue, sizeof(queue_entry_t), 8);
    queue_init(&results_queue, sizeof(int32_t), 8);
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
    queue_entry_t stop_entry = {&stop_core1, 0};
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
