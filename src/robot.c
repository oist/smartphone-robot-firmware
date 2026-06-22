#include <stdio.h>
#include <stdlib.h>
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
#include <CException.h>
#include "quad_encoders.h"
#include "drv8830.h"
#include "hardware/uart.h"
#include <string.h>
#include "rp2040_log.h"
#include "serial_comm_manager.h"
#include "hardware/pwm.h"

// Use the same UART definitions as rp2040_log.c
#if PICO_DEFAULT_UART == 0
#define LOG_UART uart0
#else
#define LOG_UART uart1
#endif
#define LOG_UART_TX_PIN PICO_DEFAULT_UART_TX_PIN
#define LOG_UART_RX_PIN PICO_DEFAULT_UART_RX_PIN

#if LIB_PICO_STDIO_UART
static stdio_driver_t *driver=&stdio_uart;
#elif LIB_PICO_STDIO_USB
static stdio_driver_t *driver=&stdio_usb;
#endif

static queue_t call_queue;
static queue_t results_queue;
static bool core1_shutdown_requested = false;
static RP2040_STATE rp2040_state;

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
static void telemetry_cache_init(void);
static void telemetry_cache_copy_to_state(RP2040_STATE *state);
static void telemetry_cache_run_due_step(void);
void turn_on_leds();
uint8_t* i2c_scan(i2c_inst_t *i2c);

#define TELEMETRY_FAST_SAMPLE_INTERVAL_MS 50
#define TELEMETRY_SLOW_SAMPLE_INTERVAL_MS 150

typedef enum {
    TELEMETRY_FAST_SAMPLE_WIRELESS_ATTACHED = 0,
    TELEMETRY_FAST_SAMPLE_USB_CHARGER_VOLTAGE,
    TELEMETRY_FAST_SAMPLE_WIRELESS_VRECT,
    TELEMETRY_FAST_SAMPLE_COUNT
} telemetry_fast_sample_step_t;

typedef enum {
    TELEMETRY_SLOW_SAMPLE_BATTERY_VOLTAGE = 0,
    TELEMETRY_SLOW_SAMPLE_BATTERY_SAFETY,
    TELEMETRY_SLOW_SAMPLE_BATTERY_TEMP,
    TELEMETRY_SLOW_SAMPLE_BATTERY_SOH,
    TELEMETRY_SLOW_SAMPLE_BATTERY_FLAGS,
    TELEMETRY_SLOW_SAMPLE_MAX77976,
    TELEMETRY_SLOW_SAMPLE_MOTOR_FAULTS,
    TELEMETRY_SLOW_SAMPLE_COUNT
} telemetry_slow_sample_step_t;

typedef struct {
    RP2040_STATE state;
    absolute_time_t next_fast_sample_at;
    absolute_time_t next_slow_sample_at;
    telemetry_fast_sample_step_t fast_step;
    telemetry_slow_sample_step_t slow_step;
} telemetry_cache_t;

static telemetry_cache_t telemetry_cache;
#ifdef TELEMETRY_TIMING_DIAG
typedef struct {
    uint32_t count;
    int64_t total_us;
} telemetry_timing_stat_t;

typedef struct {
    telemetry_timing_stat_t encoder;
    telemetry_timing_stat_t motor_fault;
    telemetry_timing_stat_t charger;
    telemetry_timing_stat_t battery;
    telemetry_timing_stat_t get_state;
    telemetry_timing_stat_t charger_max77976;
    telemetry_timing_stat_t charger_wireless_attached;
    telemetry_timing_stat_t charger_ncp3901_adc;
    telemetry_timing_stat_t charger_stwlc38_adc;
    telemetry_timing_stat_t battery_voltage;
    telemetry_timing_stat_t battery_safety;
    telemetry_timing_stat_t battery_temp;
    telemetry_timing_stat_t battery_soh;
    telemetry_timing_stat_t battery_flags;
    telemetry_timing_stat_t i2c_write;
    telemetry_timing_stat_t i2c_read;
    uint32_t i2c_write_retries;
    uint32_t i2c_read_retries;
    uint32_t i2c_write_failures;
    uint32_t i2c_read_failures;
} telemetry_timing_diag_stats_t;

static telemetry_timing_diag_stats_t telemetry_timing_diag_stats;

static void telemetry_timing_diag_run(void);
static int64_t telemetry_timing_diag_measure_us(void (*func)(RP2040_STATE *), RP2040_STATE *state);
static void telemetry_timing_diag_add(telemetry_timing_stat_t *stat, int64_t elapsed_us);
static int64_t telemetry_timing_diag_average_us(const telemetry_timing_stat_t *stat);
static void telemetry_timing_diag_run_charger_breakdown(RP2040_STATE *state);
static void telemetry_timing_diag_run_battery_breakdown(RP2040_STATE *state);
static void telemetry_timing_diag_log_summary(void);
#endif
#ifdef EXTERNAL_MAX77958_TEST
static void external_max77958_i2c1_test(void);
static bool external_max77958_i2c1_read_register(uint8_t reg, uint8_t *dst, size_t len);
static bool external_max77958_i2c1_opcode_write(const uint8_t *command);
static bool external_max77958_i2c1_opcode_read(uint8_t *response, size_t len);
static bool external_max77958_i2c1_write_customer_config(void);
static bool external_max77958_i2c1_write_cc_ctrl1_src(void);
static bool external_max77958_i2c1_read_cc_ctrl1(void);
static bool external_max77958_i2c1_set_gpio(bool gpio4, bool gpio5);
static void external_max77958_i2c1_log_status(uint32_t elapsed_ms);
#endif

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
    //rp2040_log("call_queue entry removed. call_queue has %d entries remaining to handle\n", queue_get_level(&call_queue));
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
        //rp2040_log("results_queue has %d entries remaining to handle\n", queue_get_level(&results_queue));
    }
}

void get_state(RP2040_STATE* state){
    #ifndef BOARD_PICO
    get_encoder_counts(state);
    telemetry_cache_copy_to_state(state);
    #endif
}

void get_fast_motor_state(RP2040_STATE* state){
    #ifndef BOARD_PICO
    get_encoder_counts(state);
    telemetry_cache_copy_to_state(state);
    #endif
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
    state->ChargeSideUSB.wireless_charger_vrect = STWLC38JRM_adc1();
}

static void telemetry_cache_init(void)
{
    memset(&telemetry_cache, 0, sizeof(telemetry_cache));
    absolute_time_t now = get_absolute_time();
    telemetry_cache.next_fast_sample_at = now;
    telemetry_cache.next_slow_sample_at = now;
}

static void telemetry_cache_copy_to_state(RP2040_STATE *state)
{
    state->MotorsState.Faults = telemetry_cache.state.MotorsState.Faults;
    state->BatteryDetails = telemetry_cache.state.BatteryDetails;
    state->ChargeSideUSB = telemetry_cache.state.ChargeSideUSB;
}

static void telemetry_cache_run_fast_step(void)
{
    switch (telemetry_cache.fast_step) {
        case TELEMETRY_FAST_SAMPLE_WIRELESS_ATTACHED:
            telemetry_cache.state.ChargeSideUSB.wireless_charger_attached = ncp3901_wireless_charger_attached();
            break;
        case TELEMETRY_FAST_SAMPLE_USB_CHARGER_VOLTAGE:
            telemetry_cache.state.ChargeSideUSB.usb_charger_voltage = ncp3901_adc0();
            break;
        case TELEMETRY_FAST_SAMPLE_WIRELESS_VRECT:
            telemetry_cache.state.ChargeSideUSB.wireless_charger_vrect = STWLC38JRM_adc1();
            break;
        default:
            telemetry_cache.fast_step = TELEMETRY_FAST_SAMPLE_WIRELESS_ATTACHED;
            break;
    }

    telemetry_cache.fast_step++;
    if (telemetry_cache.fast_step >= TELEMETRY_FAST_SAMPLE_COUNT) {
        telemetry_cache.fast_step = TELEMETRY_FAST_SAMPLE_WIRELESS_ATTACHED;
    }

    telemetry_cache.next_fast_sample_at = make_timeout_time_ms(TELEMETRY_FAST_SAMPLE_INTERVAL_MS);
}

static void telemetry_cache_run_slow_step(void)
{
    switch (telemetry_cache.slow_step) {
        case TELEMETRY_SLOW_SAMPLE_BATTERY_VOLTAGE:
            telemetry_cache.state.BatteryDetails.voltage = bq27742_g1_get_voltage();
            break;
        case TELEMETRY_SLOW_SAMPLE_BATTERY_SAFETY:
            telemetry_cache.state.BatteryDetails.safety_status = bq27742_g1_get_safety_stats();
            break;
        case TELEMETRY_SLOW_SAMPLE_BATTERY_TEMP:
            telemetry_cache.state.BatteryDetails.temperature = bq27742_g1_get_temp();
            break;
        case TELEMETRY_SLOW_SAMPLE_BATTERY_SOH:
            telemetry_cache.state.BatteryDetails.state_of_health = bq27742_g1_get_soh();
            break;
        case TELEMETRY_SLOW_SAMPLE_BATTERY_FLAGS:
            telemetry_cache.state.BatteryDetails.flags = bq27742_g1_get_flags();
            break;
        case TELEMETRY_SLOW_SAMPLE_MAX77976:
            telemetry_cache.state.ChargeSideUSB.max77976_chg_details = max77976_get_chg_details();
            break;
        case TELEMETRY_SLOW_SAMPLE_MOTOR_FAULTS: {
            uint8_t *motor_faults = drv8830_get_faults();
            telemetry_cache.state.MotorsState.Faults.left = motor_faults[0];
            telemetry_cache.state.MotorsState.Faults.right = motor_faults[1];
            break;
        }
        default:
            telemetry_cache.slow_step = TELEMETRY_SLOW_SAMPLE_BATTERY_VOLTAGE;
            break;
    }

    telemetry_cache.slow_step++;
    if (telemetry_cache.slow_step >= TELEMETRY_SLOW_SAMPLE_COUNT) {
        telemetry_cache.slow_step = TELEMETRY_SLOW_SAMPLE_BATTERY_VOLTAGE;
    }

    telemetry_cache.next_slow_sample_at = make_timeout_time_ms(TELEMETRY_SLOW_SAMPLE_INTERVAL_MS);
}

static void telemetry_cache_run_due_step(void)
{
#ifndef BOARD_PICO
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, telemetry_cache.next_fast_sample_at) <= 0) {
        telemetry_cache_run_fast_step();
        return;
    }

    if (absolute_time_diff_us(now, telemetry_cache.next_slow_sample_at) <= 0) {
        telemetry_cache_run_slow_step();
    }
#endif
}

void set_motor_levels(RP2040_STATE* state){
    #ifndef BOARD_PICO
    set_motor_control(MOTOR_LEFT, state->MotorsState.ControlValues.left);
    set_motor_control(MOTOR_RIGHT, state->MotorsState.ControlValues.right);
    #endif
}

int main(){
    bool shutdown = false;
    on_start();
    sleep_ms(1000);
    while (true){
        bool handled_packet = false;
        for (uint8_t i = 0; i < 8 && get_block(); i++) {
            handled_packet = true;
        }
	if (shutdown){
	    on_shutdown();
	    break;
	}else{
	    // This sleep or some other time consuming function must occur else can't reset from gdb as thread will be stuck in tight_loop_contents()
            if (!handled_packet) {
                telemetry_cache_run_due_step();
                sleep_ms(1);
            }
	    tight_loop_contents();
	}
    }
    on_shutdown();

    return 0;
}

void on_start(){
    rp2040_log_init();

    #ifdef LOGGER_UART
    // Force UART reset to clear any residual buffers
    uart_deinit(LOG_UART);
    sleep_ms(50);
    uart_init(LOG_UART, 115200);
    gpio_set_function(LOG_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LOG_UART_RX_PIN, GPIO_FUNC_UART);
    #endif

    rp2040_log("=== FIRMWARE RESTART ===\n");
    rp2040_log("on_start\n");
    stdio_init_all();
    stdio_set_translate_crlf(driver, false);
    gpio_set_irq_callback(&robot_interrupt_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);
    init_queues();
    multicore_launch_core1(core1_entry);

    #ifndef BOARD_PICO
    i2c_start();
    #ifdef EXTERNAL_MAX77958_TEST
    external_max77958_i2c1_test();
    return;
    #endif
    adc_init();
    turn_on_leds();
    STWLC38JRM_init(WIRELESS_CHG_EN, WIRELESS_CHG_VRECT);
    ncp3901_init(GPIO_WIRELESS_AVAILABLE, GPIO_OTG);
    max77976_init(BATTERY_CHARGER_INTERRUPT_PIN, &call_queue, &results_queue);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO1);
    sn74ahc125rgyr_init(SN74AHC125RGYR_GPIO2);
    max77958_init(MAX77958_INTB, &call_queue, &results_queue);
    #endif

    sleep_ms(1000);
    rp2040_log("done waiting 2\n");

    #ifndef BOARD_PICO
    bq27742_g1_init();
    bq27742_g1_fw_version_check();
    // Be sure to do this last
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO1);
    sn74ahc125rgyr_on_end_of_start(SN74AHC125RGYR_GPIO2);
    drv8830_init(DRV8830_FAULT1, DRV8830_FAULT2);
    #endif

    sleep_ms(1000);
    
    #ifndef BOARD_PICO
    encoder_init(&call_queue);
    #endif
    
    rp2040_log("encoders initialize. Waiting 1 second\n");
    sleep_ms(1000);
    rp2040_log("done waiting, Running unit tests.\n");

    #ifndef BOARD_PICO
    set_voltage(MOTOR_LEFT, 2.5);
    set_voltage(MOTOR_RIGHT, 2.5);
    int i = 0;
    while (i < 50){
       quad_encoder_update();
       i++;
       tight_loop_contents();
    }
    rp2040_log("done counting, turning off motors\n");
    set_voltage(MOTOR_LEFT, 0);
    set_voltage(MOTOR_RIGHT, 0);
    robot_unit_tests();
    #endif

    serial_comm_manager_init(&rp2040_state);
    telemetry_cache_init();

    #ifndef BOARD_PICO
    i2c_scan(i2c0);
    i2c_scan(i2c1);
    //STWLC38_get_ept_reasons(); // Note I am only adding this here so I can access it from gdb later
    read_reg(0x9);
    read_reg(0xA);
    read_reg(0xD);
    #endif

    rp2040_log("on_start complete\n");
    max77958_on_start_complete();
#ifdef TELEMETRY_TIMING_DIAG
    telemetry_timing_diag_run();
#endif
    //while(!stdio_usb_connected()){
    //    sleep_ms(100);
    //}
    //rp2040_log("USB connected\n");
}

#ifdef TELEMETRY_TIMING_DIAG
static int64_t telemetry_timing_diag_measure_us(void (*func)(RP2040_STATE *), RP2040_STATE *state)
{
    absolute_time_t start = get_absolute_time();
    func(state);
    return absolute_time_diff_us(start, get_absolute_time());
}

static void telemetry_timing_diag_add(telemetry_timing_stat_t *stat, int64_t elapsed_us)
{
    stat->count++;
    stat->total_us += elapsed_us;
}

static int64_t telemetry_timing_diag_average_us(const telemetry_timing_stat_t *stat)
{
    if (stat->count == 0) {
        return 0;
    }
    return stat->total_us / stat->count;
}

static void telemetry_timing_diag_run(void)
{
#ifdef BOARD_PICO
    rp2040_log("TELEMETRY_TIMING: skipped on BOARD_PICO\n");
#else
    const uint8_t iterations = 5;

    memset(&telemetry_timing_diag_stats, 0, sizeof(telemetry_timing_diag_stats));
    rp2040_log("TELEMETRY_TIMING: begin iterations=%u\n", iterations);
    for (uint8_t i = 0; i < iterations; i++) {
        RP2040_STATE state;
        memset(&state, 0, sizeof(state));

        int64_t encoder_us = telemetry_timing_diag_measure_us(get_encoder_counts, &state);
        int64_t motor_fault_us = telemetry_timing_diag_measure_us(get_motor_faults, &state);
        int64_t charger_us = telemetry_timing_diag_measure_us(get_charger_state, &state);
        int64_t battery_us = telemetry_timing_diag_measure_us(get_battery_state, &state);

        memset(&state, 0, sizeof(state));
        int64_t total_us = telemetry_timing_diag_measure_us(get_state, &state);

        telemetry_timing_diag_add(&telemetry_timing_diag_stats.encoder, encoder_us);
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.motor_fault, motor_fault_us);
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.charger, charger_us);
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery, battery_us);
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.get_state, total_us);

        memset(&state, 0, sizeof(state));
        telemetry_timing_diag_run_charger_breakdown(&state);
        telemetry_timing_diag_run_battery_breakdown(&state);
    }
    telemetry_timing_diag_log_summary();
    rp2040_log("TELEMETRY_TIMING: end\n");
#endif
}

static void telemetry_timing_diag_run_charger_breakdown(RP2040_STATE *state)
{
    absolute_time_t start = get_absolute_time();
    state->ChargeSideUSB.max77976_chg_details = max77976_get_chg_details();
    int64_t max77976_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.charger_max77976, max77976_us);

    start = get_absolute_time();
    state->ChargeSideUSB.wireless_charger_attached = ncp3901_wireless_charger_attached();
    int64_t wireless_attached_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.charger_wireless_attached, wireless_attached_us);

    start = get_absolute_time();
    state->ChargeSideUSB.usb_charger_voltage = ncp3901_adc0();
    int64_t ncp3901_adc_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.charger_ncp3901_adc, ncp3901_adc_us);

    start = get_absolute_time();
    state->ChargeSideUSB.wireless_charger_vrect = STWLC38JRM_adc1();
    int64_t stwlc38_adc_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.charger_stwlc38_adc, stwlc38_adc_us);
}

static void telemetry_timing_diag_run_battery_breakdown(RP2040_STATE *state)
{
    absolute_time_t start = get_absolute_time();
    state->BatteryDetails.voltage = bq27742_g1_get_voltage();
    int64_t voltage_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery_voltage, voltage_us);

    start = get_absolute_time();
    state->BatteryDetails.safety_status = bq27742_g1_get_safety_stats();
    int64_t safety_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery_safety, safety_us);

    start = get_absolute_time();
    state->BatteryDetails.temperature = bq27742_g1_get_temp();
    int64_t temp_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery_temp, temp_us);

    start = get_absolute_time();
    state->BatteryDetails.state_of_health = bq27742_g1_get_soh();
    int64_t soh_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery_soh, soh_us);

    start = get_absolute_time();
    state->BatteryDetails.flags = bq27742_g1_get_flags();
    int64_t flags_us = absolute_time_diff_us(start, get_absolute_time());
    telemetry_timing_diag_add(&telemetry_timing_diag_stats.battery_flags, flags_us);
}

static void telemetry_timing_diag_log_summary(void)
{
    rp2040_log("TELEMETRY_TIMING_SUMMARY: count=%u encoder_avg_us=%lld motor_fault_avg_us=%lld charger_avg_us=%lld battery_avg_us=%lld get_state_avg_us=%lld\n",
               telemetry_timing_diag_stats.get_state.count,
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.encoder),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.motor_fault),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.charger),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.get_state));
    rp2040_log("TELEMETRY_TIMING_SUMMARY_CHARGER: count=%u max77976_avg_us=%lld wireless_attached_avg_us=%lld ncp3901_adc_avg_us=%lld stwlc38_adc_avg_us=%lld\n",
               telemetry_timing_diag_stats.charger_max77976.count,
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.charger_max77976),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.charger_wireless_attached),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.charger_ncp3901_adc),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.charger_stwlc38_adc));
    rp2040_log("TELEMETRY_TIMING_SUMMARY_BATTERY: count=%u voltage_avg_us=%lld safety_avg_us=%lld temp_avg_us=%lld soh_avg_us=%lld flags_avg_us=%lld\n",
               telemetry_timing_diag_stats.battery_voltage.count,
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery_voltage),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery_safety),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery_temp),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery_soh),
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.battery_flags));
    rp2040_log("TELEMETRY_TIMING_SUMMARY_I2C: write_count=%u write_avg_us=%lld write_retries=%u write_failures=%u read_count=%u read_avg_us=%lld read_retries=%u read_failures=%u\n",
               telemetry_timing_diag_stats.i2c_write.count,
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.i2c_write),
               telemetry_timing_diag_stats.i2c_write_retries,
               telemetry_timing_diag_stats.i2c_write_failures,
               telemetry_timing_diag_stats.i2c_read.count,
               (long long)telemetry_timing_diag_average_us(&telemetry_timing_diag_stats.i2c_read),
               telemetry_timing_diag_stats.i2c_read_retries,
               telemetry_timing_diag_stats.i2c_read_failures);
}
#endif

void robot_unit_tests(){
    rp2040_log("----------Running robot unit tests-----------\n");
    test_max77958_get_id();
    test_max77958_status_block_read_all();
    test_max77958_bc_ctrl1_read();
    test_max77958_bc_ctrl2_read();
    test_max77958_control1_read();
    test_max77958_cc_ctrl1_read();
    test_max77958_cc_ctrl4_read();
    test_max77958_gpio_control_read();
    test_max77958_gpio0_gpio1_adc_read();
    test_max77958_snk_pdo_request();
    test_max77958_get_customer_config();
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

    #ifndef BOARD_PICO
    max77958_shutdown(MAX77958_INTB);
    //sn74ahc125rgyr_shutdown(SN74AHC125RGYR_GPIO);
    //max77976_shutdown();
    //ncp3901_shutdown();
    //wrm483265_10f5_12v_g_shutdown(WIRELESS_CHG_EN);
    adc_shutdown();
    bq27742_g1_shutdown();
    // Note this will shut off the battery to the rp2040 so unless you're plugged in, everything will fail here.
    // TODO how do I wake from this if the rp2040 has no power to respond??
    #endif

    signal_stop_core1();
    free_queues();
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
	rp2040_log("ERROR: call_queue is full");
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

#ifdef EXTERNAL_MAX77958_TEST
#define EXT_MAX77958_ADDR 0x25
#define EXT_MAX77958_OPCODE_WRITE 0x21
#define EXT_MAX77958_OPCODE_READ 0x51
#define EXT_MAX77958_OPCODE_GPIO_READ 0x23
#define EXT_MAX77958_OPCODE_GPIO_WRITE 0x24
#define EXT_MAX77958_OPCODE_CC_CTRL1_READ 0x0B
#define EXT_MAX77958_OPCODE_CC_CTRL1_WRITE 0x0C
#define EXT_MAX77958_OPCODE_CUSTOMER_CONFIG_WRITE 0x56
#define EXT_MAX77958_STATUS_POLL_MS 4000
#define EXT_MAX77958_STATUS_INTERVAL_MS 250

static void external_max77958_i2c1_test(void)
{
    uint8_t id[2] = {0x00, 0x00};
    bool vbus_enabled = false;

    rp2040_log("EXT_MAX77958_I2C1_TEST: begin addr=0x%02x sda=%u scl=%u\n",
                EXT_MAX77958_ADDR, I2C_SDA1, I2C_SCL1);

    if (!external_max77958_i2c1_read_register(0x00, id, sizeof id)) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
        return;
    }

    rp2040_log("EXT_MAX77958_I2C1_TEST: DEVICE_ID=0x%02x DEVICE_REV=0x%02x %s\n",
                id[0], id[1], (id[0] == 0x58 && id[1] == 0x02) ? "PASS" : "FAIL");

    if (id[0] != 0x58 || id[1] != 0x02) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
        return;
    }

    external_max77958_i2c1_log_status(0);

    rp2040_log("EXT_MAX77958_I2C1_TEST: forcing GPIO4/GPIO5 low before source config\n");
    if (!external_max77958_i2c1_set_gpio(false, false)) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
        return;
    }
    sleep_ms(100);
    external_max77958_i2c1_log_status(100);

    if (!external_max77958_i2c1_write_customer_config()) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
        return;
    }
    sleep_ms(100);

    if (!external_max77958_i2c1_write_cc_ctrl1_src()) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
        return;
    }
    sleep_ms(100);

    external_max77958_i2c1_read_cc_ctrl1();

    for (uint32_t elapsed_ms = 0;
         elapsed_ms <= EXT_MAX77958_STATUS_POLL_MS;
         elapsed_ms += EXT_MAX77958_STATUS_INTERVAL_MS) {
        uint8_t cc_status0 = 0;

        external_max77958_i2c1_log_status(elapsed_ms);
        if (!vbus_enabled &&
            external_max77958_i2c1_read_register(0x0C, &cc_status0, 1) &&
            (cc_status0 & 0x07) == 0x02) {
            rp2040_log("EXT_MAX77958_I2C1_TEST: source attach detected, enabling GPIO4/GPIO5\n");
            if (external_max77958_i2c1_set_gpio(true, true)) {
                vbus_enabled = true;
            }
        }

        if (elapsed_ms == EXT_MAX77958_STATUS_POLL_MS) {
            break;
        }
        sleep_ms(EXT_MAX77958_STATUS_INTERVAL_MS);
    }

    rp2040_log("EXT_MAX77958_I2C1_TEST: vbus_enabled=%u\n", vbus_enabled ? 1 : 0);
    rp2040_log("EXT_MAX77958_I2C1_TEST: end\n");
}

static bool external_max77958_i2c1_read_register(uint8_t reg, uint8_t *dst, size_t len)
{
    int write_result = i2c_write_timeout_us(i2c1, EXT_MAX77958_ADDR, &reg, 1, true, I2C_TIMEOUT);
    if (write_result != 1) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL reg=0x%02x write_result=%d\n", reg, write_result);
        return false;
    }

    int read_result = i2c_read_timeout_us(i2c1, EXT_MAX77958_ADDR, dst, len, false, I2C_TIMEOUT);
    if (read_result != (int)len) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL reg=0x%02x read_result=%d expected=%u\n",
                    reg, read_result, (unsigned)len);
        return false;
    }

    return true;
}

static bool external_max77958_i2c1_opcode_write(const uint8_t *command)
{
    uint8_t write_end[] = {0x41, 0x00};
    int write_result = i2c_write_timeout_us(i2c1, EXT_MAX77958_ADDR, command, 33, false, I2C_TIMEOUT);
    if (write_result != 33) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL opcode=0x%02x write_result=%d\n",
                    command[1], write_result);
        return false;
    }

    int end_result = i2c_write_timeout_us(i2c1, EXT_MAX77958_ADDR, write_end, sizeof write_end, false, I2C_TIMEOUT);
    if (end_result != (int)sizeof write_end) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL opcode=0x%02x end_result=%d\n",
                    command[1], end_result);
        return false;
    }

    return true;
}

static bool external_max77958_i2c1_opcode_read(uint8_t *response, size_t len)
{
    uint8_t reg = EXT_MAX77958_OPCODE_READ;
    int write_result = i2c_write_timeout_us(i2c1, EXT_MAX77958_ADDR, &reg, 1, true, I2C_TIMEOUT);
    if (write_result != 1) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL opcode read pointer write_result=%d\n", write_result);
        return false;
    }

    int read_result = i2c_read_timeout_us(i2c1, EXT_MAX77958_ADDR, response, len, false, I2C_TIMEOUT);
    if (read_result != (int)len) {
        rp2040_log("EXT_MAX77958_I2C1_TEST: FAIL opcode read_result=%d expected=%u\n",
                    read_result, (unsigned)len);
        return false;
    }

    return true;
}

static bool external_max77958_i2c1_write_customer_config(void)
{
    uint8_t command[33] = {0};
    max77958_customer_config_t config = {
        .dbg_src_enable = false,
        .dbg_snk_enable = false,
        .audio_acc_enable = false,
        .trysnk_enable = false,
        .typec_mode = TYPEC_MODE_SRC,
        .mem_update_customer = false,
        .moisture_enable = false
    };

    command[0] = EXT_MAX77958_OPCODE_WRITE;
    command[1] = EXT_MAX77958_OPCODE_CUSTOMER_CONFIG_WRITE;
    command[2] = max77958_build_customer_config_value(&config);
    command[3] = 0x6A;
    command[4] = 0x0B;
    command[5] = 0x60;
    command[6] = 0x68;
    command[7] = 0x00;
    command[8] = 0x64;
    command[9] = 0x00;
    command[10] = 0x96;
    command[11] = 0x00;
    command[21] = 0x69;
    command[22] = 0x69;
    command[23] = 0x35;
    command[24] = 0x28;

    rp2040_log("EXT_MAX77958_I2C1_TEST: writing source customer config=0x%02x\n", command[2]);
    return external_max77958_i2c1_opcode_write(command);
}

static bool external_max77958_i2c1_write_cc_ctrl1_src(void)
{
    uint8_t command[33] = {0};

    command[0] = EXT_MAX77958_OPCODE_WRITE;
    command[1] = EXT_MAX77958_OPCODE_CC_CTRL1_WRITE;
    command[2] = 0x82;

    rp2040_log("EXT_MAX77958_I2C1_TEST: writing CC_CTRL1=0x82\n");
    return external_max77958_i2c1_opcode_write(command);
}

static bool external_max77958_i2c1_read_cc_ctrl1(void)
{
    uint8_t command[33] = {0};
    uint8_t response[33] = {0};

    command[0] = EXT_MAX77958_OPCODE_WRITE;
    command[1] = EXT_MAX77958_OPCODE_CC_CTRL1_READ;
    if (!external_max77958_i2c1_opcode_write(command)) {
        return false;
    }

    sleep_ms(100);
    if (!external_max77958_i2c1_opcode_read(response, sizeof response)) {
        return false;
    }

    rp2040_log("EXT_MAX77958_I2C1_TEST: CC_CTRL1 response opcode=0x%02x value=0x%02x\n",
                response[0], response[1]);
    return response[0] == EXT_MAX77958_OPCODE_CC_CTRL1_READ;
}

static bool external_max77958_i2c1_set_gpio(bool gpio4, bool gpio5)
{
    uint8_t command[33] = {0};
    uint8_t gpio_val = (gpio5 ? 0x08 : 0x00) | 0x04 | (gpio4 ? 0x02 : 0x00) | 0x01;

    command[0] = EXT_MAX77958_OPCODE_WRITE;
    command[1] = EXT_MAX77958_OPCODE_GPIO_WRITE;
    command[2] = 0x00;
    command[3] = gpio_val;

    rp2040_log("EXT_MAX77958_I2C1_TEST: writing GPIO4=%u GPIO5=%u GPIO53=0x%02x\n",
                gpio4 ? 1 : 0, gpio5 ? 1 : 0, gpio_val);
    return external_max77958_i2c1_opcode_write(command);
}

static void external_max77958_i2c1_log_status(uint32_t elapsed_ms)
{
    uint8_t status[8] = {0};
    uint8_t gpio_command[33] = {0};
    uint8_t gpio_response[33] = {0};

    if (!external_max77958_i2c1_read_register(0x08, status, sizeof status)) {
        return;
    }

    uint8_t cc_status0 = status[4];
    uint8_t cc_status1 = status[5];
    uint8_t pd_status0 = status[6];
    uint8_t pd_status1 = status[7];
    rp2040_log("EXT_MAX77958_I2C1_TEST t=%lums USBC1=0x%02x SYS=0x%02x BC=0x%02x CC0=0x%02x pin=%u current=%u state=%u CC1=0x%02x wtr=%u detabrt=%u vsafeov=%u PD0=0x%02x PD1=0x%02x power=%u data=%u psrdy=%u\n",
                (unsigned long)elapsed_ms,
                status[0], status[1], status[2],
                cc_status0, (cc_status0 >> 6) & 0x03, (cc_status0 >> 4) & 0x03, cc_status0 & 0x07,
                cc_status1, (cc_status1 >> 1) & 0x01, (cc_status1 >> 2) & 0x01, (cc_status1 >> 3) & 0x01,
                pd_status0, pd_status1, (pd_status1 >> 6) & 0x01, (pd_status1 >> 7) & 0x01,
                (pd_status1 >> 4) & 0x01);

    gpio_command[0] = EXT_MAX77958_OPCODE_WRITE;
    gpio_command[1] = EXT_MAX77958_OPCODE_GPIO_READ;
    if (!external_max77958_i2c1_opcode_write(gpio_command)) {
        return;
    }

    sleep_ms(20);
    if (!external_max77958_i2c1_opcode_read(gpio_response, sizeof gpio_response)) {
        return;
    }

    rp2040_log("EXT_MAX77958_I2C1_TEST t=%lums GPIO opcode=0x%02x GPIO03=0x%02x GPIO47=0x%02x GPIO8=0x%02x\n",
                (unsigned long)elapsed_ms, gpio_response[0], gpio_response[1],
                gpio_response[2], gpio_response[3]);
}
#endif

uint8_t* i2c_scan(i2c_inst_t *i2c) {
    uint8_t MAX_DEVICES = 128;
    uint8_t* found_addresses = malloc(MAX_DEVICES);
    uint8_t count = 0;
    uint8_t rxdata;

    for (uint8_t address = 0x08; address < 0x78; ++address) {
        sleep_ms(10); // give some time between transactions
        if (i2c_read_blocking(i2c, address, &rxdata, 1, false) > 0) {
		found_addresses[count++] = address;
	}
    }

    found_addresses[count] = 0; // Null-terminate the array
    return found_addresses;
}

void adc_shutdown(){
}

void turn_on_leds(){
    gpio_set_function(LED_EN_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(LED_EN_PIN);

    pwm_set_wrap(slice, 255);                    // 8-bit PWM range (0–255)
    pwm_set_gpio_level(LED_EN_PIN, 128);         // ~50% duty cycle
    pwm_set_enabled(slice, true);
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
    STWLC38JRM_adc1();
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
        rp2040_log("ERROR:results_queue is full");
	assert(false);
    }
}

void call_queue_try_add(entry_func func, int32_t arg){
    queue_entry_t entry = {func, arg};
    //rp2040_log("call_queue currently has %i entries\n", queue_get_level(&call_queue));
    if(!queue_try_add(&call_queue, &entry)){
        rp2040_log("ERROR: call_queue is full");
	assert(false);
    }
}

void quad_encoders_callback(){
    // GPIO12-15 monitor past and current states to determine counts
}

void i2c_write_error_handling(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop){
    int result;
    int retries = 0;
#ifdef TELEMETRY_TIMING_DIAG
    absolute_time_t start = get_absolute_time();
#endif
    Try{
        do {
	    result = i2c_write_timeout_us(i2c, addr, src, len, nostop, I2C_TIMEOUT);
            if (result >= 0) {
#ifdef TELEMETRY_TIMING_DIAG
                int64_t elapsed_us = absolute_time_diff_us(start, get_absolute_time());
                telemetry_timing_diag_add(&telemetry_timing_diag_stats.i2c_write, elapsed_us);
                telemetry_timing_diag_stats.i2c_write_retries += retries;
#endif
                return;
            }
            retries++;
            if (retries < MAX_RETRIES) {
                sleep_ms(RETRY_DELAY);
            }
        } while (retries < MAX_RETRIES);
        Throw(result);
    }
    Catch(e){
#ifdef TELEMETRY_TIMING_DIAG
	int64_t elapsed_us = absolute_time_diff_us(start, get_absolute_time());
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.i2c_write, elapsed_us);
        telemetry_timing_diag_stats.i2c_write_retries += retries;
        telemetry_timing_diag_stats.i2c_write_failures++;
#endif
	rp2040_log("ERROR: During i2c_write. Returned value of %i %i \n", result, e);
	assert(false);
    }
}

void i2c_read_error_handling(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop){
    int result;
    int retries = 0;
#ifdef TELEMETRY_TIMING_DIAG
    absolute_time_t start = get_absolute_time();
#endif
    Try{
       do {
           result = i2c_read_timeout_us(i2c, addr, dst, len, nostop, I2C_TIMEOUT);
           if (result >= 0) {
#ifdef TELEMETRY_TIMING_DIAG
               int64_t elapsed_us = absolute_time_diff_us(start, get_absolute_time());
               telemetry_timing_diag_add(&telemetry_timing_diag_stats.i2c_read, elapsed_us);
               telemetry_timing_diag_stats.i2c_read_retries += retries;
#endif
               return;
           }
           retries++;
           if (retries < MAX_RETRIES) {
	       sleep_ms(RETRY_DELAY);
           }
       } while (retries < MAX_RETRIES);
       Throw(result);
    }
    Catch(e){
#ifdef TELEMETRY_TIMING_DIAG
	int64_t elapsed_us = absolute_time_diff_us(start, get_absolute_time());
        telemetry_timing_diag_add(&telemetry_timing_diag_stats.i2c_read, elapsed_us);
        telemetry_timing_diag_stats.i2c_read_retries += retries;
        telemetry_timing_diag_stats.i2c_read_failures++;
#endif
	rp2040_log("ERROR: During i2c_read. Returned value of %i %i \n", result, e);
	assert(false);
    }
}
