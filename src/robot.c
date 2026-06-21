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
void turn_on_leds();
uint8_t* i2c_scan(i2c_inst_t *i2c);
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
    // ChargeSideUSB: Mux MAX77976 and NCP3901 Data: Charger-side USB Voltage, and Wireless Coil State
    //uint16_t acdcValue = ncp3901_adc0();
    //memcpy(&response[2], &acdcValue, sizeof(uint16_t));
    // BatteryDetails: BQ27742-G1 Data: Battery Voltage, Current, and State of Charge

    // PhoneSideUSB: MAX77958 Phone-side USB Controller

    // MotorDetails: DRV8830DRCR Data: Includes MOTOR_FAULT, ENCODER_COUNTS, MOTOR_LEVELS, MOTOR_BRAKE
    //TODO this should not update response with static int values like this
    #ifndef BOARD_PICO
    get_encoder_counts(state);
    get_motor_faults(state);
    get_charger_state(state);
    get_battery_state(state);
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
        get_block();
	if (shutdown){
	    on_shutdown();
	    break;
	}else{
	    // This sleep or some other time consuming function must occur else can't reset from gdb as thread will be stuck in tight_loop_contents()
            sleep_ms(10);
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
    //while(!stdio_usb_connected()){
    //    sleep_ms(100);
    //}
    //rp2040_log("USB connected\n");
}

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
    Try{
        do {
	    result = i2c_write_timeout_us(i2c, addr, src, len, nostop, I2C_TIMEOUT);
            if (result >= 0) {
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
	rp2040_log("ERROR: During i2c_write. Returned value of %i %i \n", result, e);
	assert(false);
    }
}

void i2c_read_error_handling(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop){
    int result;
    int retries = 0;
    Try{
       do {
           result = i2c_read_timeout_us(i2c, addr, dst, len, nostop, I2C_TIMEOUT);
           if (result >= 0) {
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
	rp2040_log("ERROR: During i2c_read. Returned value of %i %i \n", result, e);
	assert(false);
    }
}
