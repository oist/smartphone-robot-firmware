#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "max77958.h"
#include "max77958_driver.h"
#include "bit_ops.h"
#include "robot.h"
#include <inttypes.h>
#include "custom_printf.h"

static uint8_t send_buf[33] = {0};
static uint8_t return_buf[33] = {0}; 
static uint8_t op_code_return_buf[33] = {0}; // Will read full buffer from registers 0x52 to 0x71
#define PDMSG_POWER_SUPPLY_VBUS_ENABLE 0x17
#define PDMSG_POWER_SUPPLY_VBUS_DISABLE 0x18
#define MAX77958_SWAP_REQ_DR_SWAP 0x01
#define MAX77958_SWAP_REQ_PR_SWAP 0x02
#define MAX77958_SWAP_RESP_SOURCE_UFP 0x10
#define CC_STATUS0_STATE_SOURCE 0x02
#define PD_STATUS1_DATA_ROLE_DFP (1u << 7)
#define PD_STATUS1_PSRDY (1u << 4)
static queue_t* call_queue_ptr;
static queue_t* return_queue_ptr;
static bool opcode_cmd_finished = false;
static bool power_swap_enabled = true;
static bool opcodes_finished = false;
static bool data_role_swap_requested = false;
static queue_t opcode_queue;
static uint8_t _gpio_interrupt;
static uint8_t interrupt_mask = GPIO_IRQ_EDGE_FALL;
static bool test_max77958_interrupt_bool = false;
static bool test_max77958_started = false;
static bool test_max77958_completed = false;

static int32_t parse_interrupt_vals();
static void on_interrupt();
static void get_interrupt_vals();
static void get_interrupt_masks();
static void set_interrupt_masks();
static uint8_t max77958_read_register(uint8_t reg);
static void opcode_read();
static int opcode_write(uint8_t *send_buf);
static int32_t max77958_test_response();
static void power_swap_request();
static int32_t swap_response_write(void);
static int32_t data_role_swap_request(void);
static bool queue_data_role_swap_to_ufp_once(void);
static int32_t set_snk_pdos();
static int32_t pd_msg_response();
static int32_t customer_config_write();
static bool opcode_queue_pop();
static void opcode_queue_add(int32_t (*opcode_func)(), int32_t opcode_data);
static int32_t gpio_bool_to_int32(bool _GPIO4, bool _GPIO5);
static int32_t gpio_set(int32_t gpio_val);
#ifdef MAX77958_FORCE_VBUS_DIAGNOSTIC
static int32_t force_vbus_on_for_diagnostic(void);
#endif
static int32_t set_src_pdos();
static void on_ccstat_change();
static void on_chgtype_change();
static void on_ccvcnstat_change();
static void on_ccistat_change();
static void on_ccpinstat_change();
static void vbus_turn_off();
static void vbus_turn_on();
static int32_t bc_ctrl1_read();
static int32_t bc_ctrl2_read();
static int32_t control1_read();
static int32_t cc_ctrl1_read();
static int32_t cc_ctrl1_write_src_only(void);
static int32_t cc_ctrl4_read(void);
static int32_t gpio_control_read(void);
static int32_t gpio0_gpio1_adc_read(void);
static int32_t snk_pdo_request();
static int32_t customer_config_read();
typedef struct {
    const char *name;
    uint8_t reg;
    uint8_t expected;
} max77958_status_reg_t;

typedef struct {
    uint8_t usbc_status1;
    uint8_t usbc_status2;
    uint8_t bc_status;
    uint8_t cc_status0;
    uint8_t cc_status1;
    uint8_t pd_status0;
    uint8_t pd_status1;
    uint8_t gpio0_3;
    uint8_t gpio4_7;
    uint8_t gpio8;
    bool gpio_valid;
} max77958_debug_status_t;

static const char *cc_state_name(uint8_t state)
{
    switch (state) {
        case 0: return "NO_CONNECTION";
        case 1: return "SINK_ATTACHED";
        case 2: return "SOURCE_ATTACHED";
        case 3: return "AUDIO_ACCESSORY";
        case 4: return "DEBUG_SOURCE";
        case 5: return "ERROR";
        case 6: return "DISABLED";
        case 7: return "DEBUG_SINK";
        default: return "UNKNOWN";
    }
}

static const char *data_role_name(uint8_t data_role)
{
    return data_role ? "DFP_HOST" : "UFP_DEVICE";
}

static const char *ready_name(uint8_t psrdy)
{
    return psrdy ? "READY" : "NOT_READY";
}



void max77958_on_interrupt(uint gpio, uint32_t event_mask){
    if (event_mask & interrupt_mask){
        gpio_acknowledge_irq(_gpio_interrupt, interrupt_mask);	
	call_queue_try_add(&parse_interrupt_vals, 0);
        if (test_max77958_started){
            call_queue_try_add(&max77958_test_response, 1);
        }
    }
}

static int on_pd_msg_received(){
    rp2040_log("Rec PD message\n");
    call_queue_try_add(&pd_msg_response, 0);
    return 0;
}

static int on_opcode_cmd_response(){
    // You can now READ back the OpCommand return registers
    opcode_read();
    // opcode_queue_pop will return false if the opcode queue is empty
    if (!opcode_queue_pop()){
        opcodes_finished = true;
    }
    return 0;
}

static void on_ccstat_change(void) {
    rp2040_log("CCStat: ccstat changed\n");

    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0x0C; // CC_STATUS0
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 2, false);

    uint8_t cc_status0 = return_buf[0];
    uint8_t cc_status1 = return_buf[1];

    // ----- CC_STATUS0 bitfields -----
    uint8_t CCPinStat  = cc_status0 & 0b11000000;   // bits [7:6]
    uint8_t CCIStat    = cc_status0 & 0b00110000;   // bits [5:4]
    bool    CCVcnStat  = cc_status0 & 0b00001000;   // bit  [3]
    uint8_t CCStat     = cc_status0 & 0b00000111;   // bits [2:0]

    // ----- CC_STATUS1 bitfields -----
    bool VCONN_OCP = cc_status1 & 0b00100000;  // bit [5]
    bool VCONN_SC  = cc_status1 & 0b00010000;  // bit [4]
    bool VSafeOV   = cc_status1 & 0b00001000;  // bit [3]
    bool DetAbrt   = cc_status1 & 0b00000100;  // bit [2]
    bool Wtr       = cc_status1 & 0b00000010;  // bit [1]

    rp2040_log("  CCStat: CCPinStat=%s\n",
        (CCPinStat == 0b00000000 ? "00 (none)" :
        (CCPinStat == 0b01000000 ? "01 (CC1)" :
        (CCPinStat == 0b10000000 ? "10 (CC2)" : "11 (reserved)"))));

    rp2040_log("  CCStat: CCIStat=%s\n",
        (CCIStat == 0b00000000 ? "00 (none)" :
        (CCIStat == 0b00010000 ? "01 (500mA)" :
        (CCIStat == 0b00100000 ? "10 (1.5A)" : "11 (3.0A)"))));

    rp2040_log("  CCStat: CCVcnStat=%s\n", CCVcnStat ? "1 (VCONN ON)" : "0 (VCONN OFF)");

    rp2040_log("  CCStat=%s\n",
        (CCStat == 0b000 ? "000 (none)" :
        (CCStat == 0b001 ? "001 (sink)" :
        (CCStat == 0b010 ? "010 (source)" :
        (CCStat == 0b011 ? "011 (audio)" :
        (CCStat == 0b100 ? "100 (debug)" :
        (CCStat == 0b101 ? "101 (error)" :
        (CCStat == 0b110 ? "110 (disabled)" : "111 (debug sink)"))))))));

    // ---- Original CCStat switch preserved exactly ----
    switch (CCStat){
        case 0b000:
            rp2040_log("CCStat: ccstat changed to no connection\n");
            vbus_turn_off();
            break;
        case 0b001:
            rp2040_log("CCStat: ccstat changed to SINK\n");
            vbus_turn_off();
            break;
        case 0b010:
            rp2040_log("CCStat: ccstat changed to SOURCE\n");
            vbus_turn_on();
            break;
        default:
            rp2040_log("CCStat: ccstat changed to %d\n", CCStat);
            break;
    }

    // ---- Fault / condition flags ----
    if (Wtr)
        rp2040_log("ERROR: CCStat: Moisture detected on CC (Wtr=1)\n");
    if (DetAbrt)
        rp2040_log("ERROR: CCStat: Charger detection aborted (DetAbrt=1)\n");
    if (VSafeOV)
        rp2040_log("ERROR: CCStat: VBUS overvoltage detected (VSafeOV=1)\n");
    if (VCONN_SC)
        rp2040_log("ERROR: CCStat: VCONN short-circuit detected (VCONN_SC=1)\n");
    if (VCONN_OCP)
        rp2040_log("ERROR: CCStat: VCONN overcurrent detected (VCONN_OCP=1)\n");
}

static void on_chgtype_change(){
    rp2040_log("chgtype changed\n");
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = 0xA; // BC_STATUS
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    uint8_t ChgType = return_buf[0] & 0b11;
    switch (ChgType){
	case 0b000:
	    rp2040_log("ChgTyp changed to nothing attached\n");
	    break;
	case 0b001:
	    rp2040_log("ChgTyp changed to SDP, USB cable attached\n");
	    break;
	case 0b010:
	    rp2040_log("ChgTyp changed to CDP, Charging Downstream Port\n");
	    break;
	default: 
	    rp2040_log("ChgTyp changed to DCP, Dedicated charger\n");
	    break;
	}
}

static void opcode_queue_add(int32_t (opcode_func)(), int32_t opcode_data){
    opcodes_finished = false;
    queue_entry_t opcode_entry = {opcode_func, opcode_data};
    if(!queue_try_add(&opcode_queue, &opcode_entry)){
	rp2040_log("ERROR: opcode_queue is full");
	assert(false);
    }
    rp2040_log("Added to opcode_queue. %d entries remaining\n", queue_get_level(&opcode_queue));
} 

static int32_t parse_interrupt_vals(){
    uint16_t return_val = 0;
    get_interrupt_vals();
    // don't really need these, but makes it easier to understand what each entry to the return_buf represents
    uint8_t* UIC_INT = &return_buf[0]; 
    uint8_t* CC_INT = &return_buf[1]; 
    uint8_t* PD_INT = &return_buf[2]; 
    // Check if the APCmdResI interrupt is on (AP command response pending)
    uint8_t APCmdResI_mask = 1 << 7;
    uint8_t PSRDYI_mask = 1 << 6;
    uint8_t PDMsgI = 1 << 7;
    uint8_t CCStat = 1 << 0;
    uint8_t ChgType = 1 << 1;
    uint8_t CCVcnStatI = 1 << 1;
    uint8_t CCIStatI = 1 << 2;
    uint8_t CCPinStatI = 1 << 3;
    if (*UIC_INT & APCmdResI_mask){
	on_opcode_cmd_response();
	return_val |= 1 << 0;
    }
    if (*PD_INT & PSRDYI_mask){
	rp2040_log("Power source ready\n");
	//on_power_source_ready();
        if (queue_data_role_swap_to_ufp_once()) {
            opcode_queue_pop();
        }
	return_val |= 1 << 1;
    }
    if (*PD_INT & PDMsgI){
	on_pd_msg_received();
	return_val |= 1 << 2;
    }
    if (*UIC_INT & ChgType){
	on_chgtype_change();
	return_val |= 1 << 3;
    }
    if (*CC_INT & CCStat){
	on_ccstat_change();
	return_val |= 1 << 4;
    }
    if (*CC_INT & CCVcnStatI){
	on_ccvcnstat_change();
	return_val |= 1 << 5;
    }
    if (*CC_INT & CCIStatI){
	on_ccistat_change();
	return_val |= 1 << 6;
    }
    if (*CC_INT & CCPinStatI){
	on_ccpinstat_change();
	return_val |= 1 << 7;
    }
    if (return_val == 0){
	test_max77958_interrupt_bool = true;
    }
    return return_val;
    
    // Check for other relevant interrupts here and do something with that info...
}

static void on_ccvcnstat_change(){
    rp2040_log("CCStat changed\n");
}

static void on_ccistat_change(){
    rp2040_log("CCIStat changed\n");
}

static void on_ccpinstat_change(){
    rp2040_log("CCPinStat changed\n");
}

static void get_interrupt_vals(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT; // 0x04 Register
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
    //rp2040_log("interrupts vals: 0x4: 0x%02x, 0x5: 0x%02x, 0x6: 0x%02x, 0x7: 0x%02x\n", return_buf[0], return_buf[1], return_buf[2], return_buf[3]);
}

static void get_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 4, false);
}

// Mask all interrupts for unit test purposes
static void set_interrupt_masks_all_masked(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b11111111; // UIC_INT_M 0x10 values
    send_buf[2] = 0b11111111; // CC_INT_M 0x11 values
    send_buf[3] = 0b11111111; // PD_INT_M 0x12 unmasking PSRDYI
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 4, false);
}

static void set_interrupt_masks(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = REG_UIC_INT_M; // 0x10 UIC_INT_M Register
    send_buf[1] = 0b00000100; // UIC_INT_M 0x10 values
    send_buf[2] = 0b00000000; // CC_INT_M 0x11 values
    send_buf[3] = 0b00111111; // PD_INT_M 0x12 unmasking PSRDYI
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 4, false);
}

void read_reg(uint8_t reg){
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = reg; 
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    rp2040_log("read_reg: 0x%02x: 0x%02x\n", reg, return_buf[0]);
}

static bool max77958_debug_read_gpio_control(max77958_debug_status_t *status)
{
    memset(op_code_return_buf, 0, sizeof op_code_return_buf);
    opcodes_finished = false;

    opcode_queue_add(gpio_control_read, 0);
    if (!opcode_queue_pop()) {
        rp2040_log("MAX77958_DIAG: GPIO read queue was empty\n");
        return false;
    }

    uint32_t waited_ms = 0;
    while (!opcodes_finished && waited_ms < 500) {
        sleep_ms(10);
        waited_ms += 10;
    }

    if (!opcodes_finished) {
        rp2040_log("MAX77958_DIAG: GPIO read timed out\n");
        return false;
    }

    if (op_code_return_buf[0] != OPCODE_READ_GPIO) {
        rp2040_log("MAX77958_DIAG: GPIO read unexpected opcode 0x%02x\n", op_code_return_buf[0]);
        return false;
    }

    status->gpio0_3 = op_code_return_buf[1];
    status->gpio4_7 = op_code_return_buf[2];
    status->gpio8 = op_code_return_buf[3];
    status->gpio_valid = true;
    return true;
}

static max77958_debug_status_t max77958_debug_read_status(void)
{
    max77958_debug_status_t status = {0};

    status.usbc_status1 = max77958_read_register(0x08);
    status.usbc_status2 = max77958_read_register(0x09);
    status.bc_status = max77958_read_register(0x0A);
    status.cc_status0 = max77958_read_register(0x0C);
    status.cc_status1 = max77958_read_register(0x0D);
    status.pd_status0 = max77958_read_register(0x0E);
    status.pd_status1 = max77958_read_register(0x0F);
    max77958_debug_read_gpio_control(&status);

    return status;
}

static void max77958_debug_log_status(uint32_t elapsed_ms, const max77958_debug_status_t *status)
{
    uint8_t vbadc = (status->usbc_status1 >> 3) & 0x1F;
    uint8_t cc_pin = (status->cc_status0 >> 6) & 0x03;
    uint8_t cc_current = (status->cc_status0 >> 4) & 0x03;
    uint8_t cc_state = status->cc_status0 & 0x07;
    uint8_t power_role = (status->pd_status1 >> 6) & 0x01;
    uint8_t data_role = (status->pd_status1 >> 7) & 0x01;
    uint8_t psrdy = (status->pd_status1 >> 4) & 0x01;
    uint8_t g4_dir = status->gpio4_7 & 0x01;
    uint8_t g4_out = (status->gpio4_7 >> 1) & 0x01;
    uint8_t g5_dir = (status->gpio4_7 >> 2) & 0x01;
    uint8_t g5_out = (status->gpio4_7 >> 3) & 0x01;

    rp2040_log("MAX77958_DIAG t=%" PRIu32 "ms USBC1=0x%02x VbADC=%u SYS=0x%02x BC=0x%02x\n",
                elapsed_ms, status->usbc_status1, vbadc, status->usbc_status2, status->bc_status);
    rp2040_log("MAX77958_DIAG t=%" PRIu32 "ms CC0=0x%02x pin=%u current=%u state=%u(%s)\n",
                elapsed_ms, status->cc_status0, cc_pin, cc_current, cc_state,
                cc_state_name(cc_state));
    rp2040_log("MAX77958_DIAG t=%" PRIu32 "ms CC1=0x%02x wtr=%u detabrt=%u vsafeov=%u\n",
                elapsed_ms, status->cc_status1,
                (status->cc_status1 >> 1) & 0x01,
                (status->cc_status1 >> 2) & 0x01,
                (status->cc_status1 >> 3) & 0x01);
    rp2040_log("MAX77958_DIAG t=%" PRIu32 "ms PD0=0x%02x PD1=0x%02x power_raw=%u data=%u(%s) psrdy=%u(%s)\n",
                elapsed_ms, status->pd_status0, status->pd_status1, power_role,
                data_role, data_role_name(data_role), psrdy, ready_name(psrdy));
    rp2040_log("MAX77958_DIAG t=%" PRIu32 "ms GPIO53=0x%02x valid=%u g4=%u/%u g5=%u/%u\n",
                elapsed_ms, status->gpio4_7, status->gpio_valid ? 1 : 0,
                g4_dir, g4_out, g5_dir, g5_out);
}

void max77958_debug_poll_status(uint32_t duration_ms, uint32_t interval_ms)
{
    if (interval_ms == 0) {
        interval_ms = 1000;
    }

    rp2040_log("MAX77958_DIAG: begin status poll for %" PRIu32 "ms every %" PRIu32 "ms\n",
                duration_ms, interval_ms);

    uint32_t elapsed_ms = 0;
    while (elapsed_ms <= duration_ms) {
        max77958_debug_status_t status = max77958_debug_read_status();
        max77958_debug_log_status(elapsed_ms, &status);

        if (elapsed_ms == duration_ms) {
            break;
        }

        uint32_t sleep_for_ms = interval_ms;
        if (elapsed_ms + sleep_for_ms > duration_ms) {
            sleep_for_ms = duration_ms - elapsed_ms;
        }
        sleep_ms(sleep_for_ms);
        elapsed_ms += sleep_for_ms;
    }

    rp2040_log("MAX77958_DIAG: end status poll\n");
}

static int opcode_write(uint8_t *buf){
    // buf should always be 32 bytes long since the register values from 0x22 to 0x41 are never overwritten,
    // so you may send wrong data if you don't directly specify them for ALL registers. Note the defaults are NOT always 0x00,
    // so you should send all values everytime. What a pain...
    if (buf[0] != 0x21){
	rp2040_log("ERROR: buffer should always start with the 0x21 register");
    }

    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, buf, sizeof(send_buf), false);
    //rp2040_log("opcode_write: 0x%02x 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3]);

    // For whatever reason, this is necessary for the interrupt to fire. Even though I already write 0x00 to it in the line above.
    memset(send_buf, 0, sizeof &send_buf);
    send_buf[0] = 0x41;
    send_buf[1] = 0x00;
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 2, false);
    return 1;
}

static void opcode_read(){
    // Set the current register pointer to 0x51 to you can read the return values from the OpCode Command
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_READ_COMMAND;
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, op_code_return_buf, 33, false);
    rp2040_log("opcode_read: 0x%02x 0x%02x 0x%02x 0x%02x\n", op_code_return_buf[0], op_code_return_buf[1], op_code_return_buf[2], op_code_return_buf[3]);
    if (op_code_return_buf[0] == 0x0B) {
        rp2040_log("CC_CTRL1 readback = 0x%02x\n", op_code_return_buf[1]);
    }
    // Set breakpoint before clearing the registers via the following command. 
    // I'm commenting this out since I won't use the output in the code, but you can copy/paste this into gdb if you want to
    // inspect the results before clearing them

    // Clear registers 0x21 - 0x41 to ensure you don't write wrong values to opCode Commands later
    //memset(return_buf, 0, sizeof return_buf);
    //return_buf[0] = 0x21;
    //i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 32, false);
}

static uint8_t max77958_read_register(uint8_t reg)
{
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = reg;
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    return return_buf[0];
}

void test_max77958_status_block_read_all(void)
{
    const max77958_status_reg_t regs[] = {
        {"USBC_STATUS1", 0x08, 0b00000111},
        {"USBC_STATUS2", 0x09, 0b00000000},
        {"USBC_STATUS3", 0x0A, 0b00000000},
        {"USBC_STATUS4", 0x0B, 0b00000000},
        {"CC_STATUS0",   0x0C, 0b00000000},
        {"CC_STATUS1",   0x0D, 0b00000000},
        {"PD_STATUS0",   0x0E, 0b00000000},
        {"PD_STATUS1",   0x0F, 0b00000000}
    };

    rp2040_log("test_max77958_status_block_read_all started...\n");

    for (uint8_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        uint8_t val = max77958_read_register(regs[i].reg);

        if (val != regs[i].expected)
            rp2040_log("test_max77958_status_block_read_all ERROR: %s expected 0b" BYTE_TO_BINARY_PATTERN ", got 0b" BYTE_TO_BINARY_PATTERN "\n",
                       regs[i].name, BYTE_TO_BINARY(regs[i].expected), BYTE_TO_BINARY(val));
        else
            rp2040_log("test_max77958_status_block_read_all PASSED: %s matches reset value 0b" BYTE_TO_BINARY_PATTERN "\n",
                       regs[i].name, BYTE_TO_BINARY(val));
    }
}


void test_max77958_get_id(){
    rp2040_log("test_max77958_get_id started...\n");
    // Testing for just DEVICE_ID
    // Write the register 0x00 to set the pointer there before reading its value
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 2, false);
    if (return_buf[0] != 0x58){
	rp2040_log("test_max77958_get_id ERROR: DEVICE_ID should be 0x58");
    }
    if (return_buf[1] != 0x02){
	rp2040_log("test_max77958_get_id ERROR: DEVICE_REV should be 0x02");
    }
    rp2040_log("test_max77958_get_id PASSED: DEVICE_ID = %x\n", return_buf[0]);
    rp2040_log("test_max77958_get_id PASSED: DEVICE_REV = %x\n", return_buf[1]);
}

static int32_t bc_ctrl1_read(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x01; // BC CTRL1 Config Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_bc_ctrl1_read(){
    rp2040_log("test_max77958_bc_ctrl1_read started...\n");
    opcode_queue_add(bc_ctrl1_read, 0);
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished){
	sleep_ms(100);
	i++;
	if (i > 10){
	    rp2040_log("test_max77958_bc_ctrl1_read ERROR: Timed out waiting for GPIO to finish\n");
	}
    }
    if (op_code_return_buf[0] != 0x01){
	rp2040_log("test_max77958_bc_ctrl1_read ERROR: OPCODE should be 0x01");
    }
    if (op_code_return_buf[1] != 0b10000001){
        rp2040_log("test_max77958_bc_ctrl1_read ERROR: BC_CTRL1_CONFIG should be 0b10000001 instead it is 0b"
           BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]));
    }else{
	rp2040_log("test_max77958_bc_ctrl1_read PASSED: BC_CTRL1_CONFIG = 0b"
	   BYTE_TO_BINARY_PATTERN "\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
    }
}

static int32_t bc_ctrl2_read()
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x03; // BC CTRL2 Config Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_bc_ctrl2_read(void)
{
    rp2040_log("test_max77958_bc_ctrl2_read started...\n");
    opcode_queue_add(bc_ctrl2_read, 0);
    opcode_queue_pop();

    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_bc_ctrl2_read ERROR: Timed out waiting for response\n");
        }
    }

    if (op_code_return_buf[0] != 0x03) {
		rp2040_log("test_max77958_bc_ctrl2_read ERROR: OPCODE should be 0x03 (got 0x%02x)\n",
				   op_code_return_buf[0]);
	}

    if (op_code_return_buf[1] != 0b00000001) {
        rp2040_log("test_max77958_bc_ctrl2_read ERROR: OPCODE should be 0b00000001 got 0b"
	   BYTE_TO_BINARY_PATTERN "\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
    }else{
	rp2040_log("test_max77958_bc_ctrl2_read PASSED: BC_CTRL2_CONFIG = 0b"
           BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]));
    }
}


static int32_t control1_read()
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x05; // CONTROL1 Config Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_control1_read(void)
{
    rp2040_log("test_max77958_control1_read started...\n");
    opcode_queue_add(control1_read, 0);
    opcode_queue_pop();

    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_control1_read ERROR: Timed out waiting for response\n");
        }
    }

    if (op_code_return_buf[0] != 0x05) {
    	rp2040_log("test_max77958_control1_read ERROR: OPCODE should be 0x05 (got 0x%02x)\n",
    			   op_code_return_buf[0]);
    }

    if (op_code_return_buf[1] != 0b00000000) {
        rp2040_log("test_max77958_control1_read ERROR: OPCODE should be 0b00000000 (got 0b"
	   BYTE_TO_BINARY_PATTERN ")\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
    }else{
	rp2040_log("test_max77958_control1_read PASSED: CONTROL1_CONFIG = 0b"
           BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]));
    }
}

static int32_t cc_ctrl1_read(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x0B; // CC CTRL1 Config Read
    opcode_write(send_buf);
    return 0;
}

static int32_t cc_ctrl1_write_src_only(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x0C; // CC CTRL1 Config Write
    send_buf[2] = 0x82; // VCONN auto, Try.SNK off, source-only CC detection
    rp2040_log("cc_ctrl1_write_src_only: setting CC_CTRL1 to 0x82\n");
    opcode_write(send_buf);
    return 0;
}

void test_max77958_cc_ctrl1_read(){
    rp2040_log("test_max77958_cc_ctrl1_read started...\n");
    opcode_queue_add(cc_ctrl1_read, 0);
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished){
	sleep_ms(100);
	i++;
	if (i > 10){
	    rp2040_log("test_max77958_cc_ctrl1_read ERROR: Timed out waiting for GPIO to finish\n");
	}
    }
    if (op_code_return_buf[0] != 0x0B){
	rp2040_log("test_max77958_cc_ctrl1_read ERROR: OPCODE should be 0x0B");
    }
    if (op_code_return_buf[1] != 0b10000010){
        rp2040_log("test_max77958_cc_ctrl1_read ERROR: CC_CTRL1_CONFIG should be 0b10000010 instead it is 0b"
           BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]));
    }else{
	rp2040_log("test_max77958_cc_ctrl1_read PASSED: CC_CTRL1_CONFIG = 0b"
	   BYTE_TO_BINARY_PATTERN "\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
    }
}


static int32_t cc_ctrl4_read(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x11; // CC_CTRL4 Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_cc_ctrl4_read(void)
{
    rp2040_log("test_max77958_cc_ctrl4_read started...\n");
    opcode_queue_add(cc_ctrl4_read, 0);
    opcode_queue_pop();

    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_cc_ctrl4_read ERROR: Timed out waiting for response\n");
        }
    }

    if (op_code_return_buf[0] != 0x11) {
        rp2040_log("test_max77958_cc_ctrl4_read ERROR: OPCODE should be 0x11 (got 0x%02x)\n",
                   op_code_return_buf[0]);
    }

    if (op_code_return_buf[1] != 0b00000000) {
		rp2040_log("test_max77958_cc_ctrl4_read ERROR: OPCODE should be 0b00000000 (got 0b"
				   BYTE_TO_BINARY_PATTERN ")\n",
				   BYTE_TO_BINARY(op_code_return_buf[1]));
    }else{
	rp2040_log("test_max77958_cc_ctrl4_read PASSED: CC_CTRL4 = 0b"
           BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]));
    }
}

static int32_t gpio_control_read(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x23; // GPIO Control Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_gpio_control_read(void)
{
    rp2040_log("test_max77958_gpio_control_read started...\n");
    opcode_queue_add(gpio_control_read, 0);
    opcode_queue_pop();

    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_gpio_control_read ERROR: Timed out waiting for response\n");
        }
    }

    if (op_code_return_buf[0] != 0x23) {
        rp2040_log("test_max77958_gpio_control_read ERROR: OPCODE should be 0x23 (got 0x%02x)\n",
                   op_code_return_buf[0]);
    }

    int error_count = 0;

    if (op_code_return_buf[1] != 0b00000000) {
	rp2040_log("test_max77958_gpio_control_read ERROR: OPCODE should be 0b00000000 (got 0b"
	   BYTE_TO_BINARY_PATTERN ")\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
	error_count++;
    }
    if (op_code_return_buf[2] != 0b00000000) {
	rp2040_log("test_max77958_gpio_control_read ERROR: OPCODE should be 0b00000000 (got 0b"
	   BYTE_TO_BINARY_PATTERN ")\n",
	   BYTE_TO_BINARY(op_code_return_buf[2]));
	error_count++;
    }
    if (op_code_return_buf[3] != 0b00000000) {
    	rp2040_log("test_max77958_gpio_control_read ERROR: OPCODE should be 0b00000000 (got 0b"
    	   BYTE_TO_BINARY_PATTERN ")\n",
    	   BYTE_TO_BINARY(op_code_return_buf[3]));
		error_count++;
    }
    if (error_count > 0) {
        rp2040_log("test_max77958_gpio_control_read PASSED: "
           "0x52(GPIO0-3)=0b" BYTE_TO_BINARY_PATTERN " "
           "0x53(GPIO4-7)=0b" BYTE_TO_BINARY_PATTERN " "
           "0x54(GPIO8)=0b"   BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]),
           BYTE_TO_BINARY(op_code_return_buf[2]),
           BYTE_TO_BINARY(op_code_return_buf[3]));
    }
}


static int32_t gpio0_gpio1_adc_read(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x27; // SBU1/SBU2 ADC Read
    opcode_write(send_buf);
    return 0;
}

void test_max77958_gpio0_gpio1_adc_read(void)
{
    rp2040_log("test_max77958_gpio0_gpio1_adc_read started...\n");
    opcode_queue_add(gpio0_gpio1_adc_read, 0);
    opcode_queue_pop();

    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_gpio0_gpio1_adc_read ERROR: Timed out waiting for response\n");
        }
    }

    if (op_code_return_buf[0] != 0x27) {
        rp2040_log("test_max77958_gpio0_gpio1_adc_read ERROR: OPCODE should be 0x27 (got 0x%02x)\n",
                   op_code_return_buf[0]);
    }

    if (op_code_return_buf[1] != 0b00000000 || op_code_return_buf[2] != 0b00000000) {
	rp2040_log("test_max77958_gpio0_gpio1_adc_read ERROR: SBU1 should be 0b00000000 (got 0b"
	   BYTE_TO_BINARY_PATTERN ")\n",
	   BYTE_TO_BINARY(op_code_return_buf[1]));
	rp2040_log("test_max77958_gpio0_gpio1_adc_read ERROR: SBU2 should be 0b00000000 (got 0b"
	   BYTE_TO_BINARY_PATTERN ")\n",
	   BYTE_TO_BINARY(op_code_return_buf[2]));
    }else{
	rp2040_log("test_max77958_gpio0_gpio1_adc_read PASSED: "
           "SBU1=0b" BYTE_TO_BINARY_PATTERN " "
           "SBU2=0b" BYTE_TO_BINARY_PATTERN "\n",
           BYTE_TO_BINARY(op_code_return_buf[1]),
           BYTE_TO_BINARY(op_code_return_buf[2]));
    }
}


static bool use_mtp_access = false; // global or static flag

static int32_t snk_pdo_request(void) {
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x3E; // SNK PDO Request

    // Bit7 of 0x22 = memory select: 0 = RAM, 1 = MTP
    send_buf[2] = (use_mtp_access ? 0b10000000 : 0b00000000);

    opcode_write(send_buf);
    return 0;
}

void test_max77958_snk_pdo_request(void)
{
    rp2040_log("test_max77958_snk_pdo_request started...\n");

    // ---- Read MTP copy ----
    use_mtp_access = false;  // for RAM access
    opcode_queue_add(snk_pdo_request, 0);
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_snk_pdo_request ERROR: Timed out waiting for MTP\n");
        }
    }
    if (op_code_return_buf[0] != 0x3E) {
        rp2040_log("test_max77958_snk_pdo_request ERROR: OPCODE echo mismatch for MTP (got 0x%02x)\n",
                   op_code_return_buf[0]);
    }

    uint8_t mtp_pdo_count = op_code_return_buf[1];
    rp2040_log("test_max77958_snk_pdo_request MTP PDO count = %d\n", mtp_pdo_count);
    for (int j = 0; j < mtp_pdo_count; j++) {
        uint32_t pdo = op_code_return_buf[2 + j * 4]
                     | (op_code_return_buf[3 + j * 4] << 8)
                     | (op_code_return_buf[4 + j * 4] << 16)
                     | (op_code_return_buf[5 + j * 4] << 24);
        rp2040_log("test_max77958_snk_pdo_request MTP PDO[%d] = 0x%08" PRIx32 "\n", j, pdo);
    }

    // ---- Read RAM copy ----
    use_mtp_access = true;  // for RAM access
    opcode_queue_add(snk_pdo_request, 0);
    opcode_queue_pop();
    i = 0;
    while (!opcodes_finished) {
        sleep_ms(100);
        if (++i > 10) {
            rp2040_log("test_max77958_snk_pdo_request ERROR: Timed out waiting for RAM\n");
        }
    }
    if (op_code_return_buf[0] != 0x3E) {
        rp2040_log("test_max77958_snk_pdo_request ERROR: OPCODE echo mismatch for RAM (got 0x%02x)\n",
                   op_code_return_buf[0]);
    }

    uint8_t ram_pdo_count = op_code_return_buf[1];
    rp2040_log("test_max77958_snk_pdo_request RAM PDO count = %d\n", ram_pdo_count);
    for (int j = 0; j < ram_pdo_count; j++) {
        uint32_t pdo = op_code_return_buf[2 + j * 4]
                     | (op_code_return_buf[3 + j * 4] << 8)
                     | (op_code_return_buf[4 + j * 4] << 16)
                     | (op_code_return_buf[5 + j * 4] << 24);
        rp2040_log("test_max77958_snk_pdo_request RAM PDO[%d] = 0x%08" PRIx32 "\n", j, pdo);
    }

    rp2040_log("test_max77958_snk_pdo_request PASSED: MTP=%d PDOs, RAM=%d PDOs\n",
               mtp_pdo_count, ram_pdo_count);
}


static int32_t customer_config_read(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x55; // Customer Configuration Write
    opcode_write(send_buf);
    return 0;
}

void test_max77958_get_customer_config(){
    rp2040_log("test_max77958_get_customer_config started...\n");
    opcode_queue_add(customer_config_read, 0);
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished){
	sleep_ms(100);
	i++;
	if (i > 10){
	    rp2040_log("test_max77958_get_customer_config ERROR: Timed out waiting for GPIO to finish\n");
	}
    }
    if (op_code_return_buf[0] != 0x55){
	rp2040_log("test_max77958_get_customer_config ERROR: OPCODE should be 0x55");
    }
    if ((op_code_return_buf[2] | (op_code_return_buf[3] << 8)) != 0x0B6A){
        rp2040_log("test_max77958_get_customer_config ERROR: CUSTOMER_CONFIG_ID should be 0x0B6A");
    }
    if ((op_code_return_buf[4] | (op_code_return_buf[5] << 8)) != 0x6860){
	rp2040_log("test_max77958_get_customer_config ERROR: CUSTOMER_CONFIG_REV should be 0x6860");
    }
    rp2040_log("test_max77958_get_customer_config: 0x51=0x%02x\n", op_code_return_buf[0]);
    rp2040_log("test_max77958_get_customer_config: 0x52=0x%02x\n", op_code_return_buf[1]);
    rp2040_log("test_max77958_get_customer_config: 0x52=0b"
		   BYTE_TO_BINARY_PATTERN "\n",
		   BYTE_TO_BINARY(op_code_return_buf[1]));

    rp2040_log("test_max77958_get_customer_config: 0x53=0x%02x\n", op_code_return_buf[2]);
    rp2040_log("test_max77958_get_customer_config: 0x54=0x%02x\n", op_code_return_buf[3]);
    rp2040_log("test_max77958_get_customer_config: 0x55=0x%02x\n", op_code_return_buf[4]);
    rp2040_log("test_max77958_get_customer_config: 0x56=0x%02x\n", op_code_return_buf[5]);
    rp2040_log("test_max77958_get_customer_config: 0x57=0x%02x\n", op_code_return_buf[6]);
    rp2040_log("test_max77958_get_customer_config: 0x58=0x%02x\n", op_code_return_buf[7]);
    rp2040_log("test_max77958_get_customer_config: 0x59=0x%02x\n", op_code_return_buf[8]);
    rp2040_log("test_max77958_get_customer_config: 0x5A=0x%02x\n", op_code_return_buf[9]);
    rp2040_log("test_max77958_get_customer_config: 0x5B=0x%02x\n", op_code_return_buf[10]);

    rp2040_log("test_max77958_get_customer_config PASSED: CUSTOMER_CONFIG_ID = 0x%04x\n", (op_code_return_buf[2] | (op_code_return_buf[3] << 8)));
    rp2040_log("test_max77958_get_customer_config PASSED: CUSTOMER_CONFIG_REV = 0x%04x\n", (op_code_return_buf[4] | (op_code_return_buf[5] << 8)));
}

void test_max77958_interrupt(){
    rp2040_log("test_max77958_interrupt started...\n");
    set_interrupt_masks_all_masked();

    test_max77958_started = true;
    rp2040_log("test_max77958_interrupt: prior to driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    gpio_set_dir(_gpio_interrupt, GPIO_OUT);
    if (gpio_get(_gpio_interrupt) != 0){
	rp2040_log("ERROR: test_max77958_interrupt: GPIO%d was not driven low. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    }
    rp2040_log("test_max77958_interrupt: after driving low GPIO%d. Current Value:%d\n", _gpio_interrupt, gpio_get(_gpio_interrupt));
    uint32_t i = 0;
    while (!test_max77958_completed){
        sleep_ms(10);
	tight_loop_contents();
	i++;
	if (i > 1000){
	    rp2040_log("ERROR: test_max77958_interrupt timed out\n");
	}
    }
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    test_max77958_started = false;
    test_max77958_completed = false;
    set_interrupt_masks();
    test_max77958_interrupt_bool = false;
    rp2040_log("test_max77958_interrupt: PASSED after %" PRIu32 " milliseconds.\n", i*10);
}

static int32_t max77958_test_response(){
    test_max77958_completed = true;
    return 0;
}

void max77958_init(uint gpio_interrupt, queue_t* cq, queue_t* rq){
    _gpio_interrupt = gpio_interrupt;

    rp2040_log("max77958 init started\n");
    call_queue_ptr = cq;
    return_queue_ptr = rq;
    queue_init(&opcode_queue, sizeof(queue_entry_t), 16);

    test_max77958_get_id();

    set_interrupt_masks();
    
    // max77958 sends active LOW on INTB connected to GPIO7 on the rp2040. Setup interrupt callback here
    gpio_init(_gpio_interrupt);
    gpio_put(_gpio_interrupt, 0);
    gpio_set_dir(_gpio_interrupt, GPIO_IN);
    gpio_pull_up(_gpio_interrupt);
    gpio_set_irq_enabled(_gpio_interrupt, GPIO_IRQ_EDGE_FALL, true); 

    // clear interupts
    get_interrupt_vals();

    // Add all opcode commands in order to a queue. These will be called sequentially from core1 via the call_queue
    // Set GPIO5 and GPIO4 to LOW
    opcode_queue_add(gpio_set, gpio_bool_to_int32(false, false));
    // Set GPIO5 to HIGH and GPIO4 to LOW
    opcode_queue_add(gpio_set, gpio_bool_to_int32(false, true));
    opcode_queue_add(customer_config_write, 0);
    opcode_queue_add(cc_ctrl1_write_src_only, 0);
    opcode_queue_add(cc_ctrl1_read, 0);
    opcode_queue_add(swap_response_write, 0);
#ifdef MAX77958_FORCE_VBUS_DIAGNOSTIC
    opcode_queue_add(force_vbus_on_for_diagnostic, 0);
#endif
    //opcode_queue_add(set_snk_pdos, 0);
    //opcode_queue_add(set_src_pdos, 0);

    opcode_queue_pop();
#ifdef MAX77958_FORCE_VBUS_DIAGNOSTIC
    uint32_t waited_ms = 0;
    while (!opcodes_finished && waited_ms < 3000) {
        sleep_ms(10);
        waited_ms += 10;
    }
    if (opcodes_finished) {
        rp2040_log("MAX77958_DIAG: init opcode queue finished after %" PRIu32 "ms\n", waited_ms);
    } else {
        rp2040_log("MAX77958_DIAG: init opcode queue timed out after %" PRIu32 "ms\n", waited_ms);
    }
#endif
    rp2040_log("max77958 init finished\n");
    on_ccstat_change();

}

// check if opcode_queue has entries remaining
// if so remove an entry from the opcode_queue and run in on core1 via the call_queue
// return true if an entry was removed and added to the call_queue
// return false if opccode_queue was empty 
static bool opcode_queue_pop(){
    queue_entry_t entry;
    // if there is an entry in the opcode_queue, remove it and add it to the call_queue
    if (queue_try_remove(&opcode_queue, &entry)){
	rp2040_log("Removed entry from opcode_queue. %d entries remaining\n", queue_get_level(&opcode_queue));
	// if the call_queue is full, assert
	if(queue_try_add(call_queue_ptr, &entry)){
	    rp2040_log("added opcode entry to call_queue\n");
	    return true;
	}else{
	    rp2040_log("ERROR: opcode_queue_pop: call_queue full\n");
	    return false;
	}
    }
    // if there is no entry in the opcode_queue, return false
    else {
	rp2040_log("opcode_queue_pop: opcode_queue empty\n");
    	return false;
    }
}

static int32_t customer_config_write(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = 0x56; // Customer Configuration Write

    // Create configuration with readable boolean values
    max77958_customer_config_t config = {
        .dbg_src_enable = false,
        .dbg_snk_enable = false, 
        .audio_acc_enable = false,
        .trysnk_enable = false,
	.typec_mode = TYPEC_MODE_SRC,
        .mem_update_customer = false,  // Update RAM only
        .moisture_enable = false
    };

    // Convert config to register value using helper function
    send_buf[2] = max77958_build_customer_config_value(&config);
    rp2040_log("customer_config_write config = 0b"
	       BYTE_TO_BINARY_PATTERN "\n",
	       BYTE_TO_BINARY(send_buf[2]));

    send_buf[3] = 0x6A; // default VID
    send_buf[4] = 0x0B; // default VID
    send_buf[5] = 0x60; // default PID
    send_buf[6] = 0x68; // default PID
    send_buf[7] = 0x00; // RSVD
    send_buf[8] = 0x64; // default SRC_PDO_V
    send_buf[9] = 0x00; // default SRC_PDO_V of 5.0V (0x64= 100, and 50mA*100).
    send_buf[10] = 0x96; // SRC_PDO_MaxI
    send_buf[11] = 0x00; // SRC_PDO_MaxI = 1.5A (0x96=150, and 150*10mA)
    send_buf[21] = 0x69; // SID1 default
    send_buf[22] = 0x69; // SID2 default
    send_buf[23] = 0x35; // SID3 default
    send_buf[24] = 0x28; // SID4 default
    opcode_write(send_buf);
    return 0;
}

// Helper function to convert config struct to register value
uint8_t max77958_build_customer_config_value(const max77958_customer_config_t* config) {
    uint8_t val = 0;
    
    if (!config->dbg_src_enable) val |= DBG_SRC_DISABLE;
    else val |= DBG_SRC_ENABLE;
    
    if (!config->dbg_snk_enable) val |= DBG_SNK_DISABLE;
    else val |= DBG_SNK_ENABLE;
    
    if (!config->audio_acc_enable) val |= AUDIO_ACC_DISABLE;
    else val |= AUDIO_ACC_ENABLE;
    
    if (!config->trysnk_enable) val |= TRYSNK_DISABLE;
    else val |= TRYSNK_ENABLE;
    
    // Handle TypeC mode enum
    switch (config->typec_mode) {
        case TYPEC_MODE_SRC:
            val |= TYPEC_SRC;
            break;
        case TYPEC_MODE_SNK:
            val |= TYPEC_SNK;
            break;
        case TYPEC_MODE_DRP:
            val |= TYPEC_DRP;
            break;
        default:
            val |= TYPEC_SRC; // Default to source mode
            break;
    }
    
    if (!config->mem_update_customer) val |= MEM_UPDATE_RAM;
    else val |= MEM_UPDATE_CUSTOMER;
    
    if (!config->moisture_enable) val |= MOISTURE_DISABLE;
    else val |= MOISTURE_ENABLE;
    
    return val;
}

// A function to make turning on/off GPIO4 and 5 more readable
static int32_t gpio_bool_to_int32(bool _GPIO4, bool _GPIO5){
    //Reg 0x23 GPIO7Output,GPIO7Direction,GPIO6Output,GPIO6Direction,GPIO5Output,GPIO5Direction,GPIO4Output,GPIODirection
    return (_GPIO5 << 3) | (1 << 2) | (_GPIO4 << 1) | (1 << 0);
}

// A function to set the GPIO of the max77958 taking as input two bool values setting GPIO4 and GPIO5
static int32_t gpio_set(int32_t gpio_val){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_GPIO; 
    send_buf[2] = 0x00; //Reg 0x22 by default should be all 0s
    send_buf[3] = gpio_val;
    opcode_write(send_buf);
    return 0;
}

static void power_swap_request(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SWAP_REQ;
    send_buf[2] = MAX77958_SWAP_REQ_PR_SWAP;
    opcode_write(send_buf);
}

static int32_t swap_response_write(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SWAP_RESP;
    send_buf[2] = MAX77958_SWAP_RESP_SOURCE_UFP;
    rp2040_log("MAX77958_DIAG: setting swap response power=Source data=UFP config=0x%02x\n",
                send_buf[2]);
    opcode_write(send_buf);
    return 0;
}

static int32_t data_role_swap_request(void)
{
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SWAP_REQ;
    send_buf[2] = MAX77958_SWAP_REQ_DR_SWAP;
    rp2040_log("MAX77958_DIAG: requesting DR_SWAP to make robot UFP/device\n");
    opcode_write(send_buf);
    return 0;
}

static bool queue_data_role_swap_to_ufp_once(void)
{
    if (data_role_swap_requested) {
        return false;
    }

    uint8_t cc_status0 = max77958_read_register(REG_CC_STATUS0);
    uint8_t pd_status1 = max77958_read_register(REG_PD_STATUS1);
    bool source_attached = (cc_status0 & 0x07) == CC_STATUS0_STATE_SOURCE;
    bool dfp = (pd_status1 & PD_STATUS1_DATA_ROLE_DFP) != 0;
    bool psrdy = (pd_status1 & PD_STATUS1_PSRDY) != 0;

    rp2040_log("MAX77958_DIAG: DR_SWAP check CC0=0x%02x source=%u PD1=0x%02x data=%u psrdy=%u\n",
                cc_status0, source_attached ? 1 : 0, pd_status1, dfp ? 1 : 0, psrdy ? 1 : 0);

    if (!source_attached || !psrdy) {
        return false;
    }

    if (!dfp) {
        rp2040_log("MAX77958_DIAG: data role already UFP/device\n");
        data_role_swap_requested = true;
        return false;
    }

    data_role_swap_requested = true;
    opcode_queue_add(data_role_swap_request, 0);
    return true;
}

static int32_t set_src_pdos(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SET_SOURCE_CAP;
    send_buf[2] = 0b00000001; // specify only 1 PDO
    // You can verify this by comparing against Table 6-9 of the USB-PD spec 
    // This represents:
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 1 Dual-Role Power ON
    // 0 Suspend Supported OFF
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 1 Dual-Role Data ON
    // 0 Unchunked Messages OFF
    // 0 ERP Mode Incable
    // 0 RSVD
    // 00  Peak current
    // 0001100100 = 100x 50mV = 5.0V
    // 0000101100 = 100x 10mA = 1.0A
    // In summary 00100110000000011001000000101100 or 0x2601902C
    send_buf[3] = 0x2C;
    send_buf[4] = 0x90;
    send_buf[5] = 0x01;
    send_buf[6] = 0x26;
    opcode_write(send_buf);
    return 0;
}
static int32_t set_snk_pdos(){
    memset(send_buf, 0, sizeof send_buf);
    send_buf[0] = OPCODE_WRITE;
    send_buf[1] = OPCODE_SNK_PDO_SET; 
    send_buf[2] = 0b00000001; // Write to RAM only and specify only 1 PDO 
    // Next four are specified by Analog Support via 0x1401912C LSB first
    // You can verify this by comparing against Table 6-16 of the USB-PD spec
    // I convert this to binary 00010100000000011001000100101100
    // This represents:
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 0 Dual-Role Power Off
    // 1 Higher Capability ON
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 0 Dual-Role Data OFF
    // 00 Fast Role Swap Not Supported
    // 000 RSVD
    // 0001100100 = 100x 50mV = 5.0V
    // 0100101100 = 300x 10mA = 3.0A
    // I then want to update this to 
    // 00 Fixed Supply See Table 6-7 of USB-PD Standard
    // 1 Dual-Role Power ON
    // 0 Higher Capability OFF
    // 0 Unconstrained Power OFF
    // 1 USB Communications Capable ON
    // 1 Dual-Role Data ON
    // 01 Fast Role Swap Default USB Power (can update to 10b for 1.5A@5V but not sure if this is supported by chips)
    // 000 RSVD
    // 0001100100 = 100x 50mV = 5.0V
    // 0100101100 = 300x 10mA = 3.0A
    // In summary 00100110100000011001000100101100 or 0x2681912C
    send_buf[3] = 0x2C;
    send_buf[4] = 0x91;
    send_buf[5] = 0x81;
    send_buf[6] = 0x26;
    // default values 
    //send_buf[3] = 0x2C;
    //send_buf[4] = 0x91;
    //send_buf[5] = 0x01;
    //send_buf[6] = 0x14;    
    //Trying with MSB first (didn't work)
    //send_buf[3] = 0x14;    
    //send_buf[4] = 0x01;
    //send_buf[5] = 0x91;
    //send_buf[6] = 0x2C;
    opcode_write(send_buf);
    return 0;
}

static void vbus_turn_off(){
#ifdef MAX77958_FORCE_VBUS_DIAGNOSTIC
    rp2040_log("MAX77958_DIAG: forced VBUS mode ignoring vbus_turn_off\n");
    return;
#endif
    data_role_swap_requested = false;
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(false, true));
    opcode_queue_pop();
}

#ifdef MAX77958_FORCE_VBUS_DIAGNOSTIC
static int32_t force_vbus_on_for_diagnostic(void)
{
    rp2040_log("MAX77958_DIAG: forcing GPIO4/GPIO5 high for VBUS diagnostic\n");
    return gpio_set(gpio_bool_to_int32(true, true));
}
#endif

static void vbus_turn_on(){
    opcode_queue_add(&gpio_set, gpio_bool_to_int32(true, true));
    queue_data_role_swap_to_ufp_once();
    opcode_queue_pop();
}

static int32_t pd_msg_response(){
    // Read the 0xE PD_STATUS0 register as it contains the PD message Type recieved 
    memset(send_buf, 0, sizeof send_buf);
    memset(return_buf, 0, sizeof return_buf);
    send_buf[0] = REG_PD_STATUS0; // 0xE PD_STATUS0 Register 
    i2c_write_error_handling(i2c0, MAX77958_SLAVE_P1, send_buf, 1, true);
    i2c_read_error_handling(i2c0, MAX77958_SLAVE_P1, return_buf, 1, false);
    rp2040_log("PD_STATUS0: 0x%02x\n", return_buf[0]);
    switch (return_buf[0]){
        case PDMSG_DR_SWAP_REQ_RECEIVED:
	    rp2040_log("PD Message: DR_SWAP_REQ_RECEIVED\n");
	    break;
        case PDMSG_REJECT_RECEIVED:
	    rp2040_log("PD Message: REJECT_RECEIVED\n");
	    break;
        case PDMSG_PRSWAP_SRCTOSWAP:
	    rp2040_log("PD Message: PRSWAP_SRCTOSWAP\n");
	    break;
	case PDMSG_PRSWAP_SWAPTOSNK:
	    rp2040_log("PD Message: PRSWAP_SWAPTOSNK\n");
	    vbus_turn_off();
	    break;
	case PDMSG_PRSWAP_SNKTOSWAP:
	    rp2040_log("PD Message: PRSWAP_SNKTOSWAP\n");
	    break;
	case PDMSG_PRSWAP_SWAPTOSRC:
	    rp2040_log("PD Message: PRSWAP_SWAPTOSRC\n");
	    vbus_turn_on();
	    break;
	case PDMSG_POWER_SUPPLY_VBUS_ENABLE:
	    rp2040_log("PD Message: PowerSupply VbusEnable\n");
	    vbus_turn_on();
	    break;
	case PDMSG_POWER_SUPPLY_VBUS_DISABLE:
	    rp2040_log("PD Message: PowerSupply VbusDisable\n");
	    vbus_turn_off();
	    break;
	case PDMSG_VDM_NAK_RECEIVED:
	    rp2040_log("PD Message: VDM_NAK Received\n");
	case PDMSG_VDM_BUSY_RECEIVED:
	    rp2040_log("PD Message: VDM_BUSY_RECEIVED\n");
	case PDMSG_VDM_ACK_RECEIVED:
	    rp2040_log("PD Message: VDM_ACK_RECEIVED\n");
	case PDMSG_VDM_REQ_RECEIVED:
	    rp2040_log("PD Message: VDM_REQ_RECEIVED\n");
	    break;
	default:
	    rp2040_log("PD Message: Unknown\n");
	    break;
	}	
    return 0;
}

void max77958_shutdown(uint gpio_interrupt){
    opcode_queue_add(gpio_set, gpio_bool_to_int32(false, false));
    opcode_queue_pop();
    int i = 0;
    while (!opcodes_finished){
	sleep_ms(100);
	i++;
	if (i > 10){
	    rp2040_log("ERROR: Timed out waiting for GPIO to finish\n");
	}
    }
}
