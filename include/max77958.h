#ifndef MAX77958_
#define MAX77958_

#include "pico/types.h"
#include "pico/util/queue.h"

#define FPF1048BUCX_EN _u(4) // GPIO4
#define TPS61253_EN _u(5) // GPIO5

#define DBG_SRC_DISABLE      (0 << 0)
#define DBG_SRC_ENABLE       (1 << 0)

#define DBG_SNK_DISABLE      (0 << 1)
#define DBG_SNK_ENABLE       (1 << 1)

#define AUDIO_ACC_DISABLE    (0 << 2)
#define AUDIO_ACC_ENABLE     (1 << 2)

#define TRYSNK_DISABLE       (0 << 3)
#define TRYSNK_ENABLE        (1 << 3)

// TypeC_State (bits 5:4)
#define TYPEC_SRC            (0 << 4)  // 00b
#define TYPEC_SNK            (1 << 4)  // 01b
#define TYPEC_DRP            (2 << 4)  // 10b

#define MEM_UPDATE_RAM       (0 << 6)
#define MEM_UPDATE_CUSTOMER  (1 << 6)

#define MOISTURE_DISABLE     (0 << 7)
#define MOISTURE_ENABLE      (1 << 7)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

// TypeC state enumeration for better readability
typedef enum {
    TYPEC_MODE_SRC = 0,  // Source mode
    TYPEC_MODE_SNK = 1,  // Sink mode  
    TYPEC_MODE_DRP = 2   // Dual Role Power mode
} typec_mode_t;

// Configuration structure for customer config with readable boolean fields
typedef struct {
    bool dbg_src_enable;        // Enable debug source
    bool dbg_snk_enable;        // Enable debug sink
    bool audio_acc_enable;      // Enable audio accessory
    bool trysnk_enable;         // Enable try sink
    typec_mode_t typec_mode;    // TypeC operation mode
    bool mem_update_customer;   // Update customer memory (vs RAM only)
    bool moisture_enable;       // Enable moisture detection
} max77958_customer_config_t;

// Helper function to convert config struct to register value
uint8_t max77958_build_customer_config_value(const max77958_customer_config_t* config);

// gpio_interrupt being the gpio pin on the mcu attached to the INTB pin of max77958
void max77958_init(uint gpio_interrupt, queue_t* call_queue, queue_t* results_queue);
void max77958_shutdown(uint gpio_interrupt);
void max77958_on_interrupt(uint gpio, uint32_t event_mask);
void max77958_on_start_complete(void);
void test_max77958_status_block_read_all(void);
void test_max77958_get_id();
void test_max77958_bc_ctrl1_read();
void test_max77958_bc_ctrl2_read(void);
void test_max77958_control1_read(void);
void test_max77958_cc_ctrl1_read();
void test_max77958_cc_ctrl4_read(void);
void test_max77958_gpio_control_read(void);
void test_max77958_gpio0_gpio1_adc_read(void);
void test_max77958_snk_pdo_request(void);
void test_max77958_get_customer_config();
void test_max77958_interrupt();
void read_reg(uint8_t reg);

#endif
