#ifndef MAX77958_
#define MAX77958_

#include "pico/types.h"
#include "pico/util/queue.h"

#define FPF1048BUCX_EN _u(4) // GPIO4
#define TPS61253_EN _u(5) // GPIO5

// gpio_interrupt being the gpio pin on the mcu attached to the INTB pin of max77958
void max77958_init(uint gpio_interrupt, queue_t* call_queue, queue_t* results_queue);
void max77958_shutdown(uint gpio_interrupt);
void max77958_on_interrupt(uint gpio, uint32_t event_mask);
void test_max77958_get_id();
void test_max77958_get_customer_config_id();
void test_max77958_cc_ctrl1_read();
void test_max77958_bc_ctrl1_read();
void test_max77958_interrupt();
void read_reg(uint8_t reg);

#endif
