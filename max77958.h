#ifndef MAX77958_
#define MAX77958_

#include "pico/types.h"
#include "pico/util/queue.h"
#include "robot.h"

#define FPF1048BUCX_EN _u(4) // GPIO4
#define TPS61253_EN _u(5) // GPIO5

// gpio_interrupt being the gpio pin on the mcu attached to the INTB pin of max77958
void max77958_init(uint gpio_interrupt, queue_t* call_queue, queue_t* results_queue);

static void parse_interrupt_vals();
static void on_interrupt(uint gpio, long unsigned int events);
static void get_interrupt_vals();
static void get_interrupt_masks();
static void set_interrupt_masks();
static int opcode_read();
static int opcode_write(uint8_t *send_buf, uint8_t len);
static queue_t* call_queue_ptr;
static queue_t* return_queue_ptr;
static queue_entry_t parse_interrupt_vals_entry = {&parse_interrupt_vals, 0};
static queue_entry_t opcode_read_entry = {&opcode_read, 0};

#endif
