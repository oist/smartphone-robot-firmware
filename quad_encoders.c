#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"
#include "robot.h"
#include <stdio.h>

volatile int32_t encoder_count[ENCODER_COUNT] = {0};
static queue_t* call_queue_ptr;
static int gpio;
static bool channel_a;
static bool channel_b;
static uint8_t increment;

static void on_edge_fall();
static void encoder_interrupt_handler() ;

// Initialize quadrature encoder pins and interrupts
void encoder_init(queue_t* call_queue) {
    call_queue_ptr = call_queue;

    gpio_init(ENCODER_1_CHANNEL_A);
    gpio_init(ENCODER_1_CHANNEL_B);
    gpio_init(ENCODER_2_CHANNEL_A);
    gpio_init(ENCODER_2_CHANNEL_B);
    gpio_set_dir(ENCODER_1_CHANNEL_A, GPIO_IN);
    gpio_set_dir(ENCODER_1_CHANNEL_B, GPIO_IN);
    gpio_set_dir(ENCODER_2_CHANNEL_A, GPIO_IN);
    gpio_set_dir(ENCODER_2_CHANNEL_B, GPIO_IN);
    gpio_pull_up(ENCODER_1_CHANNEL_A);
    gpio_pull_up(ENCODER_1_CHANNEL_B);
    gpio_pull_up(ENCODER_2_CHANNEL_A);
    gpio_pull_up(ENCODER_2_CHANNEL_B);
    gpio_add_raw_irq_handler_masked(ENCODER_1_CHANNEL_A | ENCODER_2_CHANNEL_A, &on_edge_fall);
    gpio_set_irq_enabled(ENCODER_1_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_2_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true);
}

static void on_edge_fall(){
    if (gpio_get_irq_event_mask(ENCODER_1_CHANNEL_A) & GPIO_IRQ_EDGE_FALL){
	gpio_acknowledge_irq(ENCODER_1_CHANNEL_A, GPIO_IRQ_EDGE_FALL);
	gpio = ENCODER_1_CHANNEL_A;
	}
    if (gpio_get_irq_event_mask(ENCODER_2_CHANNEL_A) & GPIO_IRQ_EDGE_FALL){
	gpio_acknowledge_irq(ENCODER_2_CHANNEL_A, GPIO_IRQ_EDGE_FALL);
	gpio = ENCODER_2_CHANNEL_A;
	}
    queue_entry_t on_edge_fall_entry = {&encoder_interrupt_handler, gpio};
    if (!queue_try_add(call_queue_ptr, &on_edge_fall_entry)){
        printf("call_queue is full");
        assert(false);
    }
}

static void encoder_interrupt_handler() {
    //printf("encoder int on gpio %d\n", gpio);
    channel_a = gpio_get(gpio);
    channel_b = gpio_get(gpio + 1);
    increment = (channel_a == channel_b) ? 1 : -1;
    if (gpio == ENCODER_1_CHANNEL_A) {
	encoder_count[0] += increment;
	printf("A encoder count: %d\n", (int) encoder_count[0]);
    } else if (gpio == ENCODER_2_CHANNEL_A) {
	encoder_count[1] += increment;
	printf("B encoder count: %d\n", (int) encoder_count[1]);
    }
}

// Function to get the value of the encoder count for a specific encoder
int32_t get_encoder_count(int encoder) {
    if (encoder < 0 || encoder >= ENCODER_COUNT) {
        // Invalid encoder index, return 0 or handle the error as needed
        return 0;
    }

    return encoder_count[encoder];
}

// Function to set the value of the encoder count for a specific encoder
void set_encoder_count(int encoder, int32_t value) {
    if (encoder < 0 || encoder >= ENCODER_COUNT) {
        // Invalid encoder index, do nothing or handle the error as needed
        return;
    }

    encoder_count[encoder] = value;
}
