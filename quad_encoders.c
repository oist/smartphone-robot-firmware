#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"
#include "robot.h"
#include <stdio.h>
#include <inttypes.h>

int32_t encoder_count[ENCODER_COUNT] = {0};
static queue_t* call_queue_ptr;
static int gpio;
static bool channel_a;
static bool channel_b;
static bool increment;

static void quad_encoders_interrupt_handler(uint gpio) ;

// Initialize quadrature encoder pins and interrupts
void encoder_init(queue_t* call_queue) {
    call_queue_ptr = call_queue;

    for (int i = 0; i < ENCODER_COUNT; i++) {
        encoder_count[i] = 0;
    }

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
    gpio_set_irq_enabled(ENCODER_1_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_2_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true);
}

void quad_encoders_on_interrupt(uint gpio, uint32_t events) {
    if ((gpio == ENCODER_1_CHANNEL_A) && (events & GPIO_IRQ_EDGE_FALL)){
	gpio_acknowledge_irq(ENCODER_1_CHANNEL_A, GPIO_IRQ_EDGE_FALL);
	}
    if ((gpio == ENCODER_2_CHANNEL_A) && (events & GPIO_IRQ_EDGE_FALL)){
	gpio_acknowledge_irq(ENCODER_2_CHANNEL_A, GPIO_IRQ_EDGE_FALL);
	}
    call_queue_try_add(&quad_encoders_interrupt_handler, gpio);
}

static void quad_encoders_interrupt_handler(uint gpio) {
    //printf("encoder int on gpio %d\n", gpio);
    channel_a = gpio_get(gpio);
    channel_b = gpio_get(gpio + 1);
    if (gpio == ENCODER_1_CHANNEL_A) {
	//printf("A encoder count prior to increment: %" PRId32 "\n", encoder_count[0]);
        if (channel_a == channel_b){
	    encoder_count[0]++;
	}else{
	    encoder_count[0]--;
	}
	printf("A encoder count post increment: %" PRId32 "\n", encoder_count[0]);
    } else if (gpio == ENCODER_2_CHANNEL_A) {
	//printf("B encoder count prior to increment: %" PRId32 "\n", encoder_count[1]);
        if (channel_a == channel_b){
	    encoder_count[1]++;
	}else{
	    encoder_count[1]--;
	}
	printf("B encoder count post increment: %" PRId32 "\n", encoder_count[1]);
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
