#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"
#include "robot.h"
#include <stdio.h>

volatile int32_t encoder_count[ENCODER_COUNT] = {0};
void encoder_interrupt_handler(uint gpio);
// Encoder pins
const uint8_t encoder_pins[ENCODER_COUNT][2] = {
    {12, 13},   // Encoder 1 channel A, B
    {14, 15}    // Encoder 2 channel A, B
};
static queue_t* call_queue_ptr;
static void on_edge_fall(uint gpio, uint32_t events);

// Initialize quadrature encoder pins and interrupts
void encoder_init(queue_t* call_queue) {
    call_queue_ptr = call_queue;

    for (int i = 0; i < ENCODER_COUNT; i++) {
        gpio_init(encoder_pins[i][0]);
        gpio_init(encoder_pins[i][1]);
        gpio_set_dir(encoder_pins[i][0], GPIO_IN);
        gpio_set_dir(encoder_pins[i][1], GPIO_IN);
        gpio_pull_up(encoder_pins[i][0]);
        gpio_pull_up(encoder_pins[i][1]);
        gpio_set_irq_enabled_with_callback(encoder_pins[i][0], GPIO_IRQ_EDGE_FALL, true, &on_edge_fall);
        gpio_set_irq_enabled_with_callback(encoder_pins[i][1], GPIO_IRQ_EDGE_FALL, true, &on_edge_fall);
    }
}

static void on_edge_fall(uint gpio, uint32_t events){
    queue_entry_t on_edge_fall_entry = {&encoder_interrupt_handler, gpio};
    if (!queue_try_add(call_queue_ptr, &on_edge_fall_entry)){
        printf("call_queue is full");
        assert(false);
    }
}

void encoder_interrupt_handler(uint gpio) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        if (gpio == encoder_pins[i][0] || gpio == encoder_pins[i][1]) {
            uint32_t channel_a = gpio_get(encoder_pins[i][0]);
            uint32_t channel_b = gpio_get(encoder_pins[i][1]);
            int32_t increment = (channel_a == channel_b) ? 1 : -1;
            encoder_count[i] += increment;
            break;
        }
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
