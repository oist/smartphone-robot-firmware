#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"
#include "robot.h"
#include <stdio.h>

volatile int32_t encoder_count[ENCODER_COUNT] = {0};
void encoder_interrupt_handler(uint gpio);
// Encoder pins
const uint8_t ENCODER_1_CHANNEL_A = 12;
const uint8_t ENCODER_1_CHANNEL_B = 13;
const uint8_t ENCODER_2_CHANNEL_A = 14;
const uint8_t ENCODER_2_CHANNEL_B = 15;

static queue_t* call_queue_ptr;
static bool encoder_1_channel_a;
static bool encoder_1_channel_b;
static int8_t encoder_1_increment;
static bool encoder_2_channel_a;
static bool encoder_2_channel_b;
static int8_t encoder_2_increment;

static void on_edge_fall_1(uint gpio, uint32_t events);
static void on_edge_fall_2(uint gpio, uint32_t events);
static void encoder_interrupt_handler_1(uint gpio) ;
static void encoder_interrupt_handler_2(uint gpio) ;

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
    gpio_set_irq_enabled_with_callback(ENCODER_2_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true, &on_edge_fall_2);
    sleep_ms(1000);
    gpio_set_irq_enabled_with_callback(ENCODER_1_CHANNEL_A, GPIO_IRQ_EDGE_FALL, true, &on_edge_fall_1);
}

static void on_edge_fall_1(uint gpio, uint32_t events){
    printf("on_edge_fall_1 called\n");
    queue_entry_t on_edge_fall_entry_1 = {&encoder_interrupt_handler_1, gpio};
    if (!queue_try_add(call_queue_ptr, &on_edge_fall_entry_1)){
        printf("call_queue is full");
        assert(false);
    }
}

static void on_edge_fall_2(uint gpio, uint32_t events){
    printf("on_edge_fall_2 called\n");
    queue_entry_t on_edge_fall_entry_2 = {&encoder_interrupt_handler_2, gpio};
    if (!queue_try_add(call_queue_ptr, &on_edge_fall_entry_2)){
        printf("call_queue is full");
        assert(false);
    }
}

static void encoder_interrupt_handler_1(uint gpio) {
    //printf("encoder int on gpio %d\n", gpio);
    encoder_1_channel_a = gpio_get(gpio);
    encoder_1_channel_b = gpio_get(gpio + 1);
    encoder_1_increment = (encoder_1_channel_a == encoder_1_channel_b) ? 1 : -1;
    encoder_count[0] += encoder_1_increment;
    printf("A encoder count: %d\n", (int) encoder_count[0]);
}

static void encoder_interrupt_handler_2(uint gpio) {
    printf("encoder int on gpio %d\n", gpio);
    encoder_2_channel_a = gpio_get(gpio);
    encoder_2_channel_b = gpio_get(gpio + 1);
    encoder_2_increment = (encoder_2_channel_a == encoder_2_channel_b) ? 1 : -1;
    encoder_count[1] += encoder_2_increment;
    printf("B encoder count: %d\n", (int) encoder_count[1]);
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
