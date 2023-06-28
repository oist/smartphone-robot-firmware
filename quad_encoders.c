#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"

volatile int32_t encoder_count[ENCODER_COUNT] = {0};
void encoderA_int_handler(uint gpio, uint32_t events);
void encoderB_int_handler(uint gpio, uint32_t events);
// Encoder pins
const uint8_t encoderPins[ENCODER_COUNT][2] = {
    {12, 13},   // Encoder 1 channel A, B
    {14, 15}    // Encoder 2 channel A, B
};


// Initialize quadrature encoder pins and interrupts
void encoder_init() {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        gpio_init(encoderPins[i][0]);
        gpio_init(encoderPins[i][1]);
        gpio_set_dir(encoderPins[i][0], GPIO_IN);
        gpio_set_dir(encoderPins[i][1], GPIO_IN);
        gpio_pull_up(encoderPins[i][0]);
        gpio_pull_up(encoderPins[i][1]);
        gpio_set_irq_enabled_with_callback(encoderPins[i][0], GPIO_IRQ_EDGE_FALL, true, &encoderA_int_handler);
        gpio_set_irq_enabled_with_callback(encoderPins[i][1], GPIO_IRQ_EDGE_FALL, true, &encoderB_int_handler);
    }
}

// Interrupt handler for encoder channel A
void encoderA_int_handler(uint gpio, uint32_t events) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        if (gpio == encoderPins[i][0]) {
            uint32_t channelB = gpio_get(encoderPins[i][1]);
            if (channelB) {
                encoder_count[i]++;
            } else {
                encoder_count[i]--;
            }
            break;
        }
    }
}

// Interrupt handler for encoder channel B
void encoderB_int_handler(uint gpio, uint32_t events) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        if (gpio == encoderPins[i][1]) {
            uint32_t channelA = gpio_get(encoderPins[i][0]);
            if (channelA) {
                encoder_count[i]--;
            } else {
                encoder_count[i]++;
            }
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
