#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "quad_encoders.h"
#include "robot.h"
#include <stdio.h>
#include <inttypes.h>
#include "quadrature_encoder.pio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"

const PIO pio = pio0;
const uint sm0 = 0;
const uint sm1 = 1;

static void quad_encoders_interrupt_handler(uint gpio) ;

typedef struct Encoder
{
    uint32_t count_current;
    uint32_t count_previous;
    uint32_t count_delta;
    uint32_t speed_current;
    uint32_t speed_previous;
    uint32_t speed_delta;
    uint32_t speed_average;
} encoder;

encoder encoders[2];

// Initialize quadrature encoder pins and interrupts
void encoder_init() {
    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm0, offset, ENCODER_1_CHANNEL_A, 0);
    quadrature_encoder_program_init(pio, sm1, offset, ENCODER_2_CHANNEL_A, 0);
}

// Get the current encoder count from PIO
int32_t quad_encoder_get_count(uint encoder){
    // Although the next function actually takes sm as an argument, we are using
    // encoder with value of 0 or 1 to represent sm0 or sm1.
    // I thought this was more clear than trying to explain what a state machine is
    // to the average user. 
    return quadrature_encoder_get_count(pio, encoder);
}

void quad_encoder_update(){
    for (uint encoder = 0; encoder < 2; encoder++){
	encoders[encoder].count_current = quadrature_encoder_get_count(pio, encoder);
	encoders[encoder].count_delta = encoders[encoder].count_current - encoders[encoder].count_previous;
	encoders[encoder].count_previous = encoders[encoder].count_current;
        printf("Encoder %d: %8d, delta: %6d\n", encoder, encoders[encoder].count_current, encoders[encoder].count_delta);
    }
}
