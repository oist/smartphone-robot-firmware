#ifndef QUAD_ENCODERS_
#define QUAD_ENCODERS_

#include "pico/types.h"
#include "pico/util/queue.h"

// GPIO pins on rp2040
#include <stdint.h>
#define ENC_11 _u(12)
#define ENC_12 _u(13)
#define ENC_21 _u(14)
#define ENC_22 _u(15)

#define ENCODER_COUNT 2

void encoder_init(queue_t* call_queue);
int32_t get_encoder_count(int encoder);
void set_encoder_count(int encoder, int32_t value);
void quad_encoders_on_interrupt(uint gpio, uint32_t events) ;

#endif
