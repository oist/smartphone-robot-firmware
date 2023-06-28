#ifndef QUAD_ENCODERS_
#define QUAD_ENCODERS_

// GPIO pins on rp2040
#include <stdint.h>
#define ENC_11 _u(12)
#define ENC_12 _u(13)
#define ENC_21 _u(14)
#define ENC_22 _u(15)

#define ENCODER_COUNT 2

void encoder_init();
int32_t get_encoder_count(int encoder);
void set_encoder_count(int encoder, int32_t value);

#endif
