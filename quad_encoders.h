#ifndef QUAD_ENCODERS_
#define QUAD_ENCODERS_

#include "pico/types.h"

void encoder_init();
int32_t quad_encoder_get_count(uint encoder);
void quad_encoder_update();

#endif
