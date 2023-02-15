#ifndef BIT_OPS_
#define BIT_OPS_

#include "pico/stdlib.h"

uint8_t bit_set_range(uint8_t bitvector, uint8_t lsb, uint8_t msb, uint8_t new_val);

uint8_t bit_assign(uint8_t bitvector, uint8_t new_value, uint8_t idx);

uint8_t bit_set(uint8_t bitvector, uint8_t idx);

uint8_t bit_clear(uint8_t bitvector, uint8_t idx);

uint8_t bit_flip(uint8_t bitvector, uint8_t idx);

uint8_t bit_check(uint8_t bitvector, uint8_t idx);

uint8_t bitmask_from_x_2_y(uint8_t lsb, uint8_t msb);

uint8_t bitmask_set(uint8_t bitvector, uint8_t mask);

uint8_t bitmask_clear(uint8_t bitvector, uint8_t mask);

uint8_t bitmask_flip(uint8_t bitvector, uint8_t mask);

uint8_t bitmask_check_all(uint8_t bitvector, uint8_t mask);

uint8_t bitmask_check_any(uint8_t bitvector, uint8_t mask);

#endif