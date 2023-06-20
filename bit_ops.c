#include "bit_ops.h"
#include "pico/stdlib.h"

/* Takes an 8-bit bitvector and assigns new_value (0 or 1) to index idx
and returns the modified bitvector.

 !!new_value ensures only 0 or 1 gets through (force boolean)
 */
uint8_t bit_assign(uint8_t bitvector, uint8_t new_value, uint8_t idx) {
    return (bitvector & ~((uint8_t)1 << idx)) | (!!new_value << idx);
}
/* bitvector=target variable, idx=bit index to act upon*/
uint8_t bit_set(uint8_t bitvector, uint8_t idx) {return bitvector | ((uint8_t)1 << idx);}

uint8_t bit_clear(uint8_t bitvector, uint8_t idx) {return bitvector & ~((uint8_t)1 << idx);}

uint8_t bit_flip(uint8_t bitvector, uint8_t idx) {return bitvector ^ (uint8_t)1 << idx;}

uint8_t bit_check(uint8_t bitvector, uint8_t idx) {return !!(bitvector & ((uint8_t)1 << idx));} // '!!' to make sure this returns 0 or 1

/* generates mask from lsb to msb, with lsb being the least significant bit to be set to 1
and msb being the most significant bit set to be 1 in the mask.
e.g. bitmask_from_x_2_y(2,3) will return 0b00001100
*/
uint8_t bitmask_from_x_2_y(uint8_t lsb, uint8_t msb){
    return ((((uint8_t)1 << (msb - lsb + (uint8_t)1)) - (uint8_t)1) << (lsb));
}

uint8_t bitmask_set(uint8_t bitvector, uint8_t mask){return bitvector | mask;}
uint8_t bitmask_clear(uint8_t bitvector, uint8_t mask){return bitvector & ~mask;}
uint8_t bitmask_flip(uint8_t bitvector, uint8_t mask){return bitvector ^= mask;}
uint8_t bitmask_check_all(uint8_t bitvector, uint8_t mask){return !(~bitvector & mask);}
uint8_t bitmask_check_any(uint8_t bitvector, uint8_t mask){return bitvector & mask;}

/* generates mask from lsb to msb, using bitmask_from_x_2_y, then clears those values
from bitvector, replacing them with new_val
*/
uint8_t bit_set_range(uint8_t bitvector, uint8_t lsb, uint8_t msb, uint8_t new_val){
    uint8_t mask = bitmask_from_x_2_y(lsb, msb);
    bitvector = bitmask_clear(bitvector, mask);
    return bitvector | ((new_val << lsb) & mask); // the & mask ensures even if new_val has size differernt from mask, only those in mask are overwritten
}
