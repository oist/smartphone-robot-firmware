#ifndef STWLC38JRM
#define STWLC38JRM

#include "pico/stdlib.h"

#define STWLC38_ADDR _u(0x61)

void STWLC38JRM_init(uint enable_pin, uint vrect_pin);
void STWLC38JRM_shutdown(uint enable_pin);
void STWLC38_get_ept_reasons();
uint16_t STWLC38JRM_adc1();

#endif
