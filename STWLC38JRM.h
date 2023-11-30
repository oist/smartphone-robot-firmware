#ifndef STWLC38JRM
#define STWLC38JRM

#define STWLC38_ADDR _u(0x61)

void STWLC38JRM_init(uint enable_pin);
void STWLC38JRM_shutdown(uint enable_pin);
void STWLC38_get_ept_reasons();

#endif
