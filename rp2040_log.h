#ifndef RP2040_LOG_
#define RP2040_LOG_

#include "pico/types.h"

#define LOG_BUFFER_SIZE 1024

// Circular buffer structure
typedef struct {
    char buffer[LOG_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t byte_count;
    uint16_t new_line_count;
}CircularBufferLog ;

void rp2040_log_init();
void rp2040_log(const char *format, ...);
void rp2040_log_flush();
uint16_t rp2040_get_byte_count();
void rp2040_orient_copy_buffer(char* output_array);

#endif
