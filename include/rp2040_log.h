#ifndef RP2040_LOG_
#define RP2040_LOG_

#include "pico/types.h"

#define LOG_BUFFER_LINE_COUNT 1028
#define LOG_BUFFER_CHAR_LIMIT 128

// Circular buffer structure
typedef struct {
    // create an arrary of character arrays. Each with variable size
    char log_array[LOG_BUFFER_LINE_COUNT][LOG_BUFFER_CHAR_LIMIT];
    // create an array storing the variable size of each line in the log_array
    uint16_t log_array_line_size[LOG_BUFFER_LINE_COUNT];
    uint16_t head;
    uint16_t tail;
    volatile bool lock; // Added a lock variable
}CircularBufferLog ;

void rp2040_log_init();
void rp2040_log(const char *format, ...);
void rp2040_log_flush();
uint16_t rp2040_get_byte_count();
void rp2040_orient_copy_buffer(char* output_array);
void rp2040_log_acquire_lock();
void rp2040_log_release_lock();

#endif
