#ifndef RP2040_LOG_
#define RP2040_LOG_

#include "pico/types.h"

#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARNING 3
#define LOG_LEVEL_ERROR 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_BUFFER_LINE_COUNT 1028
#define LOG_BUFFER_CHAR_LIMIT 128

// Circular buffer structure
typedef struct {
    // create an arrary of character arrays. Each with variable size
    char log_array[LOG_BUFFER_LINE_COUNT][LOG_BUFFER_CHAR_LIMIT];
    // create an array storing the variable size of each line in the log_array
    uint16_t log_array_line_size[LOG_BUFFER_LINE_COUNT];
    // number of populated entries currently stored in the ring
    uint16_t count;
    uint16_t head;
    uint16_t tail;
    volatile bool lock; // Added a lock variable
}CircularBufferLog ;

void rp2040_log_init();
void rp2040_log(int level, const char *format, ...);
void rp2040_log_flush();
uint16_t rp2040_get_byte_count();
void rp2040_orient_copy_buffer(char* output_array);
void rp2040_log_acquire_lock();
void rp2040_log_release_lock();

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define rp2040_log_d(...) rp2040_log(LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define rp2040_log_d(...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#define rp2040_log_i(...) rp2040_log(LOG_LEVEL_INFO, __VA_ARGS__)
#else
#define rp2040_log_i(...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARNING
#define rp2040_log_w(...) rp2040_log(LOG_LEVEL_WARNING, __VA_ARGS__)
#else
#define rp2040_log_w(...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define rp2040_log_e(...) rp2040_log(LOG_LEVEL_ERROR, __VA_ARGS__)
#else
#define rp2040_log_e(...) ((void)0)
#endif

#endif
