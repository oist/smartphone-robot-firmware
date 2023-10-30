#include <stdio.h>
#include "pico/types.h"
#include <string.h>

#define LOG_BUFFER_SIZE 1024

// Circular buffer structure
typedef struct {
    char buffer[LOG_BUFFER_SIZE];
    int head;
    int tail;
}CircularBufferLog ;

static CircularBufferLog log_buffer;

// Initialize the circular buffer
void rp2040_log_init() {
    log_buffer.head = 0;
    log_buffer.tail = 0;
}

void rp2040_log(const char* message) {
    int len = strlen(message);
    int space_available = LOG_BUFFER_SIZE - (log_buffer.tail - log_buffer.head);

    // Check if the message fits in the buffer
    if (len >= space_available) {
        // Handle buffer overflow by wrapping around
        int space_to_free = len - space_available;

        // Wrap around the head and tail pointers
        log_buffer.head = (log_buffer.head + space_to_free) % LOG_BUFFER_SIZE;
        log_buffer.tail = (log_buffer.tail + space_to_free) % LOG_BUFFER_SIZE;
    }

    // Copy the message to the buffer
    strcpy(log_buffer.buffer + log_buffer.tail, message);
    log_buffer.tail = (log_buffer.tail + len) % LOG_BUFFER_SIZE;
}

void rp2040_log_flush(){
    printf("%s", log_buffer.buffer);
}

