#include <stdarg.h>
#include <stdio.h>
#include "pico/types.h"
#include <string.h>
#include "rp2040_log.h"

static CircularBufferLog log_buffer;

// Initialize the circular buffer
void rp2040_log_init() {
    log_buffer.head = 0;
    log_buffer.tail = 0;
    log_buffer.byte_count = 0;
    log_buffer.new_line_count = 0; // tracks the number of lines so can account fo \r being sent everytime a \n is sent
}

void rp2040_log(const char* format, ...) {
    va_list args;
    va_start(args, format);

    // Calculate the number of characters required
    int len = vsnprintf(NULL, 0, format, args) + 1; // +1 for the null terminator

    va_end(args);

    int space_available = LOG_BUFFER_SIZE - (log_buffer.tail - log_buffer.head);

    // Check if the formatted message fits in the buffer
    if (len >= space_available) {
        // Handle buffer overflow by wrapping around
        int space_to_free = len - space_available;

        // Wrap around the head and tail pointers
        log_buffer.head = (log_buffer.head + space_to_free) % LOG_BUFFER_SIZE;
        log_buffer.tail = (log_buffer.tail + space_to_free) % LOG_BUFFER_SIZE;
    }

    // Format the message and copy it to the buffer
    va_start(args, format); // Restart the argument list
    vsnprintf(log_buffer.buffer + log_buffer.tail, LOG_BUFFER_SIZE - log_buffer.tail, format, args);
    va_end(args);

    // Update the byte count and tail pointer
    log_buffer.byte_count += len - 1; // -1 to exclude the null terminator
    log_buffer.new_line_count++;
    log_buffer.tail = (log_buffer.tail + len) % LOG_BUFFER_SIZE - 1; // -1 to exclude the null terminator
}

// Function to retrieve the byte count
uint16_t rp2040_get_byte_count() {
    return log_buffer.byte_count; // + log_buffer.new_line_count;
}

void rp2040_orient_copy_buffer(char* output_array) {
    // Copy the data from log_buffer to the provided output_array array, accounting for wrapping
    if (log_buffer.byte_count > 0) {
        if (log_buffer.head <= log_buffer.tail) {
            memcpy(output_array, log_buffer.buffer + log_buffer.head, log_buffer.byte_count);
        } else {
            int firstPartSize = LOG_BUFFER_SIZE - log_buffer.head;
            memcpy(output_array, log_buffer.buffer + log_buffer.head, firstPartSize);
            memcpy(output_array + firstPartSize, log_buffer.buffer, log_buffer.byte_count - firstPartSize);
        }
    }
}

void rp2040_log_flush(){
    int current_index = log_buffer.head;
    while (current_index != log_buffer.tail) {
	char c = log_buffer.buffer[current_index];
        putchar(c);
        current_index = (current_index + 1) % LOG_BUFFER_SIZE;
    }
}

