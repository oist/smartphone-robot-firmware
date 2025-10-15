#include <stdarg.h>
#include <stdio.h>
#include "pico/mutex.h"
#include "pico/types.h"
#include <string.h>
#include "rp2040_log.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define LOG_UART uart0
#define LOG_UART_TX_PIN 16
#define LOG_UART_RX_PIN 17

static CircularBufferLog log_buffer;
auto_init_mutex(rp2040_log_buffer_mutex);

// Initialize the circular buffer
void rp2040_log_init() {
    log_buffer.head = 0; 
    log_buffer.tail = 0;
    
    #ifdef LOGGER_UART
    // Initialize dedicated UART for logging (separate from stdio)
    uart_init(LOG_UART, 115200);
    gpio_set_function(LOG_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LOG_UART_RX_PIN, GPIO_FUNC_UART);
    #endif
}

void rp2040_log_acquire_lock() {
    mutex_enter_blocking(&rp2040_log_buffer_mutex);
}

void rp2040_log_release_lock() {
    mutex_exit(&rp2040_log_buffer_mutex);
}


void rp2040_log(const char* format, ...) {
    va_list args;

    #ifdef LOGGER_UART
    // For UART mode, log directly to UART hardware (not stdio)
    char buffer[256];
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Send directly to UART0 on GPIO16/17
    uart_puts(LOG_UART, buffer);
    return;
    #endif

    va_start(args, format);
    // Calculate the number of characters required
    int len = vsnprintf(NULL, 0, format, args) + 1; //include the /n 
    va_end(args);

    // truncate the message if it is too long else expect chaos when overwriting unknown areas of memory
    if (len > LOG_BUFFER_CHAR_LIMIT) {
	len = LOG_BUFFER_CHAR_LIMIT;
    }

    rp2040_log_acquire_lock(); // Acquire the lock

    // Format the message and copy it to the buffer, handling wrapping
    va_start(args, format); // Restart the argument list
    vsnprintf(log_buffer.log_array[log_buffer.tail], len, format, args);
    va_end(args);

    log_buffer.log_array_line_size[log_buffer.tail] = len; // store the size of the line -2 for removing \n and null terminator
    // update head
    if (log_buffer.tail == log_buffer.head) {
        log_buffer.head = (log_buffer.head + 1) % LOG_BUFFER_LINE_COUNT;
    }
    log_buffer.tail = (log_buffer.tail + 1) % LOG_BUFFER_LINE_COUNT; // Update tail correctly

    rp2040_log_release_lock(); // Release the lock
}

// Function to retrieve the total number of bytes within the log_array
uint16_t rp2040_get_byte_count() {
   // sum up the values witin log_array_line_size
   uint16_t byte_count = 0; 
   for (int i = 0; i < LOG_BUFFER_LINE_COUNT; i++) {
	   byte_count += log_buffer.log_array_line_size[i] - 1;
   }
   return byte_count;
}

void rp2040_log_flush(){
    // printf each line within the log_array starting at the head
    for (int i = 0; i < LOG_BUFFER_LINE_COUNT; i++) {
	// only print up to log_array_line_size
	printf("%.*s", log_buffer.log_array_line_size[log_buffer.head], log_buffer.log_array[log_buffer.head]);
	log_buffer.head = (log_buffer.head + 1) % LOG_BUFFER_LINE_COUNT; // Update head correctly
    }
}

