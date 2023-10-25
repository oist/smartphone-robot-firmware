#include <stdio.h>
#include "pico/types.h"
#include "pico/multicore.h"
#include "custom_printf.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/sync.h"

volatile bool printf_lock = false;

void synchronized_printf(const char* format, ...) {
    // Disable interrupts
    uint32_t state = save_and_disable_interrupts();
    
    // Wait for synchronization						   
    while (printf_lock) {
        tight_loop_contents();
    }

    lock_printf_synchronization();

    // Perform the printf
    printf(format);
    
    // Reset the synchronization flag to allow the other core to continue
    printf_lock = false;

    // Restore the interrupt state
    restore_interrupts(state);
}

void lock_printf_synchronization() {
    // Set the synchronization flag to true
    printf_lock = true;
}

void unlock_printf_synchronization() {
    // Set the synchronization flag to false
    printf_lock = false;
}
