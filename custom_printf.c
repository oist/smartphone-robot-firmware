#include <stdio.h>
#include "pico/types.h"
#include "pico/multicore.h"
#include "custom_printf.h"

volatile bool printf_synchronized = false;

void synchronized_printf(const char* format, ...) {
    // Wait for synchronization
    while (!printf_synchronized) {
        tight_loop_contents();
    }

    // Perform the printf
    printf(format);
}

void initialize_printf_synchronization() {
    // Set the synchronization flag to true
    printf_synchronized = true;
}
