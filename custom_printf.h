#ifndef CUSTOM_PRINTF_H
#define CUSTOM_PRINTF_H

#include <stdio.h>
#include <stdbool.h>

// Function to print with synchronization
void synchronized_printf(const char* format, ...);

void lock_printf_synchronization();
void unlock_printf_synchronization();
// Function to block until printf buffer is empty
void block_until_printf_empty();

#endif

