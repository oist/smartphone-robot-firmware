#ifndef CUSTOM_PRINTF_H
#define CUSTOM_PRINTF_H

#include <stdio.h>
#include <stdbool.h>

// Function to print with synchronization
void synchronized_printf(const char* format, ...);

// Function to initialize the synchronization mechanism
void initialize_printf_synchronization();

#endif

