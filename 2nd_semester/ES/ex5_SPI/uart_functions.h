#ifndef UART_FUNCTIONS_H
#define UART_FUNCTIONS_H

#include "config.h"

// UART related functions
void uart_setup();
void UART_write_string(const char* str);

#endif
