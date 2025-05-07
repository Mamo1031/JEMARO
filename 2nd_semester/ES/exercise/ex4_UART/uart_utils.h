/**
 * UART Utilities Header
 * This file contains declarations for common UART and timer functions
 */
#ifndef UART_UTILS_H
#define UART_UTILS_H

#include <xc.h>
#include <stdio.h>
#include <string.h>

// Timer constants
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4

#define MAX_VALUE_16_BITS 65535
#define FCY_PROP 72000  // Proportional value: FCY/1000
#define BAUDRATE 9600

/**
 * UART Functions
 */
char UART_read();
void UART_write(char data);
void UART_write_character(char c);
void UART_write_string(const char* str);
void setup_uart();

/**
 * Timer Functions
 */
void tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer, int ms, int has_interrupt);
int tmr_wait_period(int timer);
void tmr_set_register_values(int timer, int pr_value, int prescaler_bits, 
                             int has_interrupt);

/**
 * I/O Setup Functions
 */
void setup_io();

#endif /* UART_UTILS_H */
