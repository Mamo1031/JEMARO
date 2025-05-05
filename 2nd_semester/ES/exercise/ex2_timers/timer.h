/**
 * @file timer.h
 * @brief Timer functions for PIC microcontrollers
 * 
 * This file contains timer-related functions and definitions
 * for working with PIC microcontroller timers.
 */

// Header guard to prevent multiple inclusions
#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> // Include processor files - each processor file is guarded

// Timer identifiers
#define TIMER1 1  // Timer 1 identifier
#define TIMER2 2  // Timer 2 identifier

// Constants
#define MAX_VALUE_16_BITS 65535  // Maximum value for 16-bit timer
#define FCY_PROP 72000           // Proportional value (FCY/1000) to avoid overflows in calculations
#define MAX_ARRAY_SIZE 100       // Maximum size for timer arrays

/**
 * @brief Wait for specified milliseconds using the specified timer
 * @param timer Timer identifier (TIMER1 or TIMER2)
 * @param ms Time to wait in milliseconds
 */
void tmr_wait_ms(int timer, int ms);

/**
 * @brief Setup timer for periodic operation
 * @param timer Timer identifier (TIMER1 or TIMER2)
 * @param ms Period in milliseconds
 */
void tmr_setup_period(int timer, int ms);

/**
 * @brief Wait for the next period of the specified timer
 * @param timer Timer identifier (TIMER1 or TIMER2)
 * @return 1 if the timer was already expired when entering the function, 0 otherwise
 */
int tmr_wait_period(int timer);

#endif // TIMER_H
