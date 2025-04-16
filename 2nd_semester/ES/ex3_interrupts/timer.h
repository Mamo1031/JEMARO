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

// Timer interrupt modes
#define PERIODIC_MODE 0  // Periodic timer mode (keeps running)
#define ONESHOT_MODE 1   // One-shot timer mode (stops after one interrupt)

// Constants
#define MAX_VALUE_16_BITS 65535  // Maximum value for 16-bit timer
#define FCY_PROP 72000           // Proportional value (FCY/1000) to avoid overflows in calculations
#define MAX_ARRAY_SIZE 100       // Maximum size for timer arrays

// Global variables
extern volatile int led2;
extern volatile int led2_enabled;  // Added LED2 enabled flag
extern long int tmr1_pr_value;
extern long int tmr1_temp_pr_value;
extern int interrupt_mode;

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
 * @param mode Interrupt mode (PERIODIC_MODE or ONESHOT_MODE)
 */
void tmr_setup_period(int timer, int ms, int mode);

/**
 * @brief Setup button with external interrupt
 */
void setup_button_interrupt(void);

/**
 * @brief Timer 1 interrupt service routine
 */
void __attribute__ ((__interrupt__ , __auto_psv__)) _T1Interrupt(void);

/**
 * @brief External interrupt 1 service routine
 */
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void);

#endif // TIMER_H
