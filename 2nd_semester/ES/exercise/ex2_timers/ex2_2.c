/**
 * @file ex2_2.c
 * @brief LED blinking program using timer functionality
 * 
 * This program demonstrates the use of timers to create a blinking LED pattern.
 * The LED connected to pin A0 will turn on for 20ms and then turn off for 200ms,
 * creating an asymmetric blinking pattern.
 */

#include "timer.h"

/**
 * @brief Main program entry point
 * 
 * Sets up pin A0 as an output and enters an infinite loop that
 * creates a blinking pattern by turning the LED on and off with
 * different timing intervals.
 * 
 * @return int Program return value (never reached in this application)
 */
int main(void) {
    // Configure hardware
    TRISAbits.TRISA0 = 0;   // Set pin A0 as output for the LED
    
    // Main program loop - runs indefinitely
    while(1) {
        // LED ON phase
        LATAbits.LATA0 = 1;     // Turn on LED by setting pin A0 high
        tmr_wait_ms(TIMER1, 20); // Keep LED on for 20ms
        
        // LED OFF phase
        LATAbits.LATA0 = 0;     // Turn off LED by setting pin A0 low
        tmr_wait_ms(TIMER1, 200); // Keep LED off for 200ms
    }
   
    return 0; // This line is never reached due to infinite loop
}
