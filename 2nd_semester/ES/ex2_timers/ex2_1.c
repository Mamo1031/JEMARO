/**
 * @file ex2_1.c
 * @brief LED Blink Example using Timer
 * 
 * This program demonstrates the use of a timer to create a precise
 * blinking pattern for an LED connected to pin A0. The LED will
 * toggle its state every 200ms, creating a consistent blinking effect.
 */
#include "timer.h"

/**
 * @brief Main program entry point
 * 
 * Sets up pin A0 as an output and Timer1 with a 200ms period.
 * Then enters an infinite loop that toggles the LED state at
 * each iteration, waiting for the timer period to complete
 * before the next toggle.
 * 
 * @return int Program return value (never reached in this application)
 */
int main(void) {
    // Initialize hardware
    TRISAbits.TRISA0 = 0;           // Configure pin A0 as digital output for LED
    int led = 0;                    // Variable to track LED state (0=OFF, 1=ON)
    
    // Configure Timer1 with a 200ms period
    tmr_setup_period(TIMER1, 200);  // Initialize Timer1 with 200ms interval
    
    // Main program loop
    while(1) {
        // Toggle LED state
        led = 1 - led;              // Flip between 1 and 0 to toggle LED state
        
        // Update LED based on current state
        if(led) {
            LATAbits.LATA0 = 1;     // Turn ON the LED by setting pin A0 high
        } else {
            LATAbits.LATA0 = 0;     // Turn OFF the LED by setting pin A0 low
        }

        // Wait for the timer period to complete before next iteration
        // This ensures consistent timing between LED state changes
        tmr_wait_period(TIMER1);    // Pause execution until 200ms have elapsed
    }
    
    return 0;  // Standard return (never reached in this infinite loop)
}
