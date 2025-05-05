/**
 * @file ex2_3.c
 * @brief Timer functionality test program
 * 
 * This program demonstrates advanced timer functions with two LEDs.
 * It tests tmr_wait_ms() with different delay values and monitors
 * timer expiration using tmr_wait_period() return values.
 * LED1 blinks with the specified delay, while LED2 indicates when
 * the timer period has been exceeded.
 */

#include "timer.h"

/**
 * @brief Main program entry point
 * 
 * Sets up two LEDs and configures a timer with a fixed period.
 * Then enters an infinite loop that tests timer behavior with different
 * delay settings. LED1 toggles after each delay, while LED2 indicates
 * when the timer period has been exceeded.
 * 
 * @return int Program return value (never reached in this application)
 */
int main(void) {
    // Configure hardware
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)

    int delay = 2000;        // Delay value in ms - Try with 50, 200, 2000
    int led = 0;            // Variable to track LED1 state (0=OFF, 1=ON)
    int ret = 0;            // Return value from tmr_wait_period()
    
    // Configure Timer1 with a 200ms period
    tmr_setup_period(TIMER1, 200);

    // Main program loop
    while(1){
        // Wait for specified delay using Timer2
        tmr_wait_ms(TIMER2, delay);
        
        // Toggle LED1 state
        led = 1 - led;                  // Toggle between 1 and 0
        if(led) {LATAbits.LATA0 = 1;}   // Turn ON LED1 by setting pin A0 high
        else {LATAbits.LATA0 = 0;}      // Turn OFF LED1 by setting pin A0 low
        
        // Check or wait for Timer1 period to complete
        ret = tmr_wait_period(TIMER1);

        // Update LED2 based on timer status
        if (ret) {
            LATGbits.LATG9 = 1;         // Turn ON LED2 if timer period was already expired
        } else {
            LATGbits.LATG9 = 0;         // Turn OFF LED2 if timer completed normally
        }
    }
    
    return 0;  // Standard return (never reached in this infinite loop)
}
