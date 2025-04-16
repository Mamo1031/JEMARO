/**
 * @file ex3_1.c
 * @brief LED blinking demonstration using timer interrupts
 * 
 * This program demonstrates two LEDs blinking at different frequencies:
 * - LED1 blinks at 2.5 Hz (using polling method)
 * - LED2 blinks at 1 Hz (using timer interrupts)
 */

#include "timer.h"

// Global variable declaration to set interrupt mode
int interrupt_mode;

int main(void) {
    // Initialize LED pins as outputs
    TRISAbits.TRISA0 = 0;   // LED1 on pin A0 (set as output)
    TRISGbits.TRISG9 = 0;   // LED2 on pin G9 (set as output)
    
    // Define timing parameters
    int led1_delay = 200;   // 200ms delay for 2.5Hz frequency (200ms on, 200ms off)
    int led2_period = 500;  // 500ms for 1Hz frequency (500ms on, 500ms off)
    int led1_state = 0;     // Track LED1 state (on/off)
    
    // Setup Timer1 with periodic interrupt for LED2 blinking
    tmr_setup_period(TIMER1, led2_period, PERIODIC_MODE);
    
    // Main loop
    while(1) {
        // Toggle LED1 state
        led1_state = 1 - led1_state;
        
        // Update physical LED1 based on logical state
        LATAbits.LATA0 = led1_state;
        
        // Wait for the appropriate delay to maintain 2.5Hz frequency
        tmr_wait_ms(TIMER2, led1_delay);
    }

    return 0;  // Never reached in embedded applications
}
