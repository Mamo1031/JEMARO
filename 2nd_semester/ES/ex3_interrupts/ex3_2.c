/**
 * @file ex3_2.c
 * @brief LED control with button interrupt
 * 
 * This program demonstrates:
 * - LED2 blinking at 2.5 Hz (200ms on, 200ms off) without interrupts
 * - Button press toggles LED2 state using interrupts
 */

#include "timer.h"

// Global variables
int interrupt_mode;
volatile int led2_enabled = 1;  // Flag to control LED2 blinking (initially enabled)

int main(void) {
    // Initialize LED pins as outputs
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    
    // Define timing parameters
    int led2_delay = 200;   // 200ms delay for 2.5Hz frequency
    int led2_state = 0;     // Track LED2 state
    
    // Setup the button interrupt
    setup_button_interrupt();
    
    // Main loop for LED2 blinking without interrupts
    while(1) {
        if (led2_enabled) {
            // Toggle LED2 state
            led2_state = 1 - led2_state;
            
            // Update physical LED2 based on logical state
            LATGbits.LATG9 = led2_state;
        }
        
        // Wait for delay to maintain 2.5Hz frequency
        tmr_wait_ms(TIMER2, led2_delay);
    }

    return 0;
}
