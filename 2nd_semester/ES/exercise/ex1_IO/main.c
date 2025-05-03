#include "xc.h"

/**
 * Turn on LED when button is pressed, turn off when released
 * This function reads the button state and controls the LED accordingly
 */
void turn_on_when_pressed() {
    int pin_value;
    pin_value = PORTEbits.RE8;
    
    // When button is not pressed, its value is high (turn LED off)
    if(pin_value) {
        LATAbits.LATA0 = 0;
    }
    // When button is pressed, its value is low (turn LED on)
    else {
        LATAbits.LATA0 = 1;
    }
}

/**
 * Toggle LED state on each button press (not on release)
 * @param previous_pin_value Pointer to store the previous button state
 * @param previous_output_value Pointer to store the previous LED state
 */
void toggle_when_pressed(int *previous_pin_value, int *previous_output_value) {
    int pin_value;
    pin_value = PORTEbits.RE8;

    // Initialize previous pin value on first iteration
    if (*previous_pin_value == -1) {
        *previous_pin_value = pin_value;
        return;
    }
    
    // Detect falling edge (button press, not release)
    if (*previous_pin_value == 1 && pin_value == 0) {
        // Toggle LED state
        if (*previous_output_value == 0) {
            LATAbits.LATA0 = 1;     // set pin A0 to high (Turn on LED)
            *previous_output_value = 1;
        }
        else {
            LATAbits.LATA0 = 0;     // set pin A0 to low (Turn off LED)
            *previous_output_value = 0;
        }
    }

    // Store current pin value for next iteration
    *previous_pin_value = pin_value;
}

/**
 * Main function - initializes pins and contains the main control loop
 */
int main(void) {
    // Initialize pins
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED)
    TRISEbits.TRISE8 = 1;   // pin E8 set as input (Button)
    
    // Disable all analog functionality on pins to use them as digital I/O
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    // Variables to track button and LED states
    int previous_pin_value = -1;
    int previous_output_value = 0;
    
    // Main control loop
    while(1) {
        // DEMO 1: Always on LED (uncomment to test)
        // LATAbits.LATA0 = 1;     // set pin A0 to high
        
        // DEMO 2: LED on while button pressed (uncomment to test)
        // turn_on_when_pressed();
        
        // DEMO 3: Toggle LED with each button press (uncomment to test)
        // toggle_when_pressed(&previous_pin_value, &previous_output_value);
    }
    
    return 0; // Never reached in embedded applications
}
