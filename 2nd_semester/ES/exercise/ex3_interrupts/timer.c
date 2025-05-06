/**
 * @file timer.c
 * @brief Timer implementation functions for PIC microcontrollers.
 *
 * This file implements timer configuration, delay functions, and 
 * interrupt service routines used for LED control.
 */

#include "timer.h"

// Global Variables
volatile int led2 = 0;                // LED2 state used in interrupt-driven blinking
volatile int led2_enabled = 1;        // Flag to enable/disable LED2 blinking
int interrupt_mode;                   // Current timer interrupt mode (PERIODIC_MODE or ONESHOT_MODE)
long int tmr1_pr_value;               // Original Timer1 period value (may exceed 16 bits)
long int tmr1_temp_pr_value;          // Working Timer1 period value

// Timer1 Interrupt Service Routine
void __attribute__ ((__interrupt__ , __auto_psv__)) _T1Interrupt(void) {
    // Clear the Timer1 interrupt flag
    IFS0bits.T1IF = 0;
    
    if (interrupt_mode == PERIODIC_MODE) {
        // Decrease the working period by the maximum value allowed (16-bit limit)
        tmr1_temp_pr_value -= MAX_VALUE_16_BITS;
        
        // If the accumulated period has been completed, toggle LED2
        if (tmr1_temp_pr_value < 0) {
            led2 = 1 - led2;              // Toggle LED2 state
            LATGbits.LATG9 = led2;        // Update physical LED2
            tmr1_temp_pr_value = tmr1_pr_value; // Reset the period
        }
        // Set the next period value, ensuring it fits in 16 bits
        PR1 = (tmr1_temp_pr_value >= MAX_VALUE_16_BITS) ? MAX_VALUE_16_BITS : tmr1_temp_pr_value;
    } 
    else if (interrupt_mode == ONESHOT_MODE) {
        // For one-shot mode (e.g., debouncing), stop Timer1 after completion
        IEC0bits.T1IE = 0;
        T1CONbits.TON = 0;
    }
}

// External Interrupt 1 Service Routine for button press
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void) {
    // Clear the INT1 interrupt flag
    IFS1bits.INT1IF = 0;
    
    // Toggle the LED2 blinking enable flag
    led2_enabled = !led2_enabled;
    
    // If LED2 blinking is disabled, ensure LED2 is turned off
    if (!led2_enabled) {
        LATGbits.LATG9 = 0;
    }
    
    // Start a one-shot Timer1 for debouncing (10ms interval)
    tmr_setup_period(TIMER1, 10, ONESHOT_MODE);
}

// Blocking delay function using a timer (supports delays longer than 16 bits)
void tmr_wait_ms(int timer, int ms) {
    // Available prescaler options: {ratio, register setting bits}
    int prescaler[][2] = { {1, 0}, {8, 1}, {64, 2}, {256, 3} };
    int prescaler_bits, pr_value;
    long int temp_pr_value = 0;
    
    // Calculate the period value for the given delay in ms
    for (int i = 0; i < 4; i++) {
        temp_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (temp_pr_value < MAX_VALUE_16_BITS) break;
    }
    
    // Wait in chunks if the delay exceeds the 16-bit timer capacity
    while (temp_pr_value > 0) {
        pr_value = (temp_pr_value >= MAX_VALUE_16_BITS) ? MAX_VALUE_16_BITS : temp_pr_value;
        temp_pr_value -= MAX_VALUE_16_BITS;
        
        if (timer == TIMER1) {
            TMR1 = 0;                        
            PR1 = pr_value;                  
            T1CONbits.TCKPS = prescaler_bits; 
            IFS0bits.T1IF = 0;               
            T1CONbits.TON = 1;               
            while (!IFS0bits.T1IF);          // Busy wait until timer flag is set
            IFS0bits.T1IF = 0;               
        } 
        else if (timer == TIMER2) {
            TMR2 = 0;
            PR2 = pr_value;
            T2CONbits.TCKPS = prescaler_bits;
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 1;
            while (!IFS0bits.T2IF);          // Busy wait until timer flag is set
            IFS0bits.T2IF = 0;
        }
    }
}

// Setup a timer for periodic operation with interrupts
void tmr_setup_period(int timer, int ms, int mode) {
    // Available prescaler options: {ratio, register setting bits}
    int prescaler[][2] = { {1, 0}, {8, 1}, {64, 2}, {256, 3} };
    int prescaler_bits, pr_value;
    
    // Store the interrupt mode (PERIODIC_MODE or ONESHOT_MODE)
    interrupt_mode = mode;
    
    if (timer == TIMER1) {
        // Calculate Timer1 period based on ms and select appropriate prescaler
        for (int i = 0; i < 4; i++) {
            tmr1_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
            prescaler_bits = prescaler[i][1];
            if (tmr1_pr_value < MAX_VALUE_16_BITS) break;
        }
        pr_value = (tmr1_pr_value >= MAX_VALUE_16_BITS) ? MAX_VALUE_16_BITS : tmr1_pr_value;
        tmr1_temp_pr_value = tmr1_pr_value; // Save full period value
        
        TMR1 = 0;
        PR1 = pr_value;
        T1CONbits.TCKPS = prescaler_bits;
        IFS0bits.T1IF = 0;
        IEC0bits.T1IE = 1;               // Enable Timer1 interrupt
        T1CONbits.TON = 1;
    }
    else if (timer == TIMER2) {
        long int temp_pr_value;
        // Calculate Timer2 period value using a similar approach
        for (int i = 0; i < 4; i++) {
            temp_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
            prescaler_bits = prescaler[i][1];
            if (temp_pr_value < MAX_VALUE_16_BITS) break;
        }
        pr_value = (temp_pr_value >= MAX_VALUE_16_BITS) ? MAX_VALUE_16_BITS : temp_pr_value;
        
        TMR2 = 0;
        PR2 = pr_value;
        T2CONbits.TCKPS = prescaler_bits;
        IFS0bits.T2IF = 0;
        IEC0bits.T2IE = 1;               // Enable Timer2 interrupt
        T2CONbits.TON = 1;
    }
}

// Setup external button interrupt on INT1 (mapped to pin RE9)
void setup_button_interrupt(void) {
    // Disable analog functions on ports to use digital I/O
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = 0x0000;
    
    TRISEbits.TRISE9 = 1;       // Set pin E9 as input (button)
    RPINR0bits.INT1R = 0x59;    // Map INT1 to pin RE9 (RPI89)
    
    INTCON2bits.GIE = 1;        // Enable global interrupts
    IFS1bits.INT1IF = 0;        // Clear INT1 interrupt flag
    IEC1bits.INT1IE = 1;        // Enable INT1 interrupt
}
