#include "timer.h"

/**
 * Configure timer to trigger periodically with specified milliseconds
 * @param timer The timer number (1 or 2)
 * @param ms    Period in milliseconds
 */
void tmr_setup_period(int timer, int ms){
    
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // Calculate appropriate prescaler values to fit within 16-bit range
    int prescaler_bits;
    long int pr_value;
    
    for (int i = 0; i < 4; i++){
        pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // Set the timer register values based on timer number
    switch(timer) {
        case 1:
            TMR1 = 0;                           // Reset timer counter
            PR1 = pr_value;                     // Timer will count up to this value
            T1CONbits.TCKPS = prescaler_bits;   // Prescaler bits (0,1,2,3)
            IFS0bits.T1IF = 0;                  // Clear Interrupt flag
            T1CONbits.TON = 1;                  // Start the timer
            break;
        case 2:
            TMR2 = 0;                           // Reset timer counter
            PR2 = pr_value;                     // Timer will count up to this value
            T2CONbits.TCKPS = prescaler_bits;   // Prescaler bits (0,1,2,3)
            IFS0bits.T2IF = 0;                  // Clear Interrupt flag
            T2CONbits.TON = 1;                  // Start the timer
            break;
    }
}

/**
 * Wait for the timer period to complete (blocking function)
 * @param timer The timer number (1 or 2)
 * @return     1 if the timer was already expired when entering the function, 0 otherwise
 */
int tmr_wait_period(int timer){
    int already_expired = 0;
    
    switch(timer) {
        case 1:
            // Check if timer already expired
            if(IFS0bits.T1IF) {
                already_expired = 1;
            }
            while(!IFS0bits.T1IF);  // Busy waiting for timer1 interrupt flag
            IFS0bits.T1IF = 0;      // Clear the interrupt flag
            break;
        case 2:
            // Check if timer already expired
            if(IFS0bits.T2IF) {
                already_expired = 1;
            }
            while(!IFS0bits.T2IF);  // Busy waiting for timer2 interrupt flag
            IFS0bits.T2IF = 0;      // Clear the interrupt flag
            break;
    }
    
    return already_expired;  // Return 1 if timer was already expired, 0 otherwise
}

/**
 * Wait for specified milliseconds using the selected timer (blocking function)
 * This function configures the timer, starts it, and waits for completion.
 * It's a one-shot timer operation that combines setup and waiting.
 * 
 * @param timer The timer number (1 or 2)
 * @param ms    Delay time in milliseconds
 */
void tmr_wait_ms(int timer, int ms){
    // Configure the specified timer for the requested period
    tmr_setup_period(timer, ms);
    
    // Wait for the timer period to complete
    tmr_wait_period(timer);
    
    // Stop the timer after use
    if (timer == 1) {
        T1CONbits.TON = 0;  // Stop timer 1
    } else if (timer == 2) {
        T2CONbits.TON = 0;  // Stop timer 2
    }
}
