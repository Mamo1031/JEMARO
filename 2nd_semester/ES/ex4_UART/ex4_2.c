/**
 * Exercise 4.2: UART Control Application
 * This program implements the control of LEDs via UART commands
 * and provides status information through UART.
 */
#include "uart_utils.h"

////// GLOBAL VARIABLES
 
// For LED control
volatile int led1 = 0;
volatile int led2 = 0;
volatile int enable_blink = 1;

// For UART TX
int deadlines_missed = 0;
int characters_received = 0;
int send_deadlines = 0;
int send_characters = 0;

// Buffer to store received characters
char buffer_rx[10];
int buffer_rx_index = 0;
char* target_words[] = {"LD1", "LD2"};

/**
 * Handles button T2 interrupt (INT1) - triggers sending character count
 */
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;    // Reset interrupt flag
    IEC1bits.T4IE = 1;      // Enable timer 4 interrupt
    send_characters = 1;    // T2 button was pressed - send character count
}

/**
 * Handles button T3 interrupt (INT2) - triggers sending deadlines missed
 */
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT2Interrupt(void) {
    IFS1bits.INT2IF = 0;    // Reset interrupt flag
    IEC1bits.T4IE = 1;      // Enable timer 4 interrupt
    send_deadlines = 1;     // T3 button was pressed - send deadlines missed
}

/**
 * Handles Timer3 interrupt - controls LED2 blinking
 */
void __attribute__ ((__interrupt__ , __auto_psv__)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;  // Reset interrupt flag

    if(enable_blink){
        led2 = 1 - led2;
        if(led2) {LATGbits.LATG9 = 1;}   
        else {LATGbits.LATG9 = 0;}
    }
    else{
        LATGbits.LATG9 = 0;
    }
}

/**
 * Handles UART1 RX interrupt - processes received characters
 */
void __attribute__((__interrupt__)) _U1RXInterrupt(void){
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
    while (U1STAbits.URXDA){
        buffer_rx[buffer_rx_index++] = U1RXREG;
        characters_received++;
    }
}

/**
 * Simulates an algorithm that takes 7ms to execute
 */
void algorithm(){
    tmr_wait_ms(TIMER2, 7); // Simulate a function that takes 7ms to execute
}

/**
 * Handles actions based on received UART commands
 * @param received_string The string received from UART
 */
void handle_action(const char* received_string) {
    if (strcmp(received_string, "LD2") == 0) {
        enable_blink = 1 - enable_blink;
    }
    if (strcmp(received_string, "LD1") == 0) {
        led1 = 1 - led1;
        if(led1) {LATAbits.LATA0 = 1;}   
        else {LATAbits.LATA0 = 0;}
    } 
}

/**
 * Main function - sets up and runs the application
 */
int main(void) {
    
    // Setup LEDS and buttons
    setup_io();
    
    // Configure LED2 to blink at 2.5Hz (period = 400ms, so 200ms on, 200ms off)
    int blink_delay = 200;
    
    // Configure the UART1 to use the MikroBUS 2.
    setup_uart();
    
    // Setup algorithm execution at 100Hz (period = 10ms)
    tmr_setup_period(TIMER1, 10, 0);
    
    // Setup LED2 blinking at 2.5Hz (period = 400ms)
    tmr_setup_period(TIMER3, blink_delay, 1);
    
    while(1){
        
        algorithm();    // Takes 7[ms]
        
        // ASSIGNMENT CODE 1: SEND DATA TO UART
        if (send_characters){
            char buffer_tx[10]; // Buffer to hold the formatted string
            sprintf(buffer_tx, "C=%d", characters_received);
            UART_write_string(buffer_tx);
            send_characters = 0;
        }

        if (send_deadlines){
            char buffer_tx[10]; // Buffer to hold the formatted string
            sprintf(buffer_tx, "D=%d", deadlines_missed);
            UART_write_string(buffer_tx);
            send_deadlines = 0;
        }
        
        // ASSIGNMENT CODE 2: ANALYZE DATA READ FROM UART
        int match = 0;
        
        for (int i = 0; i < sizeof(target_words) / sizeof(target_words[0]); i++) {
            if (strcmp(buffer_rx, target_words[i]) == 0) {
                handle_action(buffer_rx);
                memset(buffer_rx, 0, sizeof(buffer_rx)); // Clear buffer
                buffer_rx_index = 0;
                match = 1;
                break;
            } 
            else if (strstr(target_words[i], buffer_rx) != NULL) {
                match = 1;
                break;
            }
        }

        if (!match) {
            memset(buffer_rx, 0, sizeof(buffer_rx)); // Clear buffer
            buffer_rx_index = 0;
        }

        // END OF ASSIGNMENT CODE
        
        int ret = tmr_wait_period(TIMER1);  // (1) if the deadline was missed
        deadlines_missed += ret;
    }
    
    return 0;
}
