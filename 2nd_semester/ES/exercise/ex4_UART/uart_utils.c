/**
 * UART Utilities
 * This file contains common UART and timer functions used across exercises
 */
#include "uart_utils.h"

/**
 * Reads a character from UART1
 * Blocking function that waits until data is available
 * @return The received character
 */
char UART_read() {
    while (!U1STAbits.URXDA); // Wait until data is available in the receive buffer
    return U1RXREG;           // Read and return the received character
}

/**
 * Writes a character to UART1
 * @param data The character to send
 */
void UART_write(char data) {
    while (U1STAbits.UTXBF); // Wait while buffer is full
    U1TXREG = data;          // Transmit character
}

/**
 * Writes a character to UART1
 * @param c The character to send
 */
void UART_write_character(char c) {
    while (U1STAbits.UTXBF) {}; // Wait while buffer is full
    U1TXREG = c;                // Transmit character
}

/**
 * Writes a string to UART1
 * @param str The string to send (null-terminated)
 */
void UART_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF) {}; // Wait while buffer is full
        U1TXREG = *str;             // Transmit character
        str++;
    }
}

/**
 * Sets up UART1 to use the MikroBUS 2 pins
 * Configures UART1 to work at 9600 baud rate
 */
void setup_uart(){
    // Configure UART1 to work at 9600 bps
    // BRG calculation: (FCY/(16*baudrate) - 1) = (72MHz/(16*9600) - 1) = 468.75 ≈ 468
    U1BRG = 468;                
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
    
    // Remapping of UART1 to pins of MicroBus 2
    RPINR18bits.U1RXR = 75;     // RD11(RP75) is configured as UART1 RX
    RPOR0bits.RP64R = 1;        // RD0(RP64) is configured as UART1 TX
    
    // Enable UART1 RX interrupt
    IFS0bits.U1RXIF = 0;        // Clear the UART RX interrupt flag
    IEC0bits.U1RXIE = 1;        // Enable UART RX interrupt
}

/**
 * Waits for the specified number of milliseconds using the specified timer
 * @param timer The timer number to use (1-4)
 * @param ms    The number of milliseconds to wait
 */
void tmr_wait_ms(int timer, int ms){
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CONSTANTS FOR PRESCALER AND TIMER PR VALUE
    int prescaler_bits;
    long int pr_value;
    long int temp_pr_value;

    // OBTAINING THE PR VALUE
    for (int i = 0; i < 4; i++) {
        temp_pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (temp_pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    // SETUP & WAITING
    while (temp_pr_value > 0){
        if(temp_pr_value >= MAX_VALUE_16_BITS) {pr_value = MAX_VALUE_16_BITS;}
        else {pr_value = temp_pr_value;}

        temp_pr_value = temp_pr_value - MAX_VALUE_16_BITS;
            
        // SETUP THE APPROPRIATE TIMER REGISTER VALUES
        tmr_set_register_values(timer, pr_value, prescaler_bits, 0);
        
        // BUSY WAITING FOR FLAG
        if (timer == 1){
            while(!IFS0bits.T1IF);      // busy waiting
            IFS0bits.T1IF = 0;          // Clear the interrupt flag
        }
        else if (timer == 2){
            while(!IFS0bits.T2IF);   
            IFS0bits.T2IF = 0;      
        }
        else if (timer == 3){
            while(!IFS0bits.T3IF);  
            IFS0bits.T3IF = 0;        
        }
        else if (timer == 4){
            while(!IFS1bits.T4IF);    
            IFS1bits.T4IF = 0;          
        }
    }
}

/**
 * Sets up a periodic timer
 * @param timer        The timer number to use (1-4)
 * @param ms           The period in milliseconds
 * @param has_interrupt Whether to enable interrupts for this timer
 */
void tmr_setup_period(int timer, int ms, int has_interrupt){
    // List of prescaler values: {ratio, bits}
    int prescaler[][2] = {
        {1, 0}, {8, 1}, {64, 2}, {256, 3}
    };
    
    // CALCULATE THE PRESCALER & PR VALUES
    int prescaler_bits;
    long int pr_value;
    
    for (int i = 0; i < 4; i++) {
        pr_value = (FCY_PROP * ms) / (prescaler[i][0]);
        prescaler_bits = prescaler[i][1];
        if (pr_value < MAX_VALUE_16_BITS) {break;}
    }
    
    tmr_set_register_values(timer, pr_value, prescaler_bits, has_interrupt);
}

/**
 * Waits for the next period of the specified timer
 * @param timer The timer number to use (1-4)
 * @return      1 if the deadline was missed, 0 otherwise
 */
int tmr_wait_period(int timer){
    if (timer == 1){
        if (IFS0bits.T1IF){IFS0bits.T1IF = 0; return 1;}
        else{while(!IFS0bits.T1IF); IFS0bits.T1IF = 0; return 0;}
    }
    else if (timer == 2){
        if (IFS0bits.T2IF) {
            IFS0bits.T2IF = 0; 
            return 1;
        }
        else{
            while(!IFS0bits.T2IF);   
            IFS0bits.T2IF = 0;          
            return 0;
        }
    }
    else if (timer == 3){
        if (IFS0bits.T3IF){
            IFS0bits.T3IF = 0;
            return 1;
        }
        else{
            while(!IFS0bits.T3IF) {};  
            IFS0bits.T3IF = 0;         
            return 0;
        }
    }
    else if (timer == 4){
        if (IFS1bits.T4IF){
            IFS1bits.T4IF = 0;
            return 1;
        }
        else{
            while(!IFS1bits.T4IF) {};     
            IFS1bits.T4IF = 0;         
            return 0;
        }
    }
    return 0;
}

/**
 * Sets the register values for a timer
 * @param timer        The timer number to use (1-4)
 * @param pr_value     The PR value to set
 * @param prescaler_bits The prescaler bits to set
 * @param has_interrupt Whether to enable interrupts for this timer
 */
void tmr_set_register_values(int timer, int pr_value, int prescaler_bits,
                              int has_interrupt){
    if (timer == 1){
        TMR1 = 0;                                   // Reset timer counter
        PR1 = pr_value;                             // Timer will count up to this value
        T1CONbits.TCKPS = prescaler_bits;           // Prescaler bits (0,1,2,3)
        IFS0bits.T1IF = 0;                          // Clear the interrupt flag
        if (has_interrupt) {IEC0bits.T1IE = 1;}     // Enable timer interrupt
        T1CONbits.TON = 1;                          // Start the timer
    }
    else if (timer == 2){
        TMR2 = 0;                   
        PR2 = pr_value;                   
        T2CONbits.TCKPS = prescaler_bits;   
        IFS0bits.T2IF = 0;
        if (has_interrupt) {IEC0bits.T2IE = 1;}     
        T2CONbits.TON = 1;          
    }
    else if (timer == 3){
        TMR3 = 0;                   
        PR3 = pr_value;                   
        T3CONbits.TCKPS = prescaler_bits;   
        IFS0bits.T3IF = 0;
        if (has_interrupt) {IEC0bits.T3IE = 1;} 
        T3CONbits.TON = 1;          
    }
    else if (timer == 4){
        TMR4 = 0;                   
        PR4 = pr_value;                   
        T4CONbits.TCKPS = prescaler_bits;   
        IFS1bits.T4IF = 0;      
        if (has_interrupt) {IEC1bits.T4IE = 1;}  
        T4CONbits.TON = 1;          
    }
}

/**
 * Sets up the I/O pins for LEDs and buttons
 */
void setup_io(){
    TRISAbits.TRISA0 = 0;   // pin A0 set as output (LED 1)
    TRISGbits.TRISG9 = 0;   // pin G9 set as output (LED 2)
    
    // Disable all analog pins
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // CONFIG: PIN REMAPPING
    TRISEbits.TRISE8 = 1;       // pin E8 set as input (Button T2)
    RPINR0bits.INT1R = 0x58;    // 0x58 = 88. Refers to PIN RE8/RPI88

    TRISEbits.TRISE9 = 1;       // pin E9 set as input (Button T3)
    RPINR1bits.INT2R = 0x59;    // 0x59 = 89. Refers to PIN RE9/RPI89 

    // If GIE is enabled before remapping, crash occurs.
    
    // Enabling interrupts
    INTCON2bits.GIE = 1;        // Global interrupt enable
    IFS1bits.INT1IF = 0;        // Clear INT1 interrupt flag
    IEC1bits.INT1IE = 1;        // Enable INT1 interrupt
    IFS1bits.INT2IF = 0;        // Clear INT2 interrupt flag
    IEC1bits.INT2IE = 1;        // Enable INT2 interrupt
}
