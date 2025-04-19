#include "uart_functions.h"

void uart_setup(){
    
    // Remapping of UART1 to pins of MicroBus 2
    RPINR18bits.U1RXR = 0x4B;
    RPOR0bits.RP64R = 1;
    IEC0bits.U1RXIE = 1;
    
    // UART Configuration
    U1BRG = 468;                // Obtained from (FCY/(16*baud_rate) - 1)
    U1MODEbits.UARTEN = 1;      // Enable UART
    U1STAbits.UTXEN = 1;        // Enable U1TX
}

void UART_write_string(const char* str) {
    while (*str != '\0') {
        while (U1STAbits.UTXBF) {}; // Wait while buffer is full
        U1TXREG = *str; // Transmit character
        str++;
    }
}
