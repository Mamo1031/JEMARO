/**
 * Exercise 4.1: UART Echo Application
 * This program configures UART1 to read characters and echo them back
 * using MikroBUS 2 pins.
 */
#include "uart_utils.h"

/**
 * Main function - configures UART1 and implements echo functionality
 */
int main(void) {
    
    // Configure the UART1 to use the MikroBUS 2.
    setup_uart();
    
    // Main loop - echo back all received characters
    while(1) {
        // Read a character from UART1 and send it back
        char received_char = UART_read();
        UART_write(received_char);
    }
    
    return 0;  // Never reached
}
