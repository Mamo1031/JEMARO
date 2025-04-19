#include "config.h"
#include "spi_functions.h"
#include "uart_functions.h"
#include "timer_functions.h"

int main(void) {
    
    // Variables
    int chip_addr = 0x40;
    int x_mag_LSB_addr = 0x42;

    // Setup and configuration
    uart_setup();
    spi_setup();
    magnetometer_config();

    // Acquire the Chip ID (CS3)
    LATDbits.LATD6 = 0;
    spi_write(chip_addr | 0x80);
    int value_from_chip = spi_write(0x00);
    LATDbits.LATD6 = 1;
    
    // Send it to the UART once
    tmr_wait_ms(TIMER1, 2000);  // Give time to connect on PC
    char buffer_tx[11];
    sprintf(buffer_tx, "N=%d", value_from_chip);
    UART_write_string(buffer_tx);

    // Set-up: Frequency of loop
    tmr_setup_period(TIMER2, 100, 0);
    
    while(1){
        
        ///////////////////////////////////
        // Acquire x-axis value from magnetometer
        ///////////////////////////////////
        
        LATDbits.LATD6 = 0;
        spi_write(x_mag_LSB_addr | 0x80);
        uint8_t LSB_byte = spi_write(0x00);
        uint8_t MSB_byte = spi_write(0x00);
        LATDbits.LATD6 = 1;
        
        // Mask to clear three least significant bits
        uint8_t masked_LSB = LSB_byte & 0b11111000;
        // Left-shift MSB_byte by 8 and OR with masked LSB_byte
        int combined_byte = ((MSB_byte << 8) | masked_LSB);
        // Divide by 8 to get correct scale
        int x_mag = combined_byte / 8;
        
        
        ///////////////////////////////////
        // Send it to UART at 10Hz
        ///////////////////////////////////
        
        // At 10Hz using protocol "$MAGX=xxx*", where xxx is the value
        memset(buffer_tx, 0, sizeof(buffer_tx)); // Clear buffer
        sprintf(buffer_tx, "$MAGX=%d*", x_mag);
        UART_write_string(buffer_tx);
        
        // Wait for timer
        tmr_wait_period(TIMER2);
    }

    return 0;
}
