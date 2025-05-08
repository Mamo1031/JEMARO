#include "spi_functions.h"

unsigned int spi_write(unsigned int addr){
    // Transmit segment
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = addr;
    // Receive segment
    while (SPI1STATbits.SPIRBF == 0);
    unsigned int response = SPI1BUF;
    return response;
}

void spi_setup(){
    // Port configuration: CS Lines
    TRISBbits.TRISB3 = 0;   // CS1: Accelerometer
    TRISBbits.TRISB4 = 0;   // CS2: Gyroscope
    TRISDbits.TRISD6 = 0;   // CS3: Magnetometer
    LATBbits.LATB3 = 1;
    LATBbits.LATB4 = 1;
    LATDbits.LATD6 = 1;
    
    // Port configuration: SPI Lines
    TRISAbits.TRISA1 = 1; // RA1?RPI17 MISO
    TRISFbits.TRISF12 = 0; //RF12?RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13?RP109 MOSI
    
    // Remapping of SPI1 to pins of Microbus1
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) ? RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) ? RF13;
    RPOR11bits.RP108R = 0b000110; // SCK1;
    
    // SPI Configutation
    SPI1CON1bits.MSTEN = 1;      // Master mode
    SPI1CON1bits.MODE16 = 0;     // 8-bit mode
    SPI1CON1bits.CKP = 1;        // Idle value of clock
    // [Prescaler] FSCK = (FCY)/(PPR * SPR) = 72M/(4*3) = 6 MHz
    SPI1CON1bits.PPRE = 0b10;    // Primary prescaler 4:1
    SPI1CON1bits.SPRE = 0b101;   // Secondary prescaler 3:1
    SPI1STATbits.SPIEN = 1;     // Enable SPI
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag
}

void magnetometer_config(){

    int sleep_mode_addr = 0x4B;
    int sleep_mode_value = 0x01;
    int active_mode_addr = 0x4C;
    int active_mode_value = 0x00;

    // Make magnetometer switch to Sleep Mode
    LATDbits.LATD6 = 0;
    spi_write(sleep_mode_addr);
    spi_write(sleep_mode_value);
    LATDbits.LATD6 = 1;
    // Wait 2 ms
    tmr_wait_ms(TIMER1, 2);

    // Make magnetometer go to active mode
    LATDbits.LATD6 = 0;
    spi_write(active_mode_addr);
    spi_write(active_mode_value);
    LATDbits.LATD6 = 1;
    // Wait 2 ms
    tmr_wait_ms(TIMER1, 2);
}
