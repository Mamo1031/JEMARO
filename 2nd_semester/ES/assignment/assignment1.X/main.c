/*
 * File:   main.c
 * Author: Team 1
 *
 * Yui Momiyama
 * Mamoru Ota
 * Paul Pham Dang
 * Waleed Elfieky
 * 
 * Created on April 7, 2025, 10:33 AM
 * 
 * Description: This file implements the main program flow for the magnetometer
 * data acquisition and transmission system. It reads magnetometer data at 25Hz,
 * calculates moving averages, and transmits the results via UART.
 * 
 * Requirements implemented:
 * 1. Simulate algorithm with 7ms execution at 100Hz
 * 4. Acquire magnetometer data at 25Hz
 * 5. Calculate average of last 5 measurements
 * 6. Send data to UART using specified protocol
 */

#include "xc.h"
#include "timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uart.h"
#include "spi.h"
#include "stdlib.h"




//#define RX_BUFFER_SIZE 128
// LED alias definitions for clarity in code.
#define LED2 LATGbits.LATG9


extern volatile char rxBuffer[RX_BUFFER_SIZE];
extern volatile uint16_t rxIndex;
extern volatile uint8_t rxStringReady;
extern uint8_t currentRate; // Initialize with 5 Hz as required

unsigned int uart_period_ms = 200;
const unsigned int yaw_period_ms = 200;

char localCopy[RX_BUFFER_SIZE];

void algorithm() {
    // Simulate algorithm execution (7ms CPU load)
    tmr_wait_ms(TIMER2, 7);
}

/**
 * @brief Main program entry point
 * 
 * Initializes peripherals and executes the main control loop:
 * - Runs at 100Hz (10ms period)
 * - Acquires magnetometer data at 25Hz (every 40ms)
 * - Calculates and transmits averages at 5Hz (every 200ms)
 * 
 * @return int Standard program return value (unused)
 */
int main(void) {
    // Initialize hardware
    // Disable analog inputs for digital IO use
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    // Configure LED pins as outputs
    TRISGbits.TRISG9 = 0; // LED2 on RG9

    // Enable global interrupts
    INTCON2bits.GIE = 1;

    // Set initial LED states
    LED2 = 1; // LED2 initially on

    // Initialize communication peripherals
    spi_setup(); // Configure SPI for magnetometer

    UART_Initialize(); // Configure UART for data transmission

    magnetometer_config(); // Set up magnetometer with 25Hz data rate

    // Configure timers
    tmr_setup_period(TIMER1, 10); // TIMER1: 100Hz main loop timing (10ms)

    // Main loop control variables

    // Tracks elapsed time in main loop
    int tmr_counter_led = 0;
    int tmr_counter_magnetometer = 0;
    int tmr_counter_uart = 0;
    int tmr_counter_yaw = 0;

    char mag_message[32]; // Buffer for MAG message
    char yaw_message[20]; // Buffer for YAW message

    // Main control loop
    while (1) {

        if (rxStringReady) {
            strcpy(localCopy, (char*) rxBuffer); // Copy safely
            rxStringReady = 0; // Reset flag AFTER copy
            rxIndex = 0; // Also reset index here (not inside ISR)

            UART_SendString("You entered: ");
            UART_SendString(localCopy);
            UART_SendString("\r\n");
            process_uart_command(localCopy);
        }

        // Simulate algorithm execution (7ms CPU load)
        algorithm();

        // Check if it's time to toggle the LED
        if (tmr_counter_led == 500) {
            LED2 = !LED2; // Toggle LED2 state
            tmr_counter_led = 0;
        }

        // Acquire magnetometer data at 25Hz (every 40ms)
        if (tmr_counter_magnetometer == 40) {
            acquire_magnetometer_data();
            tmr_counter_magnetometer = 0;
        }

        // Calculate averages of last 5 measurements once per main loop iteration
        double x_mag = calculate_average(x_values, ARRAY_SIZE);
        double y_mag = calculate_average(y_values, ARRAY_SIZE);
        double z_mag = calculate_average(z_values, ARRAY_SIZE);

        // Process and transmit MAG data at configurable rate (xx Hz)
        if (currentRate > 0) {
            uart_period_ms = (1.0 / currentRate)*1000.0;
            if (tmr_counter_uart % uart_period_ms == 0) {
                sprintf(mag_message, "$MAG,%.2f,%.2f,%.2f*\r\n", x_mag, y_mag, z_mag);
                UART_SendString(mag_message);
                tmr_counter_uart = 0;
            }
        }

        // Send YAW data always at 5Hz (200ms)
        if (tmr_counter_yaw == yaw_period_ms) {
            // Compute the angle to the magnetic North
            double angle = atan2(y_mag, x_mag) * (180.0 / M_PI); // Convert to degrees
            sprintf(yaw_message, "$YAW,%.2f*\r\n", angle);
            UART_SendString(yaw_message);
            tmr_counter_yaw = 0;
        }

        // Maintain precise 100Hz loop timing
        tmr_wait_period(TIMER1); // Wait for timer period completion
        tmr_counter_led += 10; // Increment counters by 10ms
        tmr_counter_magnetometer += 10;
        tmr_counter_uart += 10;
        tmr_counter_yaw += 10;
    }

    // Return statement (never reached in this embedded application)
    return 0;
}
