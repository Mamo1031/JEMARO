#ifndef SPI_FUNCTIONS_H
#define SPI_FUNCTIONS_H

#include "config.h"
#include "timer_functions.h"

// SPI related functions
void spi_setup();
unsigned int spi_write(unsigned int data);
void magnetometer_config();

#endif
