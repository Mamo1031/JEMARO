#ifndef TIMER_FUNCTIONS_H
#define TIMER_FUNCTIONS_H

#include "config.h"

// Timer related functions
void tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer, int ms, int has_interrupt);
int tmr_wait_period(int timer);
void tmr_set_register_values(int timer, int pr_value, int prescaler_bits, int has_interrupt);

#endif
