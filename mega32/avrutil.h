
// Routines to set and clear bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup_watchdog(int ii);
void system_sleep();

#include <avr/sleep.h>
#include <avr/interrupt.h>
