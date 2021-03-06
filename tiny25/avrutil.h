#ifndef _AVR_UTIL_H_
#define _AVR_UTIL_H_

// for attiny25
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif


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

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;

#endif
