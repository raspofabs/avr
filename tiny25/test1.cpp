/*
 * GccApplication1.cpp
 *
 * Created: 17/05/2014 10:26:14 pm
 *  Author: raspo
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "avrutil.h"

const uint32_t dtime = 100;

// Variables for the Sleep/power down modes:
volatile bool f_wdt = 1;

bool sleeping = false;
void setup_watchdog(int ii);
void system_sleep();

const int OUT_PINS = 1+2+4+8;
const int IN_PINS = 16;

void updateWDTLED() {
	if( f_wdt ) PORTB |= 8;
	else PORTB &= ~8;
}

int main(void) {
	setup_watchdog(8); // approximately 0.5 seconds sleep

	// set pin state
	DDRB |= OUT_PINS;
	DDRB &= ~IN_PINS;
	int count = 0;
	while(1)
	{
		int old = PORTB & ~7;
		PORTB = old | (count&7);
		count += 1;
		updateWDTLED();
		_delay_ms(100);
	}
}

// sleep stuff

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {
  
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System actually sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  unsigned char bb;
  //int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  //ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt^=1;  // set global flag
}
