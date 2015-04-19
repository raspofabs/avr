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

int main(void) {
	setup_watchdog(0); // approximately 0.016 seconds sleep
	//setup_watchdog(8); // approximately 0.5 seconds sleep

	while(1)
	{
		if( sleeping ) {
			// set pin state
			DDRB &= ~OUT_PINS; // all outputs turned off in sleep
			DDRB &= ~IN_PINS;
			PORTB &= ~( OUT_PINS + IN_PINS );
			_delay_ms(dtime);
			if( PINB & 16 ) {
				DDRB |= OUT_PINS; // all outputs turned off in sleep
				PORTB |= 1;
				while( PINB & 16 ) {
					_delay_ms(10);
				}
				sleeping = false;
				continue;
			}
			//system_sleep();
		} else {
			// set pin state
			DDRB |= OUT_PINS;
			DDRB &= ~IN_PINS;

			//TODO:: Please write your application code 
			PORTB |= DDRB;
			_delay_ms(dtime);
			PORTB &= ~1;
			_delay_ms(dtime);
			PORTB &= ~2;
			_delay_ms(dtime);		
			PORTB &= ~4;
			_delay_ms(dtime);
			PORTB &= ~8;
			_delay_ms(dtime);		
			//PORTB &= ~16;
			//_delay_ms(dtime);		
			//PORTB &= ~32;
			//_delay_ms(dtime);		
			if( PINB & 16 ) {
				PORTB |= 2;
				while( PINB & 16 ) {
					_delay_ms(10);
				}
				sleeping = true;
				continue;
			}
		}
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
  f_wdt=1;  // set global flag
}
