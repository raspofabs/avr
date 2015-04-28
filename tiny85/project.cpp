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
volatile bool f_wdt = 0;

bool sleeping = false;
void setup_watchdog(int ii);
void system_sleep();
void setup_flashing();

const int OUT_PINS = 1+2+4+8;
const int IN_PIN = 16;

void updateWDTLED() {
	//if( f_wdt ) PORTB |= 8;
	//else PORTB &= ~8;
}

unsigned char BRIGHTR = 32;
unsigned char BRIGHTG = 32;
unsigned char BRIGHTB = 32;
long effect_time;

void UpdateWithFade( int r, int g, int b, int BITS ) {
	int SHIFT = BITS - 8; // how many bits to reduce fade by to put it in 0-255
	int fadetime = 1<<BITS;
	int halffade = fadetime/2;
	int subfade = fadetime-1;
	int fade = effect_time & subfade;
	if( fade >= halffade ) fade = subfade - fade;
	fade >>= SHIFT; // bring it down (or up?)

	BRIGHTR = r * fade >> 8;
	BRIGHTG = g * fade >> 8;
	BRIGHTB = b * fade >> 8;
}

void UpdateDirect( int r, int g, int b ) {
	BRIGHTR = r;
	BRIGHTG = g;
	BRIGHTB = b;
}

template <int rate, int fadeRate, bool interp, int N>
void UpdateWithArray( int (&RGB)[N] ) {
	int et = effect_time / rate;
	const int numCols = N / 3;
	const int selected = et % numCols;
	int r = RGB[selected*3+0];
	int g = RGB[selected*3+1];
	int b = RGB[selected*3+2];
	if( interp ) {
		const int selected2 = (et+1) % numCols;
		const int t = 256 * ( effect_time % rate ) / rate;
		int r2 = RGB[selected2*3+0];
		int g2 = RGB[selected2*3+1];
		int b2 = RGB[selected2*3+2];
		r = ( r * (256 - t) + r2 * t ) >> 8;
		g = ( g * (256 - t) + g2 * t ) >> 8;
		b = ( b * (256 - t) + b2 * t ) >> 8;
	}

	if( fadeRate ) {
		UpdateWithFade(r,g,b, fadeRate);
	} else {
		UpdateDirect(r,g,b);
	}
}


void UpdateRainbow() {
	int RGB[] = {
		255,0,0,
		255,255,0,
		0,255,0,
		0,255,255,
		0,0,255,
		255,0,255,
	};
	UpdateWithArray<64,10,true>(RGB);
}
void UpdateRainbow2() {
	int RGB[] = {
		255,0,0,
		255,255,0,
		0,255,0,
		0,255,255,
		0,0,255,
		255,0,255,
	};
	UpdateWithArray<32,10,false>(RGB);
}

void UpdateFire() {
	int RGB[] = {
		150,30,0,
		150,30,0,
		180,130,0,
		150,30,0,
		255,255,0,
		150,30,0,
		255,205,0,
		255,255,0,
		255,205,0,
		150,80,0,
		150,30,0,
		255,205,0,
	};
	UpdateWithArray<8,8,true>(RGB);
}

void UpdateIce() {
	int RGB[] = {
		0,200,255,
		0,180,100,
		0,0,100,
		0,50,200,
		0,100,240,
		0,80,255,
		0,130,240,
		0,200,100,
		150,0,200,
		50,100,200,
		0,0,100,
		0,0,150,
		80,80,200,
		50,130,200,
	};
	UpdateWithArray<16,0,true>(RGB);
}

void UpdateNature() {
	int RGB[] = {
		100,200,50,
		150,200,50,
		100,250,80,
		100,200,50,
		200,250,120,
		100,200,100,
		170,170,70,
		230,150,100,
		120,250,120,
	};
	UpdateWithArray<32,0,false>(RGB);
}

void UpdateTest() {
	int RGB[] = {
		255,0,255,
		0,255,255,
		255,255,0,
	};
	UpdateWithArray<32,11,false>(RGB);
}
void UpdateTest2() {
	int RGB[] = {
		255,255,255,
		0,0,0,
	};
	UpdateWithArray<128,11,true>(RGB);
}

void UpdateBlink() {
	effect_time %= 16;
	if( effect_time < 10 ) {
		BRIGHTR = 180;
		BRIGHTG = 180;
		BRIGHTB = 180;
	} else {
		BRIGHTR = 0;
		BRIGHTG = 0;
		BRIGHTB = 0;
	}
}

bool buttonPressed() {
	static int downFor;
	static int upFor;
	static bool debounce;
	if( PINB & IN_PIN ) {
		upFor += 1;
		downFor = 0;
	} else {
		downFor += 1;
		upFor = 0;
	}
	if( downFor > 3 ) {
		debounce = true;
	}
	if( upFor > 3 && debounce ) {
		debounce = false;
		return true;
	}
	return false;
}

int main(void) {
	// set pin state
	DDRB |= OUT_PINS;
	DDRB &= ~IN_PIN;
	PORTB = IN_PIN;
	for( int i = 0; i < 10; ++i ) {
		PORTB ^= OUT_PINS;
		_delay_ms(100);
	}

	setup_watchdog(6); // approximately 1024ms timer
	setup_flashing();

	int mode = 0;
	while(1)
	{
		if( buttonPressed() ) {
			mode += 1;
		}
		//int old = PORTB & ~3;
		//PORTB = old | (count&3);
		//count += 1;
		updateWDTLED();
		_delay_ms(5);
		effect_time += 1;
		switch( mode ) {
			case 0: UpdateRainbow(); break;
			case 1: UpdateRainbow2(); break;
			case 2: UpdateFire(); break;
			case 3: UpdateIce(); break;
			case 4: UpdateNature(); break;
			case 5: UpdateTest(); break;
			case 6: UpdateTest2(); break;
			//case 7: UpdateBlink(); break;
			default: mode = 0; break;
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
  sei();
}

void setup_flashing() {
	//TCCR0A = 0x01; // mode : PWM
	//TCCR0B = 0x0; // clock source none, stop clock
	//TCCR0B = 0x01; // clock source CLK, start timer

	TCCR0A = 0x02; // mode : clear timer on compare match
	TCCR0B = 0x01; // clock source CLK, start timer
	TCCR0B = 0x02; // clock source CLK/8, start timer
	TCCR0B = 0x03; // clock source CLK/64, start timer
	TCCR0B = 0x04; // clock source CLK/256, start timer
	TCCR0B = 0x05; // clock source CLK/1024, start timer
	TCCR0B = 0x02;
	OCR0A = 0x20; // counting target
	//TIFR |= 0x01; // clear the flag
	TIMSK = 0x10; // TC0 compare match A interrupt variable
	sei();
}

unsigned char PWMTIME = 0;
//unsigned char RGBTIME = 1;

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(TIMER0_COMPA_vect)
{
	//RGBTIME = ((RGBTIME<<1)|(RGBTIME>>2))&7;
	PWMTIME += 4;
	int r = PWMTIME<BRIGHTR;
	int g = PWMTIME<BRIGHTG;
	int b = PWMTIME<BRIGHTB;
	//PORTB = (PORTB&~7)|(RGBTIME&(r|(g<<1)|(b<<2)));
	PORTB = (PORTB&~7)|((b|(g<<1)|(r<<2)));
	//PORTB ^= 4; 
  //TIFR |= 0x01; // clear the flag
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt ^= 1;  // set global flag
}
