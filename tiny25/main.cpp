/*
 * GccApplication1.cpp
 *
 * Created: 17/05/2014 10:26:14 pm
 *  Author: raspo
 */ 


#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    while(1)
    {
        //TODO:: Please write your application code 
		PORTB |= 6;
		_delay_ms(1000);
		PORTB &= ~4;
		_delay_ms(1000);
		PORTB &= ~2;
		_delay_ms(1000);		
    }
}
