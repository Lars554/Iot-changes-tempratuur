/*
 * testdingmicrochip.c
 *
 * Created: 20/01/2026 11:27:21
 * Author : larsd
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

void set_pin()
{
	
	
	
	
}
void set_pwm()
{
	TCCR0A |= (1 << WGM00) | (1 << WGM01);
	TCCR0A |= (1 << COM0A1);
	TCCR0B |= (1 << CS01) | (1 << CS00);
}



int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

