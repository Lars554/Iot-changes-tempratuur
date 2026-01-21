/*
 * testdingmicrochip.c
 *
 * Created: 20/01/2026 11:27:21
 * Author : larsd
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>



void init_gpio()
{
	DDRC &= ~(1<<DDRC0); //potentie meter voor tempratuur te zetten
	DDRC &= ~(1<<DDRC1); //potentiemeter als sensor
	DDRD |= (1<<DDRD5) | (1<<DDRD6);// licht op D5 is blauw en D6 rood
	DDRD &= ~(1<<DDRD2); //knop
	PORTC &= ~((1 << PORTC0) | (1 << PORTC1));
	
}
void init_pwm()
{
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
	TCCR0B = (1 << CS01) | (1 << CS00);
}

void ADC_init(void)
{
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void init_uart(void)
{
	UBRR0 = 103;
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	// RXCIE0 = enable RX interrupt
	// RXEN0 = enable receiver
	// TXEN0 = enable transmitter
}

uint16_t ADC_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void UART_send_char(char c) {
	while (!(UCSR0A & (1<<UDRE0))); 
	UDR0 = c;                         
}

void UART_send_number(uint16_t num) {
	char buffer[4];
	itoa(num, buffer, 10);
	for (char *p = buffer; *p; p++) {
		UART_send_char(*p);
	}
	UART_send_char('\n');
}



int main(void)
{
	init_gpio();
	init_pwm();
	ADC_init();
	init_uart();
    _delay_ms(100);
    /* Replace with your application code */
    while (1) 
    { 
		UART_send_char('i');
		OCR0A = ADC_read(0) >> 2;
		OCR0B = ADC_read(1) >> 2;
		_delay_ms(200);
    }
}

