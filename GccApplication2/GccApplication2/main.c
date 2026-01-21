/*
 * GccApplication2.c
 *
 * Created: 21/01/2026 9:55:02
 * Author : larsd
 */ 

#include <avr/io.h>


#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

void uart_init(void) {
	UBRR0H = (UBRR_VALUE >> 8);
	UBRR0L = UBRR_VALUE;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0); // Enable TX/RX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void uart_send_char(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void uart_send_string(const char *s) {
	while (*s) uart_send_char(*s++);
}

char uart_receive_char(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

int main(void) {
	uart_init();

	while (1) {
		uart_send_string("Hello via UART!");
		_delay_ms(1000);

		if (UCSR0A & (1 << RXC0)) {
			char c = uart_receive_char();
			uart_send_char(c); // Echo back
		}
	}
}