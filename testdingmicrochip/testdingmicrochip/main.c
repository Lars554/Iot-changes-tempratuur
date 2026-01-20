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

#define SCL_CLOCK 100000L 

void init_gpio()
{
	DDRC |= (1<<DDRC4) | (1<<DDRC5); // pc4 is sda en pc5 is scl
	DDRC &= ~(1<<DDRC0); //potentie meter voor tempratuur te zetten
	DDRC &= ~(1<<DDRC1); //potentiemeter als sensor
	DDRB |= (1<<DDRB1) | (1<<DDRB2);// licht op B1 is blauw en B2 rood
	DDRD &= ~(1<<DDRD2); //knop
	PORTC &= ~((1 << PORTC0) | (1 << PORTC1));

	
}
void init_pwm()
{
	TCCR1A = (1 << WGM10);
	TCCR1B = (1 << WGM12);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	TCCR1B |= (1 << CS11) | (1 << CS10);
}

void ADC_init(void)
{
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void i2c_init(void) {
    // Bit rate register instellen
    TWSR = 0x00;  // Prescaler = 1
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;

    // TWI inschakelen
    TWCR = (1<<TWEN);
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

uint8_t i2c_start(uint8_t address) {
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT); // start conditie
    while (!(TWCR & (1<<TWINT))); // wacht op complete

    TWDR = address; // adres + R/W bit
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));

    uint8_t twst = TWSR & 0xF8;
    if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) return 1;
    return 0;
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWINT);
}


int main(void)
{
	init_gpio();
	init_pwm();
	ADC_init();
    /* Replace with your application code */
    while (1) 
    {
		OCR1A = ADC_read(0) >> 2;
		OCR1B = ADC_read(1) >> 2;
    }
}

