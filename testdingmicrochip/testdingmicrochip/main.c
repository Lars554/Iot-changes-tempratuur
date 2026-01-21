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
#include <stdlib.h>
#include <math.h>


#define START           0x08
#define MT_SLA_ACK      0x18
#define MT_DATA_ACK     0x28
#define LCD_ADDR 0x27

#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

char togel = '0';

void init_gpio()
{
	DDRC &= ~(1<<DDRC0); //potentie meter voor tempratuur te zetten op pc0
	DDRC &= ~(1<<DDRC1); //potentiemeter als sensor op pc1
	DDRD |= (1<<DDRD5) | (1<<DDRD6);// licht op D5 is blauw en D6 rood
	DDRD &= ~(1<<DDRD2); // input
	PORTD |= (1 << PORTD2);
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

void i2c_init(void)
{
	TWSR0 = 0x00;          // prescaler = 1
	TWBR0 = 72;            // 100kHz @ 16MHz
	TWCR0 = (1<<TWEN);     // enable TWI
}

	
uint16_t ADC_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint8_t dht_read_byte(void)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; i++) {
        // wacht tot lijn hoog (start van bit)
        uint16_t count = 0;
        while (!(PIND & (1 << PD2))) {
            if (++count > 10000) return 0xFF; // fout
        }

        _delay_us(40); // midden van bit

        // lees bit
        if (PIND & (1 << PD2)) result |= (1 << (7 - i));

        // wacht tot lijn laag (einde bit)
        count = 0;
        while ((PIND & (1 << PD2))) {
            if (++count > 10000) return 0xFF; // fout
        }
    }
    return result;
}

int8_t read_dht11_temperature(void)
{
    // Startsignaal
    DDRD |= (1 << PD2);
    PORTD &= ~(1 << PD2);   // lijn laag >18ms
    _delay_ms(20);

    PORTD |= (1 << PD2);    // lijn hoog
    _delay_us(40);           // korte wacht
    DDRD &= ~(1 << PD2);    // lijn als input

    // Wacht op respons van sensor (~80us laag + ~80us hoog)
    uint16_t timeout = 0;
    while ((PIND & (1 << PIND2))) if (++timeout > 10000) return  -1;
    timeout = 0;
    while (!(PIND & (1 << PD2))) if (++timeout > 10000) return -2;
    timeout = 0;
    while ((PIND & (1 << PD2))) if (++timeout > 10000) return -3;

    // 5 bytes uitlezen
    uint8_t hum_int  = dht_read_byte();
    uint8_t hum_dec  = dht_read_byte();
    uint8_t temp_int = dht_read_byte();
    uint8_t temp_dec = dht_read_byte();
    uint8_t checksum = dht_read_byte();

    // Controleer checksum
    if ((hum_int + hum_dec + temp_int + temp_dec) != checksum) return -5;

    return temp_int;
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

uint8_t i2c_start(uint8_t address)
{
	// START
	TWCR0 = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != START) return 1;

	// SLA+W
	TWDR0 = address;
	TWCR0 = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != MT_SLA_ACK) return 2;

	return 0;
}

uint8_t i2c_write(uint8_t data)
{
	TWDR0 = data;
	TWCR0 = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR0 & (1<<TWINT)));
	if ((TWSR0 & 0xF8) != MT_DATA_ACK) return 1;
	return 0;
}

void i2c_stop(void)
{
	TWCR0 = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	_delay_us(10);
}


void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble & 0xF0) | LCD_BL;
    if (rs) data |= LCD_RS;

    i2c_start(LCD_ADDR << 1);
    i2c_write(data | LCD_EN);
    _delay_us(1);
    i2c_write(data & ~LCD_EN);
    i2c_stop();
}

void lcd_write_byte(uint8_t value, uint8_t rs)
{
    lcd_write_nibble(value & 0xF0, rs);
    lcd_write_nibble(value << 4, rs);
    _delay_ms(2);
}

void lcd_command(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
}

void lcd_data(uint8_t data)
{
    lcd_write_byte(data, 1);
}

void lcd_init(void)
{
    _delay_ms(50);

    lcd_write_nibble(0x30, 0);
    _delay_ms(5);
    lcd_write_nibble(0x30, 0);
    _delay_us(150);
    lcd_write_nibble(0x30, 0);
    lcd_write_nibble(0x20, 0); // 4-bit mode

    lcd_command(0x28); // 4-bit, 2 lines
    lcd_command(0x0C); // display ON
    lcd_command(0x06); // entry mode
    lcd_command(0x01); // clear
    _delay_ms(2);
}

void lcd_print(char *s)
{
    while (*s)
        lcd_data(*s++);
}

void lcd_clear(void)
{
    lcd_command(0x01);
    _delay_ms(2);
}

void UART_send_string(char *s)
{
	while (*s) UART_send_char(*s++);
}


int main(void)
{
	init_gpio();
	init_pwm();
	ADC_init();
	init_uart();
	i2c_init();
    lcd_init();
	_delay_ms(100);
	
	while (1)
	{
		lcd_command(0x80);
		//int8_t tempC = read_dht11_temperature();
		//char buf[8];

		//lcd_command(0x80);
		//lcd_print("TEMP: ");
		//itoa(tempC, buf, 10);
		//lcd_print(buf);
		//lcd_print("C ");
		 if (togel == '1') {
            int8_t tempC = read_dht11_temperature();
			char buf[8];
			lcd_print("TEMP: ");
			itoa(tempC, buf, 10);
			lcd_print(buf);
			lcd_print("C ");
			UART_send_number(tempC);
        } else {
            // Analoge sensor uitlezen en omzetten naar °C
			char buf[8];
            uint16_t adc = ADC_read(0);
            float Vref = 5.0;
            float R_pullup = 10000.0;
            float Vadc = adc * Vref / 1023.0;
            float R_thermistor = R_pullup * (Vref / Vadc - 1);

            // Beta formule NTC
            float B = 3950.0;
            float R0 = 10000.0;
            float T0 = 25.0 + 273.15; // Kelvin
            float tempK = 1.0 / (1.0/T0 + (1.0/B) * log(R_thermistor / R0));
            float tempC = tempK - 273.15;

            lcd_print("TEMP: ");
            itoa((int)tempC, buf, 10);
            lcd_print(buf);
            lcd_print("C ");
			
			UART_send_number(tempC);
			
        }


		OCR0A = ADC_read(0) >> 2;
		OCR0B = ADC_read(1) >> 2;

		_delay_ms(200);
	}

}