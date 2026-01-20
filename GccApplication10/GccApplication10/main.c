#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#define __AVR_ATmega328PB__
#include <avr/io.h>

#define LCD_ADDR 0x27  // Pas aan als jouw LCD een ander adres heeft
#define SCL_CLOCK 100000L  // 100kHz I2C

// --- I2C init ---
void i2c_init(void) {
	TWSR = 0x00;                  // prescaler = 1
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
	TWCR = (1<<TWEN);             // TWI inschakelen
}

// --- I2C start ---
uint8_t i2c_start(uint8_t address) {
	TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT);
	while (!(TWCR & (1<<TWINT)));

	TWDR = address;
	TWCR = (1<<TWEN) | (1<<TWINT);
	while (!(TWCR & (1<<TWINT)));

	uint8_t twst = TWSR & 0xF8;
	if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) return 1;
	return 0;
}

// --- I2C stop ---
void i2c_stop(void) {
	TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWINT);
	_delay_us(10);
}

// --- I2C write ---
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1<<TWEN) | (1<<TWINT);
	while (!(TWCR & (1<<TWINT)));
}

// --- LCD helper functies ---
void lcd_send(uint8_t data) {
	i2c_start(LCD_ADDR << 1);  // adres + write
	i2c_write(data);
	i2c_stop();
}

void lcd_init(void) {
	_delay_ms(50);  // LCD power-up delay
	lcd_send(0x33); // init
	lcd_send(0x32); // init
	lcd_send(0x28); // 4-bit mode, 2 lijnen
	lcd_send(0x0C); // display aan, cursor uit
	lcd_send(0x06); // cursor vooruit
	lcd_send(0x01); // scherm leeg
	_delay_ms(2);
}

void lcd_write_char(char c) {
	lcd_send(c);
}

void lcd_write_string(const char *str) {
	while (*str) {
		lcd_write_char(*str++);
	}
}

// --- Main ---
int main(void) {
	i2c_init();
	lcd_init();

	while (1) {
		lcd_write_string("Hallo ATmega328PB!");
		_delay_ms(1000);
	}
}
