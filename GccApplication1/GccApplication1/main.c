#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// --- UART ---
void UART_init(unsigned int baud) {
	unsigned int ubrr = F_CPU/16/baud - 1;
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<TXEN0);              // alleen TX
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8N1
}

void UART_tx(char c) {
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void UART_print(const char *s) {
	while(*s) UART_tx(*s++);
}

// --- SPI0 ---
void SPI_init(void) {
	DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2); // MOSI,SCK,CS output
	DDRB &= ~(1<<PB4);                  // MISO input
	SPCR0 = (1<<SPE0)|(1<<MSTR0)|(1<<SPR00); // enable, master, fosc/16
	PORTB |= (1<<PB2); // CS high
}

uint8_t SPI_transfer(uint8_t data) {
	SPDR0 = data;
	while(!(SPSR0 & (1<<SPIF0)));
	return SPDR0;
}

#define CS_LOW()   PORTB &= ~(1<<PB2)
#define CS_HIGH()  PORTB |= (1<<PB2)

// --- BMP280 ---
uint8_t bmp280_read8(uint8_t reg) {
	CS_LOW();
	SPI_transfer(reg | 0x80);
	uint8_t val = SPI_transfer(0x00);
	CS_HIGH();
	return val;
}

void bmp280_write8(uint8_t reg, uint8_t val) {
	CS_LOW();
	SPI_transfer(reg & 0x7F);
	SPI_transfer(val);
	CS_HIGH();
}

int32_t bmp280_read_temp_raw(void) {
	CS_LOW();
	SPI_transfer(0xFA | 0x80);
	uint8_t msb = SPI_transfer(0x00);
	uint8_t lsb = SPI_transfer(0x00);
	uint8_t xlsb = SPI_transfer(0x00);
	CS_HIGH();
	return ((int32_t)msb<<12) | ((int32_t)lsb<<4) | (xlsb>>4);
}

// Kalibratie (alleen temperatuur)
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
int32_t t_fine;

void bmp280_read_calib(void) {
	dig_T1 = bmp280_read8(0x88) | (bmp280_read8(0x89)<<8);
	dig_T2 = bmp280_read8(0x8A) | (bmp280_read8(0x8B)<<8);
	dig_T3 = bmp280_read8(0x8C) | (bmp280_read8(0x8D)<<8);
}

int32_t bmp280_compensate_T_int32(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T; // 0.01 °C
}

// --- MAIN ---
int main(void) {
	UART_init(9600);
	SPI_init();

	// check chip ID
	uint8_t id = bmp280_read8(0xD0);
	if(id != 0x58) {
		UART_print("BMP280 niet gevonden\r\n");
		while(1);
	}

	bmp280_read_calib();
	bmp280_write8(0xF4, 0x27); // temp oversampling x1, normaal mode

	char buf[32];
	while(1) {
		int32_t raw = bmp280_read_temp_raw();
		int32_t temp = bmp280_compensate_T_int32(raw);
		sprintf(buf, "Temp: %ld.%02ld C\r\n", temp/100, temp%100);
		UART_print(buf);
		_delay_ms(1000);
	}
}