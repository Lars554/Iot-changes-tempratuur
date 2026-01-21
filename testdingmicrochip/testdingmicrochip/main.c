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

#define START           0x08
#define MT_SLA_ACK      0x18
#define MT_DATA_ACK     0x28
#define LCD_ADDR 0x27

#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BL 0x08

#define CS_PIN PD7


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

void i2c_init(void)
{
	TWSR0 = 0x00;          // prescaler = 1
	TWBR0 = 72;            // 100kHz @ 16MHz
	TWCR0 = (1<<TWEN);     // enable TWI
}

void SPI0_MasterInit(void) {
    DDRB |= (1<<DDB3) | (1<<DDB5);  // MOSI, SCK output
    DDRB &= ~(1<<DDB4);              // MISO input
    SPCR0 = (1<<SPE) | (1<<MSTR) | (1<<SPR0); // SPI enable, master, fck/16
}

uint8_t SPI0_MasterTransmit(uint8_t data) {
    SPDR0 = data;
    while (!(SPSR0 & (1<<SPIF)));
    return SPDR0;
}

void CS_Enable(void)  { PORTD &= ~(1<<CS_PIN); }
void CS_Disable(void) { PORTD |= (1<<CS_PIN); }
uint8_t BMP0_ReadRegister(uint8_t reg) {
    uint8_t val;
    CS_Enable();
    SPI0_MasterTransmit(reg | 0x80);  // MSB=1 read
    val = SPI0_MasterTransmit(0x00);
    CS_Disable();
    return val;
}

uint16_t BMP0_Read16(uint8_t reg) {
    uint16_t val = BMP0_ReadRegister(reg);
    val |= (BMP0_ReadRegister(reg+1) << 8);
    return val;
}

int16_t BMP0_ReadS16(uint8_t reg) {
    return (int16_t)BMP0_Read16(reg);
}

// Read raw temperature
int32_t BMP0_ReadTemperatureRaw(void) {
    uint8_t msb = BMP0_ReadRegister(0xFA);
    uint8_t lsb = BMP0_ReadRegister(0xFB);
    uint8_t xlsb = BMP0_ReadRegister(0xFC);
    return ((int32_t)msb << 12) | ((int32_t)lsb << 4) | ((xlsb >> 4) & 0x0F);
}

// Compensate temperature
int32_t BMP0_CompensateTemperature(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 <<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8; // °C *100
    return T;
}

// Read calibration
void BMP0_ReadCalibration(void) {
    dig_T1 = BMP0_Read16(0x88);
    dig_T2 = BMP0_ReadS16(0x8A);
    dig_T3 = BMP0_ReadS16(0x8C);
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

int main(void)
{
	init_gpio();
	init_pwm();
	ADC_init();
	init_uart();
	i2c_init();
    lcd_init();
	SPI0_MasterInit();
    _delay_ms(100);
    /* Replace with your application code */
    while (1) 
    { 
		// BMP280 uitlezen en compensatie toepassen
        int32_t raw = BMP0_ReadTemperatureRaw();
        tempC = BMP0_CompensateTemperature(raw) / 100; // nu °C

        // LCD update zonder clear
        lcd_command(0x80);  // cursor naar begin
        lcd_print("TEMP: ");
        itoa(tempC, buf, 10);
        lcd_print(buf);
        lcd_print("C ");

        // LEDs via ADC
        OCR0A = ADC_read(0) >> 2;
        OCR0B = ADC_read(1) >> 2;

        _delay_ms(500);

	}
}

