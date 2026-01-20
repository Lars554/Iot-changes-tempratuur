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

#define DC   DDD2
#define RST  DDD3
#define CS   DDD4
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define SPIF0 7 

void init_gpio()
{
	DDRC &= ~(1<<DDRC0); //potentie meter voor tempratuur te zetten
	DDRC &= ~(1<<DDRC1); //potentiemeter als sensor
	DDRD |= (1<<DDRD5) | (1<<DDRD6);// licht op D5 is blauw en D6 rood
	DDRD &= ~(1<<DDRD2); //knop
	PORTC &= ~((1 << PORTC0) | (1 << PORTC1));
	DDRB |= (1<<DDRB0)|(1<<DDRB)|(1<<DDRB2); // spi
	SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	
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

// SPI init voor ATmega328PB
void spi_init(void){
    // PB0 = MOSI, PB1 = SCK, PB2 = SS als output
    DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2);

    // SPI master, fosc/16
    SPCR0 = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
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

// SPI write byte
void spi_write(uint8_t data){
    SPDR0 = data;
    while(!(SPSR0 & (1<<SPIF0)));
}

// OLED control functions
void oled_command(uint8_t cmd){
    PORTD &= ~(1<<DC);  // DC=0 -> command
    PORTD &= ~(1<<CS);  // CS low
    spi_write(cmd);
    PORTD |= (1<<CS);   // CS high
}

void oled_data(uint8_t data){
    PORTD |= (1<<DC);   // DC=1 -> data
    PORTD &= ~(1<<CS);  // CS low
    spi_write(data);
    PORTD |= (1<<CS);   // CS high
}

// Reset OLED
void oled_reset(void){
    PORTD &= ~(1<<RST);
    _delay_ms(10);
    PORTD |= (1<<RST);
    _delay_ms(10);
}

// Basic SSD1306 init sequence
void oled_init(void){
    oled_reset();
    _delay_ms(50);

    oled_command(0xAE); // Display off
    oled_command(0xD5); oled_command(0x80); // Set display clock
    oled_command(0xA8); oled_command(0x3F); // Set multiplex
    oled_command(0xD3); oled_command(0x00); // Set display offset
    oled_command(0x40); // Start line
    oled_command(0x8D); oled_command(0x14); // Charge pump
    oled_command(0x20); oled_command(0x00); // Memory mode
    oled_command(0xA1); // Segment remap
    oled_command(0xC8); // COM scan dec
    oled_command(0xDA); oled_command(0x12); // COM pins
    oled_command(0x81); oled_command(0xCF); // Contrast
    oled_command(0xD9); oled_command(0xF1); // Pre-charge
    oled_command(0xDB); oled_command(0x40); // VCOM detect
    oled_command(0xA4); // Resume display RAM
    oled_command(0xA6); // Normal display
    oled_command(0xAF); // Display on
}

// Clear OLED (writes 0 to all pixels)
void oled_clear(void){
    for(uint16_t i=0;i<OLED_WIDTH*OLED_HEIGHT/8;i++){
        oled_data(0x00);
    }
}

// Draw a simple character (8x8 font)
void oled_draw_char(uint8_t c){
    // Dummy 8x8 font: just a block for example
    for(uint8_t i=0;i<8;i++){
        oled_data(0xFF);
    }
}

// Draw "Hello" on screen
void oled_draw_hello(void){
    const char* str = "HELLO";
    while(*str){
        oled_draw_char(*str++);
    }
}

int main(void)
{
	init_gpio();
	init_pwm();
	ADC_init();
	spi_init();
    oled_init();
    oled_clear();
    _delay_ms(100);
    /* Replace with your application code */
    while (1) 
    {
		OCR0A = ADC_read(0) >> 2;
		OCR0B = ADC_read(1) >> 2;
		_delay_ms(200);
    }
}

