#include <Arduino.h>

void init_gpio()
{
    DDRD |= (1 << DDD0);
}
 void setup() {
    init_gpio();
 }

void loop() {
    PORTD ^= (1 << PORTD0);
    delay(1000);
}
