#include "../objects.h"
#include "simarduino.h"


AConsole Console;



unsigned long millis(void){
  return Timer.millis();
}

unsigned long micros(void){
  return Timer.millis() * 1000;
}

void delay(unsigned long){
}

void delayMicroseconds(unsigned int us){
}

void pinMode(uint8_t pin, uint8_t mode){
}

void digitalWrite(uint8_t pin, uint8_t state){
}

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration){
}

void noTone(uint8_t _pin){
}





