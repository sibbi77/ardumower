#include "../objects.h"
#include "simarduino.h"


AConsole Console;



unsigned long millis(void){
  return Robot.millis();
}

unsigned long micros(void){
  return 1;
}

void delay(unsigned long){
}

void delayMicroseconds(unsigned int us){
}

void pinMode(uint8_t pin, uint8_t mode){
}

void digitalWrite(uint8_t pin, uint8_t state){
}




