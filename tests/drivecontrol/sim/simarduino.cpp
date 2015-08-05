/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

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





