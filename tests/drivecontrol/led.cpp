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

#include "common.h"
#include "led.h"


LEDControl::LEDControl(){
  nextLEDTime = 0;
  ledSequenceIdx = LED_SEQ_OFF;
  onState = 0;
}

void LEDControl::setup(){
  Console.println(F("LEDControl::setup"));
}

void LEDControl::run(){
  if (millis() < nextLEDTime) return;
  nextLEDTime = millis() + 500;
  switch (ledSequenceIdx){
    case LED_SEQ_OFF:
      driverSetLED(LED_ARDUINO, LOW);
      driverSetLED(LED_DUAL_RED, LOW);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
    case LED_SEQ_GREEN_ON:
      driverSetLED(LED_ARDUINO, HIGH);
      driverSetLED(LED_DUAL_RED, LOW);
      driverSetLED(LED_DUAL_GREEN, HIGH);
      break;
    case LED_SEQ_ORANGE_ON:
      driverSetLED(LED_DUAL_RED, HIGH);
      driverSetLED(LED_DUAL_GREEN, HIGH);
      break;
    case LED_SEQ_ORANGE_BLINK:
      onState = !onState;
      driverSetLED(LED_DUAL_RED, onState);
      driverSetLED(LED_DUAL_GREEN, onState);
      break;
    case LED_SEQ_RED_BLINK:
      onState = !onState;
      driverSetLED(LED_ARDUINO, onState);
      driverSetLED(LED_DUAL_RED, onState);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
    case LED_SEQ_RED_ON:
      driverSetLED(LED_DUAL_RED, HIGH);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
  };
}

void LEDControl::driverSetLED(int LEDidx, bool state){
}

void LEDControl::playSequence(int sequenceIdx){
  ledSequenceIdx = sequenceIdx;
  nextLEDTime = 0;
}


