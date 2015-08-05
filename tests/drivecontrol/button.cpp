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

#include "objects.h"
#include "button.h"



ButtonControl::ButtonControl(){
  nextButtonTime = 0;
  beepCounter = tempBeepCounter = 0;
  pressed = false;
}

void ButtonControl::setup(){
  // button
  Console.println(F("ButtonControl::setup"));
}

void ButtonControl::resetBeepCounter(){
  beepCounter = 0;
}

void ButtonControl::setBeepCount(int count){
  nextButtonTime = millis() + 2000;
  beepCounter = count;
  Console.print(F("BUTTON beeps "));
  Console.println(beepCounter);
  tempBeepCounter = 0;
  pressed = true;
}

// call this in main loop
void ButtonControl::run(){
  //if (pressed) Console.println(F("BUTTON pressed"));
  pressed = driverButtonPressed();

  if ( (!pressed) && (tempBeepCounter > 0) ){
    setBeepCount(tempBeepCounter);
  }
  if (millis() >= nextButtonTime){
    if (pressed) {
      nextButtonTime = millis() + 1000;
      Buzzer.play(BC_SHORT);
      tempBeepCounter++;
    } else {
      beepCounter = 0;
    }
  }
}


bool ButtonControl::driverButtonPressed(){
    return false;
}

