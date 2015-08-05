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
#include "buzzer.h"


// beep code data
byte beepCodesTable[][6]={
  {BEEP_MUTE,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE},
  {BEEP_SHORT, BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE},
  {BEEP_SHORT, BEEP_MUTE, BEEP_SHORT, BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE},
  {BEEP_SHORT, BEEP_MUTE, BEEP_SHORT, BEEP_MUTE, BEEP_SHORT, BEEP_MUTE},
  {BEEP_LONG,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE},
  {BEEP_LONG,  BEEP_MUTE, BEEP_SHORT, BEEP_MUTE, BEEP_SHORT, BEEP_MUTE},
  {BEEP_LONG,  BEEP_MUTE, BEEP_LONG,  BEEP_MUTE, BEEP_MUTE,  BEEP_MUTE},
  {BEEP_LONG,  BEEP_MUTE, BEEP_SHORT, BEEP_MUTE, BEEP_LONG,  BEEP_MUTE},
};



BuzzerControl::BuzzerControl(){
  nextBeepTime = 0;
  beepCodeToneIdx = 6;
  beepCodeIdx = BC_SILENCE;
}

void BuzzerControl::setup(){
  Console.println(F("BuzzerControl::setup"));
}

// call this in main loop
void BuzzerControl::run(){
  if (millis() < nextBeepTime) return;
  if (beepCodeToneIdx >= 6) return;
  switch (beepCodesTable[beepCodeIdx][beepCodeToneIdx]){
    case BEEP_MUTE:
      driverNoTone();
      nextBeepTime = millis() + 400;
      if (beepCodeToneIdx == 5) nextBeepTime = millis() + 1500;
      break;
    case BEEP_SHORT:
      driverTone(440);
      nextBeepTime = millis() + 100;
      break;
    case BEEP_LONG:
      driverTone(440);
      nextBeepTime = millis() + 700;
      break;
  }
  beepCodeToneIdx++;
}

// play a 'melody' (non-blocking)
void BuzzerControl::play(int beepCode){
  beepCodeIdx = beepCode;
  beepCodeToneIdx = 0;
  nextBeepTime = 0;
}

bool BuzzerControl::isPlaying(){
  return ((millis() < nextBeepTime) || (beepCodeToneIdx < 6));
}

void BuzzerControl::driverNoTone(){
}

void BuzzerControl::driverTone(int frequencyHz){
}

