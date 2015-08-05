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
#include "perimeter.h"


PerimeterControl::PerimeterControl(){
  nextPerimeterTime = 0;
  //timedOutIfBelowSmag = 300;
  timeOutSecIfNotInside = 8;
  callCounter = 0;
  mag[0] = mag[1] = 0;
  smoothMag[0] = smoothMag[1] = 0;
  filterQuality[0] = filterQuality[1] = 0;
  signalCounter[0] = signalCounter[1] = -300;
  lastInsideTime[0] = lastInsideTime[1] = 0;
  enable = false;
}

void PerimeterControl::setup(){
  Console.println(F("PerimeterControl::setup"));
}

void PerimeterControl::run(){
}

int PerimeterControl::getMagnitude(char coilIdx){
  return mag[coilIdx];
}

int PerimeterControl::getSmoothMagnitude(char coilIdx){
  return smoothMag[coilIdx];
}

int PerimeterControl::getSignalMin(char coilIdx){
  return signalMin[coilIdx];
}

int PerimeterControl::getSignalMax(char coilIdx){
  return signalMax[coilIdx];
}

int PerimeterControl::getSignalAvg(char coilIdx){
  return signalAvg[coilIdx];
}


float PerimeterControl::getFilterQuality(char coilIdx){
  return filterQuality[coilIdx];
}

bool PerimeterControl::isInside(char coilIdx){
  return (signalCounter[coilIdx] < 0);
}


bool PerimeterControl::signalTimedOut(char coilIdx){
  /*if (getSmoothMagnitude(coilIdx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[coilIdx] > timeOutSecIfNotInside * 1000) return true;*/
  return false;
}




