#include "perimeter.h"
#include <limits.h>
#include "types.h"


PerimeterControl::PerimeterControl(){
  nextPerimeterTime = 0;
  timedOutIfBelowSmag = 300;
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
  if (getSmoothMagnitude(coilIdx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[coilIdx] > timeOutSecIfNotInside * 1000) return true;
  return false;
}




