/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  
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
/*
perimeter v2 receiver for Arduino sound sensors/LM386 using digital filter: matched filter - evaluates signal polarity of 'pulse3' signal on one ADC pin (for one coil)
 (for details see    http://wiki.ardumower.de/index.php?title=Perimeter_wire )

How to use it (example):    
  1. initialize ADC:        ADCMan.init(); 
  2. set perimeter pins:    Perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);  
  3. read perimeter:        int value = Perimeter.getMagnitude(0);  
    
*/

#ifndef MOWER_PERIMETER_H
#define MOWER_PERIMETER_H

#include <Arduino.h>
#include "perimeter.h"


class MowerPerimeter : public PerimeterControl
{
  public:
    MowerPerimeter();
    // set ADC pins
    void setPins(byte idx0Pin, byte idx1Pin);    
    // get perimeter magnitude
    virtual int getMagnitude(char coilIdx);    
    virtual int getSmoothMagnitude(char coilIdx);
    // inside perimeter (true) or outside (false)?  
    virtual bool isInside(char coilIdx);
    // perimeter signal timed out? (e.g. due to broken wire)
    virtual bool signalTimedOut(char coilIdx);
    virtual int getSignalMin(char coilIdx);
    virtual int getSignalMax(char coilIdx);    
    virtual int getSignalAvg(char coilIdx);
    virtual float getFilterQuality(char coilIdx); 
    void speedTest();
    int16_t timedOutIfBelowSmag;
    //int16_t timeOutSecIfNotInside;
    // use differential perimeter signal as input for the matched filter? 
    bool useDifferentialPerimeterSignal;
    // swap coil polarity?
    bool swapCoilPolarity;      
    char subSample;
    virtual void setup();
    virtual void run();
  private:
    unsigned long nextPerimeterTime;
    unsigned long lastInsideTime[2];
    byte idxPin[2]; // channel for idx
    int callCounter;
    int16_t mag [2]; // perimeter magnitude per channel
    float smoothMag[2];
    float filterQuality[2];
    int16_t signalMin[2];
    int16_t signalMax[2];
    int16_t signalAvg[2];    
    int signalCounter[2];    
    void matchedFilter(char coilIdx);
    int16_t corrFilter(int8_t *H, int8_t subsample, int16_t M, int8_t *ip, int16_t nPts, float &quality);
    void printADCMinMax(int8_t *samples);
};



#endif

