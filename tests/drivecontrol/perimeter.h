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

// perimeter loop receiver

#ifndef PERIMETER_H
#define PERIMETER_H



// coil index
enum {
  COIL_LEFT,
  COIL_RIGHT,
};

class PerimeterControl
{
  public:
    PerimeterControl();
    bool enable; // enable ?
    // get perimeter magnitude
    virtual int getMagnitude(char coilIdx);
    virtual int getSmoothMagnitude(char coilIdx);
    // inside perimeter (true) or outside (false)?
    virtual bool isInside(char coilIdx);
    // perimeter signal timed out? (e.g. due to broken wire)
    bool signalTimedOut(char coilIdx);
    int getSignalMin(char coilIdx);
    int getSignalMax(char coilIdx);
    int getSignalAvg(char coilIdx);
    float getFilterQuality(char coilIdx);
    int timeOutSecIfNotInside;
    virtual void setup();
    virtual void run();
  private:
    unsigned long nextPerimeterTime;
    unsigned long lastInsideTime[2];
    char idxPin[2]; // channel for idx
    int callCounter;
    int mag [2]; // perimeter magnitude per channel
    float smoothMag[2];
    float filterQuality[2];
    int signalMin[2];
    int signalMax[2];
    int signalAvg[2];
    int signalCounter[2];
};


#endif

