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

// Ardumower robot

#ifndef ROBOT_H
#define ROBOT_H


#include "arbitrator.h"
#include "behavior.h"


class RobotControl
{
  public:
    unsigned long loopCounter;   // loop counter
    int loopsPerSec; // loops per second
    int num_collision; // collision counter
    float motorMowCircleAbovePower;
    DriveForwardBehavior driveForwardBehavior;
    HitPerimeterBehavior hitPerimeterBehavior;
    HitObstacleBehavior hitObstacleBehavior;
    TrackingBehavior trackingBehavior;
    ChargingBehavior chargingBehavior;
    CircleBehavior circleBehavior;
    Arbitrator arbitrator;
    PID perimeterPID;
    RobotControl();
    unsigned long nextPrintTime;
    virtual void run();
    virtual char readKey();
    virtual void processKey(char key);
    virtual void setup();
    virtual void loop();
    virtual void print();
};


#endif

