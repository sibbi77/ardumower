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

// subsumption architecture
// external resources:
//   http://www.convict.lu/Jeunes/Subsumption.htm
//   http://www.lejos.org/nxt/nxj/tutorial/Behaviors/BehaviorProgramming.htm
//   http://www.ibm.com/developerworks/library/j-robots/


#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"


// abstract superclass
class Behavior
{
  public:
    String name;
    bool suppressed;
    bool enabled;
    Behavior() { suppressed = false; enabled = true; }
    virtual void enable(bool flag){
      if (enabled == flag) return;
      enabled = flag;
      Console.print(F("ENABLE "));
      Console.print(name);
      Console.print(" ");
      Console.println(enabled);
    }
    virtual void suppress() { suppressed = true; }
    virtual bool takeControl() {}
    virtual void action() {}
};


// robot is mowing and driving forward
class DriveForwardBehavior : public Behavior
{
  public:
    DriveForwardBehavior();
    virtual bool takeControl();
    virtual void action();
};


// robot hit perimeter
class HitPerimeterBehavior : public Behavior
{
  public:
    HitPerimeterBehavior();
    virtual bool takeControl();
    virtual void action();
};

// robot hit obstacle
class HitObstacleBehavior : public Behavior
{
  public:
    HitObstacleBehavior();
    virtual bool takeControl();
    virtual void action();
};

// robot tracking perimeter and going home
class TrackingBehavior : public Behavior
{
  public:
    TrackingBehavior();
    virtual bool takeControl();
    virtual void action();
};

// robot is charging
class ChargingBehavior : public Behavior
{
  public:
    ChargingBehavior();
    virtual bool takeControl();
    virtual void action();
};

// robot is mowing in circles
class CircleBehavior : public Behavior
{
  public:
    CircleBehavior();
    virtual bool takeControl();
    virtual void action();
};



#endif

