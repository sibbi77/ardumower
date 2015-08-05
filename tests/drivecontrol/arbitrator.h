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

/* Behavior Pattern Arbitrator
   the arbitrator decides 
   -if there is any behavior with higher priority requiring action (monitor)
   -if so, sets next behavior (run) 
*/                                               

#ifndef ARBITRATOR_H
#define ARBITRATOR_H

#include "behavior.h"

class Arbitrator
{
  public:
    int behaviorCount;    
    Behavior *activeBehavior;
    Behavior *nextBehavior;    
    int activeBehaviorIdx;
    int nextBehaviorIdx;    
    Arbitrator();
    virtual void addBehavior(Behavior *behavior);
    virtual void setNextBehavior(Behavior *behavior);
    virtual void run();
    virtual void monitor();
};





#endif

