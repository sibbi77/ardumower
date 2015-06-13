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

