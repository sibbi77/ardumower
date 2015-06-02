#ifndef ROBOT_H
#define ROBOT_H

// Ardumower robot

#include "arbitrator.h"
#include "behavior.h"


class RobotControl
{
  public:
    unsigned long loopCounter;   // loop counter
    int num_collision; // collision counter
    DriveForwardBehavior driveForwardBehavior;
    Arbitrator arbitrator;
    RobotControl();
    unsigned long nextPrintTime;
    virtual void run();
    virtual void setup();
    virtual void loop();
    virtual void print();
};


#endif

