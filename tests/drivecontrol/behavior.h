#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "common.h"


// subsumption architecture
// external resources:
//   http://www.convict.lu/Jeunes/Subsumption.htm
//   http://www.lejos.org/nxt/nxj/tutorial/Behaviors/BehaviorProgramming.htm
//   http://www.ibm.com/developerworks/library/j-robots/


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

#endif

